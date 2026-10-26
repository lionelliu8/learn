#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <iostream>
#include <numeric>
#include <stdexcept>
#include <string>
#include <sstream>
#include <utility>
#include <vector>
#include <linux/videodev2.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <im2d.h>
#include <RgaUtils.h>
#include <rga.h>
#include "monocular_6d_pose/pose_solver.hpp"
#include "monocular_6d_pose/resize.hpp"

using Clock = std::chrono::steady_clock;
volatile std::sig_atomic_t running = 1;
volatile std::sig_atomic_t snapshot_requested = 0;
void stop(int) {running = 0;}
void request_snapshot(int) {snapshot_requested = 1;}
double ms(Clock::duration d) {return std::chrono::duration<double, std::milli>(d).count();}
int call_ioctl(int fd, unsigned long request, void* argument) {
  int result;
  do {result = ioctl(fd, request, argument);} while (result < 0 && errno == EINTR);
  return result;
}
void require(bool ok, const std::string& message) {
  if (!ok) {throw std::runtime_error(message);}
}

void json_string(std::ostream& out,const std::string& value) {
  static const char* hex="0123456789abcdef";
  out<<'"';
  for(unsigned char c:value){
    if(c=='"'||c=='\\')out<<'\\'<<char(c);
    else if(c<32)out<<"\\u00"<<hex[c>>4]<<hex[c&15];
    else out<<char(c);
  }
  out<<'"';
}

void json_corners(std::ostream& out,const std::vector<cv::Point2f>& points) {
  out<<'[';
  for(size_t i=0;i<points.size();++i){if(i)out<<',';out<<'['<<points[i].x<<','<<points[i].y<<']';}
  out<<']';
}

struct FrameBuffer {void* address=MAP_FAILED; size_t length=0; int dma_fd=-1;};
class Camera {
  int fd=-1;
  bool streaming=false;
public:
  int width=0, height=0, stride=0;
  unsigned dropped=0;
  unsigned sequence=0;
  long long timestamp_us=0;
  bool timestamp_monotonic=false;
  std::vector<FrameBuffer> buffers;
  explicit Camera(const std::string& device) {
    try {
      fd=open(device.c_str(),O_RDWR|O_NONBLOCK|O_CLOEXEC);
      require(fd>=0,"open camera: "+std::string(strerror(errno)));
      v4l2_format format{};
      format.type=V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
      require(call_ioctl(fd,VIDIOC_G_FMT,&format)==0,"VIDIOC_G_FMT failed");
      auto& p=format.fmt.pix_mp;
      require(p.pixelformat==V4L2_PIX_FMT_NV12 && p.num_planes==1,
              "expected single-plane NV12 in V4L2 multiplanar API; not NV12M");
      width=p.width; height=p.height; stride=p.plane_fmt[0].bytesperline;
      require(width>0 && height>0 && width%2==0 && height%2==0 && stride>=width,
              "invalid NV12 geometry");
      // NV12 chroma offset assumes no extra vertical padding. Board is verified at 3840x2160.
      require(p.plane_fmt[0].sizeimage==size_t(stride)*height*3/2,
              "unsupported NV12 vertical padding/sizeimage");
      v4l2_requestbuffers request{};
      request.count=4;request.type=format.type;request.memory=V4L2_MEMORY_MMAP;
      require(call_ioctl(fd,VIDIOC_REQBUFS,&request)==0 && request.count>=2,"REQBUFS failed");
      buffers.resize(request.count);
      for (unsigned i=0;i<request.count;++i) {
        v4l2_plane plane{};
        v4l2_buffer buffer{};
        buffer.type=format.type;buffer.memory=V4L2_MEMORY_MMAP;
        buffer.index=i;buffer.length=1;buffer.m.planes=&plane;
        require(call_ioctl(fd,VIDIOC_QUERYBUF,&buffer)==0,"QUERYBUF failed");
        auto& mapped=buffers[i];
        mapped.length=plane.length;
        mapped.address=mmap(nullptr,plane.length,PROT_READ|PROT_WRITE,MAP_SHARED,fd,plane.m.mem_offset);
        require(mapped.address!=MAP_FAILED,"mmap failed");
        v4l2_exportbuffer exported{};
        exported.type=format.type;exported.index=i;exported.plane=0;exported.flags=O_CLOEXEC;
        if(call_ioctl(fd,VIDIOC_EXPBUF,&exported)==0) {mapped.dma_fd=exported.fd;}
        queue(i);
      }
      auto type=V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
      require(call_ioctl(fd,VIDIOC_STREAMON,&type)==0,"STREAMON failed");
      streaming=true;
      std::cout<<"camera="<<device<<" "<<width<<"x"<<height<<" stride="<<stride
               <<" buffers="<<buffers.size()<<" source_dma="<<(buffers[0].dma_fd>=0)<<std::endl;
    } catch(...) {close_all();throw;}
  }
  Camera(const Camera&)=delete;
  Camera& operator=(const Camera&)=delete;
  ~Camera(){close_all();}
  void close_all() {
    if(streaming){auto type=V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;call_ioctl(fd,VIDIOC_STREAMOFF,&type);streaming=false;}
    for(auto& b:buffers){if(b.address!=MAP_FAILED){munmap(b.address,b.length);b.address=MAP_FAILED;}
      if(b.dma_fd>=0){close(b.dma_fd);b.dma_fd=-1;}}
    if(fd>=0){close(fd);fd=-1;}
  }
  void queue(unsigned index) {
    v4l2_plane plane{};
    v4l2_buffer b{};b.type=V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;b.memory=V4L2_MEMORY_MMAP;
    b.index=index;b.length=1;b.m.planes=&plane;
    require(call_ioctl(fd,VIDIOC_QBUF,&b)==0,"QBUF failed");
  }
  unsigned latest() {
    pollfd p{fd,POLLIN,0};
    int status;
    do {status=poll(&p,1,5000);} while(status<0&&errno==EINTR&&running);
    require(status>0 && !(p.revents&(POLLERR|POLLNVAL)),"camera poll timeout/error");
    int latest=-1;
    // Bound drain iterations so a fast camera cannot starve processing.
    for(size_t i=0;i<buffers.size();++i){
      v4l2_plane plane{};
      v4l2_buffer b{};b.type=V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;b.memory=V4L2_MEMORY_MMAP;
      b.length=1;b.m.planes=&plane;
      if(call_ioctl(fd,VIDIOC_DQBUF,&b)<0){if(errno==EAGAIN)break;throw std::runtime_error("DQBUF failed");}
      require(b.index<buffers.size(),"invalid buffer index");
      if(b.flags&V4L2_BUF_FLAG_ERROR){queue(b.index);++dropped;continue;}
      if(latest>=0){queue(latest);++dropped;}
      latest=b.index;
      require(plane.bytesused>=size_t(stride)*height*3/2,"short NV12 frame");
      sequence=b.sequence;
      timestamp_us=static_cast<long long>(b.timestamp.tv_sec)*1000000+b.timestamp.tv_usec;
      timestamp_monotonic=(b.flags&V4L2_BUF_FLAG_TIMESTAMP_MASK)==V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
    }
    require(latest>=0,"no valid frame");return unsigned(latest);
  }
};

void preprocess(void* source,int dma_fd,int width,int height,int stride,
                cv::Mat& nv12,cv::Mat& gray,bool rga) {
  if(!rga){
    cv::Mat y(height,width,CV_8U,source,stride);
    cv::resize(y,gray,gray.size(),0,0,cv::INTER_LINEAR);
    return;
  }
  auto src=dma_fd>=0 ? wrapbuffer_fd_t(dma_fd,width,height,stride,height,RK_FORMAT_YCbCr_420_SP)
    : wrapbuffer_virtualaddr_t(source,width,height,stride,height,RK_FORMAT_YCbCr_420_SP);
  auto dst=wrapbuffer_virtualaddr_t(nv12.data,gray.cols,gray.rows,gray.cols,gray.rows,RK_FORMAT_YCbCr_420_SP);
  im_rect empty{};
  rga_buffer_t pattern{};
  auto status=imcheck_t(src,dst,pattern,empty,empty,empty,0);
  require(status==IM_STATUS_NOERROR,std::string("RGA check: ")+imStrError_t(status));
  status=imresize_t(src,dst,0,0,1,1);
  require(status==IM_STATUS_SUCCESS,std::string("RGA resize: ")+imStrError_t(status));
  gray=cv::Mat(gray.rows,gray.cols,CV_8U,nv12.data);
}

monocular_6d_pose::PoseResult pose(const std::vector<cv::Point2f>& points,
                                const cv::Mat& K,const cv::Mat& D,double length) {
  auto result=monocular_6d_pose::solve_square_pose(points,K,D,length);
  if(result.reprojection_error>2.0){
    const float h=length/2;
    std::vector<cv::Point3f> obj={{-h,h,0},{h,h,0},{h,-h,0},{-h,-h,0}};
    require(cv::solvePnP(obj,points,K,D,result.rotation,result.translation,false,cv::SOLVEPNP_ITERATIVE),
            "fallback PnP failed");
    std::vector<cv::Point2f> projected;
    cv::projectPoints(obj,result.rotation,result.translation,K,D,projected);
    result.reprojection_error=0;
    for(size_t i=0;i<4;++i)result.reprojection_error+=cv::norm(points[i]-projected[i]);
    result.reprojection_error/=4;
  }
  require(cv::checkRange(cv::Mat(result.rotation))&&cv::checkRange(cv::Mat(result.translation))&&
          result.translation[2]>0 && result.reprojection_error<=2.0,"rejected invalid/high-error pose");
  return result;
}

monocular_6d_pose::PoseResult mounted_object_pose(
    const monocular_6d_pose::PoseResult& marker, const cv::Matx33d& marker_R_object,
    const cv::Vec3d& marker_t_object) {
  cv::Matx33d camera_R_marker;
  cv::Rodrigues(marker.rotation,camera_R_marker);
  monocular_6d_pose::PoseResult object=marker;
  cv::Rodrigues(camera_R_marker*marker_R_object,object.rotation);
  object.translation=camera_R_marker*marker_t_object+marker.translation;
  require(cv::checkRange(cv::Mat(object.rotation))&&cv::checkRange(cv::Mat(object.translation))&&
          object.translation[2]>0,"invalid mounted-object pose");
  return object;
}

void save_pgm(const std::string& filename,const cv::Mat& gray) {
  std::ofstream file(filename,std::ios::binary);
  require(bool(file),"cannot open preview file");
  file<<"P5\n"<<gray.cols<<" "<<gray.rows<<"\n255\n";
  for(int row=0;row<gray.rows;++row)file.write(reinterpret_cast<const char*>(gray.ptr(row)),gray.cols);
  file.close();
  require(bool(file),"cannot write preview file");
}

void self_test(bool rga) {
  std::ostringstream escaped;json_string(escaped,"quote\" slash\\ line\n");
  require(escaped.str()=="\"quote\\\" slash\\\\ line\\u000a\"","diagnostic JSON escaping failed");
  monocular_6d_pose::PoseResult rotated_marker{};
  rotated_marker.rotation=cv::Vec3d(0,0,CV_PI/2);
  rotated_marker.translation=cv::Vec3d(0,0,.3);
  auto mounted=mounted_object_pose(rotated_marker,cv::Matx33d::eye(),{0,-.024,.002});
  require(cv::norm(mounted.translation-cv::Vec3d(.024,0,.302))<1e-9,
          "rigid mount transform must rotate its translation offset");
  auto dict=cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
  cv::Mat K=(cv::Mat_<double>(3,3)<<600,0,320,0,600,240,0,0,1),D=cv::Mat::zeros(1,5,CV_64F);
  for(int turn=0;turn<4;++turn){
    cv::Mat marker;cv::aruco::drawMarker(dict,0,200,marker);
    for(int i=0;i<turn;++i)cv::rotate(marker,marker,cv::ROTATE_90_CLOCKWISE);
    cv::Mat source(480*3/2,640,CV_8U,cv::Scalar(128));
    source.rowRange(0,480).setTo(255);marker.copyTo(source(cv::Rect(220,140,200,200)));
    cv::Mat nv12(240*3/2,320,CV_8U),gray(240,320,CV_8U);
    preprocess(source.data,-1,640,480,640,nv12,gray,rga);
    std::vector<int> ids;std::vector<std::vector<cv::Point2f>> corners;
    auto params=cv::aruco::DetectorParameters::create();params->cornerRefinementMethod=cv::aruco::CORNER_REFINE_SUBPIX;
    cv::aruco::detectMarkers(gray,dict,corners,ids,params);
    require(ids.size()==1 && ids[0]==0,"self-test marker detection failed");
    auto scaled=monocular_6d_pose::scaled_intrinsics(K,{640,480},gray.size());
    auto result=pose(corners[0],scaled,D,.08);
    require(std::abs(result.translation[2]-.24)<.008,"self-test translation error");
    std::cout<<"rotation="<<turn*90<<" z="<<result.translation[2]
             <<" reprojection_px="<<result.reprojection_error<<std::endl;
  }
  for(int turn=0;turn<4;++turn){
    cv::Mat marker;cv::aruco::drawMarker(dict,42,80,marker);
    for(int i=0;i<turn;++i)cv::rotate(marker,marker,cv::ROTATE_90_CLOCKWISE);
    cv::Mat source(480*3/2,640,CV_8U,cv::Scalar(128));
    source.rowRange(0,480).setTo(255);marker.copyTo(source(cv::Rect(280,200,80,80)));
    cv::Mat nv12(240*3/2,320,CV_8U),gray(240,320,CV_8U);
    preprocess(source.data,-1,640,480,640,nv12,gray,rga);
    std::vector<int> ids;std::vector<std::vector<cv::Point2f>> corners;
    auto params=cv::aruco::DetectorParameters::create();params->cornerRefinementMethod=cv::aruco::CORNER_REFINE_SUBPIX;
    cv::aruco::detectMarkers(gray,dict,corners,ids,params);
    require(ids.size()==1&&ids[0]==42,"assembly ID42 detection self-test failed");
    auto scaled=monocular_6d_pose::scaled_intrinsics(K,{640,480},gray.size());
    auto marker_pose=pose(corners[0],scaled,D,.020);
    auto object=mounted_object_pose(marker_pose,cv::Matx33d::eye(),{0,-.024,.002});
    double angle=turn*CV_PI/2;
    cv::Vec3d expected(-.024*std::sin(angle),.024*std::cos(angle),.148);
    require(cv::norm(object.translation-expected)<.003,"rotated marker42 rigid object transform failed");
    std::cout<<"ASSEMBLY_TEST rotation="<<turn*90<<" object_z="<<object.translation[2]<<std::endl;
  }
  std::cout<<"SELF_TEST_PASS backend="<<(rga?"rga":"cpu")<<std::endl;
}

int main(int argc,char** argv) {
  try {
    std::string device="/dev/video0",calibration,output="poses.jsonl",snapshot_dir,assembly,record_dir;
    int frames=120,detect_width=960,detect_height=540,threads=1,target_id=-1;
    bool rga=false,test=false,preview=false,charuco=false;double length=.08;
    for(int i=1;i<argc;++i){
      std::string a=argv[i];
      if(a=="--self-test"){test=true;continue;}
      if(a=="--rga"){rga=true;continue;}
      if(a=="--preview"){preview=true;continue;}
      if(a=="--charuco"){charuco=true;continue;}
      if(a=="--help"){
        std::cout<<"pose6d [--self-test] [--rga] [--device /dev/video0] [--frames 120] "
          "[--calibration camera.yaml] [--marker-length 0.08] [--width 960] [--height 540] "
          "[--threads 1] [--output poses.jsonl] [--preview] [--charuco] [--marker-id 42] [--assembly assembly.json] [--snapshot-dir existing_directory] [--record-dir NEW_DIRECTORY] "
          "(frames=0: continuous; SIGUSR1: save original Y frame)"<<std::endl;return 0;
      }
      require(i+1<argc,"missing argument for "+a);std::string v=argv[++i];
      if(a=="--device")device=v;else if(a=="--calibration")calibration=v;
      else if(a=="--frames")frames=std::stoi(v);else if(a=="--width")detect_width=std::stoi(v);
      else if(a=="--height")detect_height=std::stoi(v);else if(a=="--threads")threads=std::stoi(v);
      else if(a=="--marker-length")length=std::stod(v);else if(a=="--output")output=v;
      else if(a=="--snapshot-dir")snapshot_dir=v;
      else if(a=="--assembly")assembly=v;
      else if(a=="--marker-id")target_id=std::stoi(v);
      else if(a=="--record-dir")record_dir=v;
      else throw std::invalid_argument("unknown argument "+a);
    }
    require(frames>=0&&detect_width>0&&detect_width%16==0&&detect_height>0&&detect_height%2==0&&
            threads>0&&std::isfinite(length)&&length>0&&target_id>=-1&&target_id<50,"invalid dimensions/count/length/marker id");
    require(!(charuco&&target_id>=0),"--marker-id cannot be combined with --charuco");
    if(!record_dir.empty()){
      require(frames>0&&frames<=1800,"diagnostic recording requires1..1800 frames");
      require(mkdir(record_dir.c_str(),0755)==0,"record-dir must be a new directory with an existing parent");
    }
    cv::setNumThreads(threads);std::signal(SIGINT,stop);std::signal(SIGTERM,stop);
    if(!snapshot_dir.empty()){
      struct stat info{};
      require(stat(snapshot_dir.c_str(),&info)==0 && S_ISDIR(info.st_mode),"snapshot directory must exist");
      std::signal(SIGUSR1,request_snapshot);
    }
    if(test){self_test(rga);return 0;}
    Camera camera(device);
    cv::Mat K,D,fullK;
    if(!calibration.empty()){
      cv::FileStorage file(calibration,cv::FileStorage::READ);
      require(file.isOpened(),"cannot open calibration");
      int w=0,h=0;file["image_width"]>>w;file["image_height"]>>h;
      require(w==camera.width&&h==camera.height,"calibration resolution mismatch");
      std::string model;file["distortion_model"]>>model;
      require(model=="plumb_bob"||model=="rational_polynomial","unsupported distortion model");
      std::vector<double> values,coefficients;
      file["camera_matrix"]["data"]>>values;file["distortion_coefficients"]["data"]>>coefficients;
      require(values.size()==9&&(coefficients.size()==5||coefficients.size()==8),"invalid calibration dimensions");
      K=cv::Mat(3,3,CV_64F,values.data()).clone();D=cv::Mat(coefficients,true);
      fullK=K.clone();
      require(cv::checkRange(K)&&cv::checkRange(D)&&K.at<double>(0,0)>0&&K.at<double>(1,1)>0,
              "invalid calibration values");
      K=monocular_6d_pose::scaled_intrinsics(K,{camera.width,camera.height},{detect_width,detect_height});
    }else std::cout<<"NO_CALIBRATION: marker detection only; metric pose disabled"<<std::endl;
    require(!charuco || !fullK.empty(),"--charuco requires calibration");
    bool mount_verified=false;
    std::vector<cv::Point3f> object_preview_points;
    std::vector<int> object_preview_edges;
    cv::Matx33d marker_R_object=cv::Matx33d::eye();cv::Vec3d marker_t_object(0,0,0);
    if(!assembly.empty()){
      require(!fullK.empty()&&!charuco,"assembly mode requires calibration and cannot use --charuco");
      cv::FileStorage file(assembly,cv::FileStorage::READ);
      require(file.isOpened(),"cannot read assembly file");
      int requested_id=target_id;
      file["marker_id"]>>target_id;file["marker_length_m"]>>length;
      require(requested_id<0||requested_id==target_id,"marker id conflicts with assembly file");
      std::vector<double> rotation,translation;
      file["T_marker_object"]["rotation_matrix"]>>rotation;
      file["T_marker_object"]["translation_m"]>>translation;
      int verified=0;file["mount_independently_verified"]>>verified;mount_verified=verified!=0;
      if(!file["preview_geometry"].empty()){
        std::vector<double> vertices;
        file["preview_geometry"]["vertices_m"]>>vertices;
        file["preview_geometry"]["edges"]>>object_preview_edges;
        require(!vertices.empty()&&vertices.size()%3==0&&object_preview_edges.size()%2==0,
                "invalid object preview geometry");
        for(size_t i=0;i<vertices.size();i+=3){
          require(std::isfinite(vertices[i])&&std::isfinite(vertices[i+1])&&std::isfinite(vertices[i+2]),
                  "nonfinite object preview vertex");
          object_preview_points.emplace_back(vertices[i],vertices[i+1],vertices[i+2]);
        }
        for(int edge:object_preview_edges)require(edge>=0&&size_t(edge)<object_preview_points.size(),
                                                "object preview edge out of range");
      }
      require(target_id>=0&&target_id<50&&std::isfinite(length)&&length>0&&
              rotation.size()==9&&translation.size()==3,"invalid assembly dimensions/marker");
      std::copy(rotation.begin(),rotation.end(),marker_R_object.val);
      std::copy(translation.begin(),translation.end(),marker_t_object.val);
      require(cv::checkRange(cv::Mat(marker_R_object))&&cv::checkRange(cv::Mat(marker_t_object))&&
              cv::norm(cv::Mat(marker_R_object.t()*marker_R_object-cv::Matx33d::eye()))<1e-6&&
              std::abs(cv::determinant(cv::Mat(marker_R_object))-1)<1e-6,"invalid rigid assembly transform");
    }
    require(target_id<0||!fullK.empty(),"target marker pose requires calibration");
    cv::Mat destination(detect_height*3/2,detect_width,CV_8U),gray(detect_height,detect_width,CV_8U);
    auto dict=cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    auto board=cv::aruco::CharucoBoard::create(7,5,.030f,.022f,dict);
    auto parameters=cv::aruco::DetectorParameters::create();
    parameters->cornerRefinementMethod=cv::aruco::CORNER_REFINE_SUBPIX;
    std::ofstream log(output);require(bool(log),"cannot open output");
    std::vector<double> timings;uint64_t processed=0,detected=0;auto run_start=Clock::now();
    for(uint64_t index=0;running&&(frames==0||index<uint64_t(frames));++index){
      unsigned current=camera.latest();auto start=Clock::now();
      cv::Mat original;
      try{
        // Hold the dequeued V4L2 buffer until full-resolution refinement finishes.
        if(charuco||target_id>=0)original=cv::Mat(camera.height,camera.width,CV_8U,
          camera.buffers[current].address,camera.stride);
        if(snapshot_requested){
          snapshot_requested=0;
          const std::string filename=snapshot_dir+"/capture_"+std::to_string(getpid())+"_"+
            std::to_string(Clock::now().time_since_epoch().count())+".pgm";
          cv::Mat original(camera.height,camera.width,CV_8U,camera.buffers[current].address,camera.stride);
          save_pgm(filename+".tmp",original);
          require(std::rename((filename+".tmp").c_str(),filename.c_str())==0,"snapshot rename failed");
          std::cout<<"SNAPSHOT_SAVED "<<filename<<" "<<camera.width<<"x"<<camera.height<<std::endl;
        }
        preprocess(camera.buffers[current].address,camera.buffers[current].dma_fd,
          camera.width,camera.height,camera.stride,destination,gray,rga);
      }
      catch(...){camera.queue(current);throw;}
      if(!charuco&&target_id<0)camera.queue(current);
      auto resized=Clock::now();
      std::vector<int> ids;std::vector<std::vector<cv::Point2f>> corners,rejected;
      cv::aruco::detectMarkers(gray,dict,corners,ids,parameters,rejected);auto detected_at=Clock::now();
      log<<"{\"frame\":"<<index<<",\"capture_sequence\":"<<camera.sequence
         <<",\"capture_timestamp_us\":"<<camera.timestamp_us<<",\"capture_timestamp_clock\":\""
         <<(camera.timestamp_monotonic?"monotonic":"unspecified")<<"\",\"detector_width\":"<<gray.cols
         <<",\"detector_height\":"<<gray.rows<<",\"backend\":\""<<(rga?"rga":"cpu")<<"\",\"markers\":[";
      bool emitted_marker=false;
      std::vector<std::pair<cv::Vec3d,cv::Vec3d>> preview_axes;
      for(size_t n=0;n<ids.size();++n){
        if(target_id>=0&&ids[n]!=target_id)continue;
        if(emitted_marker){log<<",";}emitted_marker=true;
        log<<"{\"id\":"<<ids[n];
        if(!record_dir.empty()){
          log<<",\"corners_detector_px\":";json_corners(log,corners[n]);
        }
        if(!K.empty()){
          try{
            auto pose_corners=corners[n];auto pose_K=K;
            if(target_id>=0){
              for(auto& p:pose_corners){
                p.x=(p.x+.5f)*camera.width/detect_width-.5f;
                p.y=(p.y+.5f)*camera.height/detect_height-.5f;
              }
              cv::cornerSubPix(original,pose_corners,{5,5},{-1,-1},
                {cv::TermCriteria::COUNT|cv::TermCriteria::EPS,30,.01});
              pose_K=fullK;
              if(!record_dir.empty()){
                log<<",\"corners_sensor_px\":";json_corners(log,pose_corners);
              }
            }
            auto result=pose(pose_corners,pose_K,D,length);
            log<<",\"xyz_m\":["<<result.translation[0]<<","<<result.translation[1]<<","<<result.translation[2]
               <<"],\"rvec_rad\":["<<result.rotation[0]<<","<<result.rotation[1]<<","<<result.rotation[2]
               <<"],\"reprojection_px\":"<<result.reprojection_error;
            if(target_id>=0)log<<",\"reprojection_coordinate_space\":\"original_4k\"";
            if(!assembly.empty()){
              auto object=mounted_object_pose(result,marker_R_object,marker_t_object);
              log<<",\"object_pose\":{\"source\":\"rigid_marker_mount\",\"xyz_m\":["
                 <<object.translation[0]<<","<<object.translation[1]<<","<<object.translation[2]
                 <<"],\"rvec_rad\":["<<object.rotation[0]<<","<<object.rotation[1]<<","<<object.rotation[2]
                 <<"],\"mount_independently_verified\":"<<(mount_verified?"true":"false")<<"}";
              preview_axes.emplace_back(object.rotation,object.translation);
            }else if(target_id>=0)preview_axes.emplace_back(result.rotation,result.translation);
          }catch(const std::exception& e){
            log<<",\"pose_rejected\":true,\"pose_rejection_reason\":";json_string(log,e.what());
          }
        }
        log<<"}";
      }
      log<<"]";
      if(charuco){
        auto fullCorners=corners;
        for(auto& marker:fullCorners){
          for(auto& p:marker){
            p.x=(p.x+.5f)*camera.width/detect_width-.5f;
            p.y=(p.y+.5f)*camera.height/detect_height-.5f;
          }
          cv::cornerSubPix(original,marker,{5,5},{-1,-1},
            {cv::TermCriteria::COUNT|cv::TermCriteria::EPS,30,.01});
        }
        cv::Mat points,cornerIds;
        if(!ids.empty())cv::aruco::interpolateCornersCharuco(
          fullCorners,ids,original,board,points,cornerIds,fullK,D);
        log<<",\"board\":{\"corners\":"<<cornerIds.total();
        bool valid=false;
        if(cornerIds.total()>=12){
          std::vector<cv::Point3f> objects;
          for(int j=0;j<cornerIds.rows;++j)objects.push_back(board->chessboardCorners.at(cornerIds.at<int>(j)));
          cv::Vec3d rv,tv;
          if(cv::solvePnP(objects,points,fullK,D,rv,tv)){
            std::vector<cv::Point2f> projected;
            cv::projectPoints(objects,rv,tv,fullK,D,projected);
            double squared=0;
            for(size_t j=0;j<projected.size();++j){auto delta=projected[j]-points.at<cv::Point2f>(int(j));squared+=delta.dot(delta);}
            double rms=std::sqrt(squared/projected.size());
            valid=cv::checkRange(cv::Mat(rv))&&cv::checkRange(cv::Mat(tv))&&tv[2]>0&&std::isfinite(rms)&&rms<=2.0;
            log<<",\"rms_4k_px\":"<<rms;
            if(valid){
              cv::Matx33d rotation;cv::Rodrigues(rv,rotation);
              auto center=rotation*cv::Vec3d(.105,.075,0)+tv;
              log<<",\"center_xyz_m\":["<<center[0]<<","<<center[1]<<","<<center[2]
                 <<"],\"rvec_rad\":["<<rv[0]<<","<<rv[1]<<","<<rv[2]<<"]";
            }
          }
        }
        log<<",\"valid\":"<<(valid?"true":"false")<<"}";
      }
      if(charuco||target_id>=0)camera.queue(current);
      auto end=Clock::now();double total=ms(end-start);
      if(timings.size()<4096)timings.push_back(total);else timings[processed%4096]=total;
      ++processed;
      if(target_id<0?!ids.empty():emitted_marker)++detected;
      double record_ms=0;
      if(!record_dir.empty()){
        std::string name="frame_"+std::to_string(index)+".pgm";
        auto before_record=Clock::now();
        save_pgm(record_dir+"/"+name,gray); // Exactly the input used by detectMarkers, before overlays.
        record_ms=ms(Clock::now()-before_record);
        log<<",\"record_image\":";json_string(log,name);
        log<<",\"decoded_ids\":[";
        for(size_t i=0;i<ids.size();++i){if(i)log<<',';log<<ids[i];}
        log<<"],\"rejected_quad_count\":"<<rejected.size()<<",\"rejected_quads_top20\":[";
        std::sort(rejected.begin(),rejected.end(),[](const auto& a,const auto& b){return std::abs(cv::contourArea(a))>std::abs(cv::contourArea(b));});
        for(size_t i=0;i<std::min<size_t>(20,rejected.size());++i){if(i)log<<',';json_corners(log,rejected[i]);}
        log<<']';
      }
      log<<",\"resize_ms\":"<<ms(resized-start)<<",\"detect_ms\":"<<ms(detected_at-resized)
         <<",\"pose_ms\":"<<ms(end-detected_at)<<",\"processing_ms\":"<<total<<",\"record_ms\":"<<record_ms<<"}\n";
      if(index%30==0){log.flush();std::cout<<"frame="<<index<<" markers="<<ids.size()
        <<" resize_ms="<<ms(resized-start)<<" detect_ms="<<ms(detected_at-resized)<<" processing_ms="<<total<<std::endl;
      }
      if(preview && index%3==0){
        cv::Mat view=gray.clone();
        if(!ids.empty())cv::aruco::drawDetectedMarkers(view,corners,ids,cv::Scalar(255));
        for(const auto& axes:preview_axes){
          // Latest PGM remains grayscale; XYZ mm label is object-frame origin in camera coordinates.
          cv::drawFrameAxes(view,K,D,axes.first,axes.second,.01,1);
          if(!object_preview_points.empty()){
            std::vector<cv::Point2f> projected;
            cv::projectPoints(object_preview_points,axes.first,axes.second,K,D,projected);
            for(size_t i=0;i<object_preview_edges.size();i+=2){
              const auto& a=projected[object_preview_edges[i]];
              const auto& b=projected[object_preview_edges[i+1]];
              if(std::isfinite(a.x)&&std::isfinite(a.y)&&std::isfinite(b.x)&&std::isfinite(b.y)&&
                 std::abs(a.x)<100000&&std::abs(a.y)<100000&&std::abs(b.x)<100000&&std::abs(b.y)<100000){
                cv::line(view,a,b,cv::Scalar(255),1,cv::LINE_AA);
              }
            }
            cv::putText(view,"MODEL CHECK - mount must be fixed",{10,45},
                        cv::FONT_HERSHEY_SIMPLEX,.4,cv::Scalar(255),1,cv::LINE_AA);
          }
          std::string label=std::string(assembly.empty()?"TAG XYZ mm ":"BIT XYZ mm ")+std::to_string(int(std::lround(axes.second[0]*1000)))+" "+
            std::to_string(int(std::lround(axes.second[1]*1000)))+" "+std::to_string(int(std::lround(axes.second[2]*1000)));
          cv::putText(view,label,{10,25},cv::FONT_HERSHEY_SIMPLEX,.45,cv::Scalar(255),1,cv::LINE_AA);
        }
        save_pgm("latest.pgm.tmp",view);
        require(std::rename("latest.pgm.tmp","latest.pgm")==0,"preview rename failed");
      }
    }
    const double seconds=std::chrono::duration<double>(Clock::now()-run_start).count();
    std::sort(timings.begin(),timings.end());
    if(!timings.empty())std::cout<<"SUMMARY frames="<<processed<<" detected_frames="<<detected
      <<" dropped="<<camera.dropped<<" output_fps="<<processed/seconds
      <<" processing_p50_ms="<<timings[(timings.size()-1)/2]
      <<" processing_p95_ms="<<timings[size_t((timings.size()-1)*.95)]<<std::endl;
    return 0;
  }catch(const std::exception& e){
    if(!running){std::cout<<"Stopped; camera resources released"<<std::endl;return 0;}
    std::cerr<<"ERROR: "<<e.what()<<std::endl;return 1;
  }
}
