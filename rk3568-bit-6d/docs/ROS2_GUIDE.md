# 单目6D位姿估计系统

性能版本说明及RK3568的RGA编译、A/B测试命令见`RK3568.md`。为减少CPU与内存开销，调试图默认关闭；需要标注画面时运行`ros2 run monocular_6d_pose pose_node --ros-args -p publish_debug:=true`并启动图像订阅器。默认检测分辨率为320×240，程序会同步缩放相机内参。

```text
RGB图像 + CameraInfo
        ↓
ArUco四角检测
        ↓
已知标记尺寸 + solvePnP(IPPE)
        ↓
平移XYZ + 旋转四元数
        ↓
PoseStamped + TF + 标注图
```

## 编译与测试

```bash
cd /home/llx/6D
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon test --event-handlers console_direct+
colcon test-result --verbose
source install/setup.sh
```

## 无相机演示

```bash
ros2 launch monocular_6d_pose demo.launch.py
```

新终端查看位姿：

```bash
source /opt/ros/humble/setup.bash
source /home/llx/6D/install/setup.sh
ros2 topic echo /object_pose --once
```

查看标注画面：

```bash
rqt_image_view /pose_debug_image
```

合成相机内参为 `fx=fy=600, cx=320, cy=240`，标记边长为80mm，画面中的标记宽200像素，因此正视距离应约为 `600×0.08÷200=0.24m`。

## 接真实相机

真实相机驱动需要发布：

- `/camera/image_raw`：`sensor_msgs/Image`
- `/camera/camera_info`：标定后的`sensor_msgs/CameraInfo`

确认`config/pose.yaml`中的`marker_length`与打印后实际测量的黑色外边长一致，再运行：

```bash
ros2 launch monocular_6d_pose camera.launch.py
```

输出：

- `/object_pose`：相机坐标系下目标6D位姿
- `/pose_debug_image`：角点、坐标轴和XYZ标注
- `/tf`：`camera_optical_frame → detected_object`

## RK3568移植

虚拟机是x86_64，RK3568通常是aarch64，不能复制`build/`和`install/`中的二进制。只复制`src/`、`README.md`，在板端安装ROS2/OpenCV后重新执行`colcon build`。PnP和ArUco运行在CPU；后续将ArUco替换为RKNN关键点模型时，ROS2位姿发布部分无需改写。
