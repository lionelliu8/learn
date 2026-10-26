# 带标记批头6D链路

这是带标记的刚性组件估计，不是无标记批头识别。用户已允许临时加标记进行链路验证。

2026-10-26更新：用户暂无固定材料，批头会滑动且已离开安装轮廓。
目前已切到只测标记模式，不再使用名义外参输出批头位姿。
当前只能验证ID42标记自身的位姿；以下刚性组件步骤待真正固定后恢复。

## 打印与固定

打印 `bit_marker42_20mm_mount_A4.pdf`：A4、100%、关闭适应页面。
量黑框外边20 mm和页面100 mm刻度。沿60×65 mm虚线裁下，贴在平整硬卡上。
批头按25×5 mm轮廓侧躺固定，杆尾在左、刀头朝右，不可滚动或滑动。
用少量胶或胶带固定，保持标记完整、白色边距不被遮挡。标记与批头必须刚性连接。

字典DICT_4X4_50，ID42，与原7×5标定板的标记ID分开。

## 坐标与变换

`assembly_marker42.json` 定义 `T_marker_object`，将批头坐标点变换到标记坐标。

```
T_camera_object = T_camera_marker * T_marker_object
```

模板正向为杆尾左、刀头右。标记X向右、Y向上、Z朝纸面外；
批头坐标与标记方向对齐，原点为总长中心，+X指向刀头。
批头中心相对标记中心为 `[0,-0.024,0.002]` 米。
2 mm高度由4 mm对边高度的一半假定，胶层、安装误差与近似截面尚未独立标定。
`mount_independently_verified=0`，不能把按模板安装当成高精度外参标定。

相机坐标仍是X向右、Y向下、Z向前；输出是相机相对批头的坐标变换，不是机器人基座坐标。
相机内参继续使用已冻结的4K标定，不因加标记而重新标定。

## 板端程序

编译和主源码在Ubuntu `/home/llx/6D/native/src/main.cpp`。
新二进制单独部署为 `/userdata/6D-native/pose6d-mounted`，原程序保留。

```sh
cd /userdata/6D-native
./pose6d-mounted --self-test --rga
./pose6d-mounted --rga --width 640 --height 360 --threads 2 --frames 300 \
  --calibration calibration/camera.yaml --assembly part/assembly_marker42.json \
  --output UNIQUE_MARKER42_LOG.jsonl
```

不要与已运行的相机预览争用V4L2设备；需要先停止该预览。
MIPI预览启动方式如下，必须指定新二进制，否则旧程序不识别`--assembly`：

```sh
cd /userdata/6D-native
POSE6D_BINARY=./pose6d-mounted python3 view_live.py \
  --calibration calibration/camera.yaml --assembly part/assembly_marker42.json
```

处理链：V4L2原始4K NV12 -> RGA缩图 -> ArUco检测 -> 只处理ID42 ->
回到原始Y平面精修四角 -> 4K内参PnP -> 组合固定安装变换 -> 输出批头XYZ和rvec。
保持V4L2缓冲区到精修完成，不做整幅4K图的CPU复制。NPU未使用。
在JSON中，标记位姿为`markers[].xyz_m/rvec_rad`，批头位姿为`markers[].object_pose`。
未检测到42或位姿被拒绝时不输出有效批头位姿；不会沿用上一帧坐标。
单个方形标记仍有平面位姿歧义，实测应包含倾斜视角，不能仅凭残差小就宣称姿态正确。

## 已完成验证与当前边界

- 模板PDF已渲染检查，A4媒体框210×297 mm，标记黑框在矢量坐标中为20 mm。
- OpenCV生成的ID42图案解码检查通过。
- RK3568实际RGA自检：ID42旋转0/90/180/270度、固定偏移随标记旋转的检查通过。
- 真实相机60帧输入检查：只响应42的过滤与JSON检查通过；当时未看到42，未输出批头位姿。
- 原标定、原二进制均保留。硬件预览已切至新模式，等待打印组件进入画面。
- 物理固定组件的运动实验尚未完成，也没有独立位置/角度真值；当前不能宣称物理6D验证已完成。

## 下一轮物理实验

用户打印固定后，先倾斜20至30度、整卡入镜，短时静止采集。
再平移和分别改变三个方向的朝向，检查位置及旋转是否响应、标记丢失时是否停止输出。
稳定性、响应性与绝对精度分别报告；没有测量真值时不把稳定输出当作准确输出。

只测标记的启动方式（不产生批头坐标）：

```sh
cd /userdata/6D-native
POSE6D_BINARY=./pose6d-mounted POSE6D_LABEL='Marker only - bit not fixed' python3 view_live.py \
  --calibration calibration/camera.yaml --marker-id 42 --marker-length 0.020
```

只有标记纸保持平整即可观测标记本身；若与批头不刚性连接，就不能沿用二者的固定变换。

## 2026-10-26 批头模型投影检查

新源码增加了可选 `preview_geometry` 顶点/边投影，复用既有批头近似模型；单位由毫米转换为米。生成工具为 `prepare_bit_preview.py`，输出 `assembly_marker42_preview.json`，原安装配置保留。

Ubuntu源码和配置已经同步；板端独立二进制为 `/userdata/6D-native/pose6d-bit-preview`。RGA自检通过。随后完成30帧软件集成检查：模型配置读取、线框绘制路径、30个名义刚体变换计算通过；这不验证实际安装关系。试录保留在 `/userdata/6D-native/part/data/bit_preview_software_check_20261026_035129`，当时固定无效，输出不能用作实物批头坐标。

当前恢复纯标记预览，没有启用持续批头位姿输出。实物固定后再切换模型检查模式，逐项核对杆尾、刀尖与外形方向，确认中心偏移和轴定义后验证平移/旋转/遮挡恢复。胶层与装夹位置需测量，不把模板的名义偏移当实测外参。

单帧跳变排查：`replay_square_candidates.py`已在Ubuntu对全部959帧记录的原始4K角点进行双候选及局部优化回放。依然有一对相邻帧超过10度，且第583帧仍被2px门限拒绝。因此未替换在线求解器，也没有用滤波掩盖跳变。缺少189帧4K原图，无法断言模糊、角点或PnP歧义中的哪一项是唯一原因。
