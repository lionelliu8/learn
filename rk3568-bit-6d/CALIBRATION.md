# 单目棋盘格离线标定

工具：`/home/llx/6D/src/monocular_6d_pose/tools/calibrate_camera.py`。
仅依赖Python3、NumPy、OpenCV；独立运行，无需启动ROS2或重新编译C++。

1. 准备平整棋盘格，默认9列×6行**内角点**（10×7个格子），每格实测25mm。
2. 修改脚本顶部的`BOARD_SIZE`、`SQUARE_SIZE_M`、`IMAGE_DIR`。锁定焦距及分辨率，采集20～30张不同位置、距离、倾角的清晰棋盘格照片，覆盖画面中心和边缘；板子须完整可见。
3. 把照片放入`/home/llx/6D/calibration/images`。不要缩放、裁剪或混合不同分辨率。当前是普通针孔/plumb_bob模型，不适用于鱼眼模型。
4. 执行：

```bash
python3 /home/llx/6D/src/monocular_6d_pose/tools/calibrate_camera.py
```

输出在`/home/llx/6D/calibration/results/时间戳/`：

- `camera.yaml`：相机K、D、分辨率及ROS相机标定常用字段（JSON语法的合法YAML）。
- `report.json`：总RMS像素误差、每张图RMS、接受/拒绝图片及标定设置。
- `corners_*.png`：角点检测结果。

至少10张有效照片才能计算，但数量不能替代视角多样性。当前固定k3=0以减少高阶参数过拟合，其他五参数中的k1、k2、p1、p2参与求解。结果需要独立照片和实物测距验证，不能只凭低重投影误差判定标定准确。

脚本不生成示例真实参数，也不覆盖任何旧标定文件。当前真实相机节点仍未实现，生成的YAML尚不会自动加载进Pose节点；后续由相机节点读取并发布CameraInfo。R是单目无校正旋转的单位阵，P为[K|0]，不是相机在机器人基座中的外参。各视图外参不等于手眼标定。

自检：`python3 /home/llx/6D/src/monocular_6d_pose/tools/test_calibration.py`。
