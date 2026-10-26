# Ubuntu 中的批头建模与位姿学习

主目录：`/home/llx/6D/native/part`。
在 Ubuntu 做建模、图像回放、几何投影和算法验证；RK3568 负责真实 MIPI 采集、RGA 与部署性能测试。
本轮已在 Ubuntu 22.04、OpenCV 4.5.4 中实际运行，日志为 `setup-last.log`。

## 1. 查看三维模型

在 Ubuntu 桌面的终端运行：

```sh
cd /home/llx/6D/native/part
python3 view_model.py
```

鼠标左键拖动旋转，Q 或 Esc 退出。静态图：`models/bit-v02/preview.png`。
可导入其他 CAD/网格软件的文件：

- `models/bit-v02/bit_outer_proxy.stl`
- `models/bit-v02/bit_outer_proxy.obj`
- `models/bit-v02/bit_outer_proxy.scad`

单位 mm；总长25、六角杆身17、刀头加过渡段约8、对角宽5、对边高4。
几何参数在 `dimensions.json`；模型生成器是 `build_bit_model.py`。
原点位于总长中点，+X 指向刀头，Y 为端面宽方向，Z 为端面高方向。
杆身和刀头分界位于 X=4.5 mm，背面 X=-12.5 mm，尖端包络面 X=12.5 mm。

灰色为六角杆身，黄色为简化刀头包络。由于尖端尺寸、槽纹和过渡面尚未实测，
当前尖端截面缩为杆身的25%，这是明确记录的示意假设，不是测量值。
十字槽未建模；当前模型不能冒充完整的精确机械图纸或已验证的6D配准模型。
对边4与对角5暂用仿射六角形共同表达；严格正六边形对边4时对角约4.619，需要以后用卡尺核实。

## 2. 运行模型投影和 PnP 仿真

```sh
cd /home/llx/6D/native/part
python3 simulate_projection.py --calibration calibration/camera-final.yaml \
  --output "results/simulation_$(date +%Y%m%d_%H%M%S)"
```

脚本将三维模型的虚拟顶点投影到相机图像，然后使用正确点对应恢复位姿，
再加入0.25、0.5、1像素高斯噪声，每档40次，固定随机种子，生成 `simulation.json` 和 `projection.png`。
几何单位从CAD毫米显式转成相机米。无噪声自检通过。

此仿真假设模型和标定准确，并提供所有虚拟顶点的正确对应，连真实相机可能看不到的顶点也参与。
没有模拟金属反光、遮挡、误匹配、对称性识别或图像模糊；当前实物检测器并不会自动产出这些点。
它用于学习投影及求解器对噪声的敏感性，不能作为实物识别精度或可观测性的证明。
已运行的结果中，0.5像素噪声出现大误差解，且`solvePnP`依然返回成功；
不能只凭函数返回值接受位姿，也不能因40次有限样本的噪声统计非单调而删除不利结果。

## 3. 回放真实相机图

```sh
cd /home/llx/6D/native/part
python3 inspect_part.py samples/side01.pgm --roi 2030 580 295 240 \
  --output "results/real_side_$(date +%Y%m%d_%H%M%S)"
```

保存框选图、二值图和 JSON。Ubuntu 已复现板端的单个轮廓候选，
约274×42像素；程序仍将自动头尾判为不确定，不输出伪造的深度或完整6D位姿。
`samples/side01-burst-summary.json` 保留先前5帧板端定位统计。
RGA和V4L2采图脚本要在板端运行，不能把虚拟机中的回放当作MIPI硬件测试。

后续优先处理真实刀头特征与头尾验证，再建立可靠的图像/三维点对应和姿态质量检查。
无需在这一步安装Gazebo或物理仿真引擎；当前学习对象是相机几何与视觉位姿。
