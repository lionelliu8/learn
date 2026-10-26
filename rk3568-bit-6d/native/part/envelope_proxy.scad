// 单位 mm。用户确认总长含刀头；端面对角宽5、对边高4。
// 仅外包络，不是完整六角杆身/十字刀头模型。
// 禁止将此外包络的八个角当作真实零件上可见的 PnP 特征点。
length_mm = 25;
width_mm = 5;
height_mm = 4;
// X: 沿零件长轴；Y/Z: 横截面；原点为外包络中心。
cube([length_mm, width_mm, height_mm], center=true);
