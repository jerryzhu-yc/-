# 第 2 章　刚体位姿与坐标变换

## 学习目标

使用旋转矩阵和齐次变换统一描述刚体位姿。

## 2.1 平面旋转

平面旋转矩阵为

$$R(	heta)=egin{bmatrix}cos	heta&-sin	heta\sin	heta&cos	hetaend{bmatrix}.$$

它满足 $R^TR=I$、$det R=1$，且 $R^{-1}=R^T$。若点在坐标系 B 中为 $p^B$，B 相对 A 的位置为 $r^A$，则 $p^A=r^A+R^A_Bp^B$。

## 2.2 齐次变换

将旋转和平移写成

$$T^A_B=egin{bmatrix}R^A_B&r^A_B\0&1end{bmatrix}.$$

连续变换通过矩阵相乘组合：$T^A_C=T^A_BT^B_C$。乘法顺序不可交换。

## 2.3 二连杆正运动学

对于杆长 $l_1,l_2$ 的 2R 机械臂：

$$x=l_1cos q_1+l_2cos(q_1+q_2),$$
$$y=l_1sin q_1+l_2sin(q_1+q_2).$$

代码对应 TwoLinkArm.forward_kinematics。

## 常见错误

混淆“坐标系的旋转”和“向量的主动旋转”；忽略角度方向；矩阵连乘次序颠倒。解决办法是始终写清上标、下标和坐标系。

## 习题

1. 验证 $R(	heta)^{-1}=R(-	heta)$。
2. 推导 2R 机械臂肘部与末端坐标。
3. 说明 $q_2$ 是相对角还是绝对角会怎样改变公式。
