# 第 2 章　刚体位姿与坐标变换

## 学习目标

使用旋转矩阵和齐次变换统一描述刚体位姿。

## 2.1 平面旋转

平面旋转矩阵为

$$
R(\theta)=
\begin{bmatrix}
\cos\theta & -\sin\theta \\
\sin\theta & \cos\theta
\end{bmatrix}.
$$

它满足 $R^TR=I$、$\det R=1$、$R^{-1}=R^T$。若点在 B 系中的坐标为 $p^B$，则

$$p^A=r^A_B+R^A_Bp^B.$$

## 2.2 齐次变换

$$
T^A_B=
\begin{bmatrix}
R^A_B & r^A_B \\
0 & 1
\end{bmatrix},
\qquad T^A_C=T^A_BT^B_C.
$$

矩阵乘法顺序不可交换。

## 2.3 二连杆正运动学

$$x=l_1\cos q_1+l_2\cos(q_1+q_2),$$

$$y=l_1\sin q_1+l_2\sin(q_1+q_2).$$

代码对应 TwoLinkArm.forward_kinematics。

## 习题

1. 验证 $R(\theta)^{-1}=R(-\theta)$。
2. 推导肘部和末端坐标。
