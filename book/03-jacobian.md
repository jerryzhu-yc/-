# 第 3 章　速度、雅可比与奇异性

## 学习目标

从正运动学求雅可比，理解速度映射、力映射和奇异位形。

末端位置 $p=f(q)$ 对时间求导得到

$$\dot p=J(q)\dot q,\qquad J=\frac{\partial f}{\partial q}.$$

2R 机械臂的平移雅可比为

$$
J=
\begin{bmatrix}
-l_1\sin q_1-l_2\sin(q_1+q_2) & -l_2\sin(q_1+q_2) \\
l_1\cos q_1+l_2\cos(q_1+q_2) & l_2\cos(q_1+q_2)
\end{bmatrix}.
$$

虚功给出力映射

$$\tau=J^TF.$$

当 J 降秩时出现奇异位形。2R 机械臂满足

$$\det J=l_1l_2\sin q_2,$$

所以伸直或折叠时奇异。

## 习题

1. 用有限差分验证雅可比。
2. 解释接近奇异位形时速度噪声为何被放大。
