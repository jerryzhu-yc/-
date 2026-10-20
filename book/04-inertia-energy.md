# 第 4 章　刚体惯性与能量

## 学习目标

理解质心、转动惯量、平行轴定理与刚体动能。

质心为

$$r_C=\frac{1}{m}\int r\,dm,$$

转动惯量为

$$I=\int r_\perp^2\,dm.$$

均匀细杆绕质心的惯量为 $I_C=ml^2/12$。由平行轴定理，绕端点为 $I_O=I_C+ml^2/4=ml^2/3$。

平面刚体动能为

$$T=\frac12m v_C^Tv_C+\frac12I_C\omega^2.$$

机械臂总动能可整理为

$$T=\frac12\dot q^TM(q)\dot q.$$

重力势能为 $V=\sum_i m_i g h_i(q)$，重力项满足

$$g(q)=\frac{\partial V}{\partial q}.$$

代码中的 energy 与 gravity_vector 应满足这一关系。

## 习题

1. 推导均匀细杆惯量。
2. 写出 2R 两根杆的质心坐标。
