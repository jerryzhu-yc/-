# 第 1 章　数学与经典力学基础

## 学习目标

掌握向量、矩阵、导数、功、能量和力矩的必备知识。

矩阵映射写作 $y=Ax$。若任意非零 $x$ 都满足 $x^TAx>0$，则 A 正定。机器人的惯性矩阵正定，因此可逆。

时间导数描述变化率。链式法则为

$$\dot f(q)=\frac{\partial f}{\partial q}\dot q.$$

质心平动满足

$$\sum F=ma_C,$$

刚体转动满足

$$\sum \tau_C=I_C\alpha+\omega\times(I_C\omega).$$

瞬时功率为 $P=F^Tv+\tau^T\omega$。无输入、无阻尼的保守系统保持机械能 $E=T+V$。

## 例题：点质量单摆

速度为 $l\dot q$，所以

$$T=\frac12ml^2\dot q^2,\qquad V=mgl\sin q.$$

## 习题

1. 验证 $ml^2\ddot q$ 的单位是 N·m。
2. 推导均匀细杆绕端点的转动惯量。
