# 第 5 章　拉格朗日动力学

## 学习目标

从能量系统推导单摆和多关节机械臂动力学。

定义拉格朗日量 $L=T-V$。每个广义坐标满足

$$
\frac{d}{dt}\left(\frac{\partial L}{\partial\dot q_i}\right)
-\frac{\partial L}{\partial q_i}=\tau_i.
$$

粘性摩擦可写为广义阻力 $-B\dot q$。

## 单摆推导

取

$$T=\frac12ml^2\dot q^2,\qquad V=mgl\sin q,$$

代入得到

$$ml^2\ddot q+b\dot q+mgl\cos q=\tau.$$

这正是 Pendulum 类实现的模型。

## 多自由度形式

对每个 $q_i$ 应用同一方程并整理：

$$M(q)\ddot q+c(q,\dot q)+g(q)=\tau.$$

推导时必须明确零点和正方向，写出每个质心速度，并进行量纲与极端情况检查。

## 习题

1. 推导带阻尼单摆。
2. 若角度从竖直向下测量，重力项如何改变？
