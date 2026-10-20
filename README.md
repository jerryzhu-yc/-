# 机器人动力学：从建模到控制

> 一本面向初学者、强调推导与可运行代码对应关系的中文开源教材。

本书从向量、坐标系和牛顿定律出发，逐步建立串联机械臂的运动学与动力学模型，最后实现数值仿真、轨迹跟踪和计算力矩控制。正文使用统一符号、章末小结、例题和习题；代码使用 Python、NumPy 与 Matplotlib。

## 阅读目标

完成本书后，你应该能够：

1. 建立刚体坐标系并使用旋转矩阵、齐次变换描述位姿；
2. 推导速度雅可比矩阵并解释奇异位形；
3. 使用拉格朗日法和牛顿–欧拉法建立机器人动力学；
4. 理解机械臂标准方程及其结构性质；
5. 编写正动力学、逆动力学和数值积分程序；
6. 实现重力补偿、PD 与计算力矩控制。

## 目录

### 第一篇　数学与运动学基础

- [第 0 章　前言与学习路线](book/00-preface.md)
- [第 1 章　数学与经典力学基础](book/01-math-mechanics.md)
- [第 2 章　刚体位姿与坐标变换](book/02-rigid-body-kinematics.md)
- [第 3 章　速度、雅可比与奇异性](book/03-jacobian.md)

### 第二篇　机器人动力学建模

- [第 4 章　刚体惯性与能量](book/04-inertia-energy.md)
- [第 5 章　拉格朗日动力学](book/05-lagrange.md)
- [第 6 章　牛顿–欧拉递推法](book/06-newton-euler.md)
- [第 7 章　机械臂动力学标准形式](book/07-manipulator-equation.md)

### 第三篇　仿真与控制

- [第 8 章　正/逆动力学与数值仿真](book/08-simulation.md)
- [第 9 章　关节空间动力学控制](book/09-control.md)
- [第 10 章　进阶主题与学习路线](book/10-advanced-topics.md)

### 附录

- [附录 A　符号、单位与约定](book/appendix-a-notation.md)
- [附录 B　章末习题参考答案](book/appendix-b-solutions.md)

## 章节与代码对应

| 内容 | 主要代码 | 运行方式 |
|---|---|---|
| RK4 数值积分 | `rk4_step`, `simulate` | `pytest` |
| 单摆正/逆动力学 | `Pendulum` | `python robot_dynamics.py pendulum` |
| 二连杆惯性、科氏与重力项 | `TwoLinkArm` | `python robot_dynamics.py gravity` |
| 重力补偿 | `demo_gravity` | `python robot_dynamics.py gravity` |
| 计算力矩控制 | `computed_torque` | `python robot_dynamics.py control` |
| 物理与数值性质验证 | `test_robot_dynamics.py` | `pytest -q` |

## 快速开始

需要 Python 3.10 或更高版本：

```bash
git clone https://github.com/jerryzhu-yc/robot-dynamics-from-zero.git
cd robot-dynamics-from-zero
python -m venv .venv
source .venv/bin/activate       # Windows: .venv\Scripts\activate
pip install -r requirements.txt
pytest -q
python robot_dynamics.py pendulum
python robot_dynamics.py gravity
python robot_dynamics.py control
```

图像输出在 `figures/`。

## 建议学习方式

- 每周学习一章，先做公式推导，再运行代码；
- 修改质量、长度、阻尼、时间步长和控制增益，解释曲线变化；
- 每章至少完成两道习题；
- 对任何公式都追问三件事：来源是什么、量纲是否正确、去掉这一项会怎样。

## 符号约定

- q、dq、ddq：关节位置、速度与加速度；
- tau：关节驱动力矩；
- M(q)：惯性矩阵；
- c(q,dq)：科里奥利与离心项；
- g(q)：重力项；
- 所有角度使用弧度，物理量使用 SI 单位。

## 许可证

MIT
