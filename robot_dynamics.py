"""机器人动力学从零开始：可运行参考实现。

运行：
    python robot_dynamics.py pendulum
    python robot_dynamics.py gravity
    python robot_dynamics.py control
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

import matplotlib.pyplot as plt
import numpy as np
from numpy.typing import ArrayLike, NDArray

Vector = NDArray[np.float64]


def rk4_step(f: Callable[[float, Vector], Vector], t: float, x: Vector, dt: float) -> Vector:
    """四阶 Runge–Kutta 单步积分。"""
    k1 = f(t, x)
    k2 = f(t + dt / 2, x + dt * k1 / 2)
    k3 = f(t + dt / 2, x + dt * k2 / 2)
    k4 = f(t + dt, x + dt * k3)
    return x + dt * (k1 + 2 * k2 + 2 * k3 + k4) / 6


def simulate(f: Callable[[float, Vector], Vector], x0: ArrayLike,
             duration: float, dt: float) -> tuple[Vector, NDArray[np.float64]]:
    """积分 x_dot=f(t,x)，返回时间与状态历史。"""
    if duration <= 0 or dt <= 0:
        raise ValueError("duration 和 dt 必须为正数")
    times = np.arange(0.0, duration + dt / 2, dt)
    states = np.empty((len(times), len(np.asarray(x0))), dtype=float)
    states[0] = np.asarray(x0, dtype=float)
    for i in range(len(times) - 1):
        states[i + 1] = rk4_step(f, times[i], states[i], dt)
    return times, states


@dataclass(frozen=True)
class Pendulum:
    """质量集中在杆端的单摆；q 从水平 x 轴逆时针计算。"""
    mass: float = 1.0
    length: float = 1.0
    gravity: float = 9.81
    damping: float = 0.0

    @property
    def inertia(self) -> float:
        return self.mass * self.length**2

    def gravity_torque(self, q: float) -> float:
        return self.mass * self.gravity * self.length * np.cos(q)

    def forward_dynamics(self, q: float, dq: float, tau: float) -> float:
        return (tau - self.damping * dq - self.gravity_torque(q)) / self.inertia

    def inverse_dynamics(self, q: float, dq: float, ddq: float) -> float:
        return self.inertia * ddq + self.damping * dq + self.gravity_torque(q)

    def energy(self, q: float, dq: float) -> float:
        return 0.5 * self.inertia * dq**2 + self.mass * self.gravity * self.length * np.sin(q)


@dataclass(frozen=True)
class TwoLinkArm:
    """两根均匀细杆组成的平面 2R 机械臂。"""
    m1: float = 1.0
    m2: float = 1.0
    l1: float = 1.0
    l2: float = 1.0
    gravity: float = 9.81
    damping: tuple[float, float] = (0.05, 0.05)

    @property
    def lc1(self) -> float:
        return self.l1 / 2

    @property
    def lc2(self) -> float:
        return self.l2 / 2

    @property
    def i1(self) -> float:
        return self.m1 * self.l1**2 / 12

    @property
    def i2(self) -> float:
        return self.m2 * self.l2**2 / 12

    def mass_matrix(self, q: ArrayLike) -> NDArray[np.float64]:
        q2 = np.asarray(q, dtype=float)[1]
        m11 = (self.i1 + self.i2 + self.m1 * self.lc1**2
               + self.m2 * (self.l1**2 + self.lc2**2
                            + 2 * self.l1 * self.lc2 * np.cos(q2)))
        m12 = self.i2 + self.m2 * (self.lc2**2 + self.l1 * self.lc2 * np.cos(q2))
        m22 = self.i2 + self.m2 * self.lc2**2
        return np.array([[m11, m12], [m12, m22]], dtype=float)

    def coriolis_vector(self, q: ArrayLike, dq: ArrayLike) -> Vector:
        q2 = np.asarray(q, dtype=float)[1]
        dq1, dq2 = np.asarray(dq, dtype=float)
        h = self.m2 * self.l1 * self.lc2 * np.sin(q2)
        return np.array([-h * (2 * dq1 * dq2 + dq2**2), h * dq1**2])

    def gravity_vector(self, q: ArrayLike) -> Vector:
        q1, q2 = np.asarray(q, dtype=float)
        g1 = ((self.m1 * self.lc1 + self.m2 * self.l1) * self.gravity * np.cos(q1)
              + self.m2 * self.lc2 * self.gravity * np.cos(q1 + q2))
        g2 = self.m2 * self.lc2 * self.gravity * np.cos(q1 + q2)
        return np.array([g1, g2])

    def damping_vector(self, dq: ArrayLike) -> Vector:
        return np.asarray(self.damping) * np.asarray(dq, dtype=float)

    def inverse_dynamics(self, q: ArrayLike, dq: ArrayLike, ddq: ArrayLike) -> Vector:
        q, dq, ddq = (np.asarray(v, dtype=float) for v in (q, dq, ddq))
        return (self.mass_matrix(q) @ ddq + self.coriolis_vector(q, dq)
                + self.gravity_vector(q) + self.damping_vector(dq))

    def forward_dynamics(self, q: ArrayLike, dq: ArrayLike, tau: ArrayLike) -> Vector:
        q, dq, tau = (np.asarray(v, dtype=float) for v in (q, dq, tau))
        rhs = tau - self.coriolis_vector(q, dq) - self.gravity_vector(q) - self.damping_vector(dq)
        return np.linalg.solve(self.mass_matrix(q), rhs)

    def forward_kinematics(self, q: ArrayLike) -> tuple[Vector, Vector]:
        q1, q2 = np.asarray(q, dtype=float)
        elbow = np.array([self.l1 * np.cos(q1), self.l1 * np.sin(q1)])
        tip = elbow + np.array([self.l2 * np.cos(q1 + q2), self.l2 * np.sin(q1 + q2)])
        return elbow, tip

    def energy(self, q: ArrayLike, dq: ArrayLike) -> float:
        q = np.asarray(q, dtype=float)
        dq = np.asarray(dq, dtype=float)
        kinetic = 0.5 * dq @ self.mass_matrix(q) @ dq
        q1, q2 = q
        potential = ((self.m1 * self.lc1 + self.m2 * self.l1) * self.gravity * np.sin(q1)
                     + self.m2 * self.lc2 * self.gravity * np.sin(q1 + q2))
        return float(kinetic + potential)


def pd_gravity(arm: TwoLinkArm, q: ArrayLike, dq: ArrayLike,
               q_desired: ArrayLike, kp=25.0, kd=8.0) -> Vector:
    """PD + 重力补偿。"""
    q, dq, qd = (np.asarray(v, dtype=float) for v in (q, dq, q_desired))
    return np.asarray(kp) * (qd - q) - np.asarray(kd) * dq + arm.gravity_vector(q)


def computed_torque(arm: TwoLinkArm, q: ArrayLike, dq: ArrayLike,
                    qd: ArrayLike, dqd: ArrayLike, ddqd: ArrayLike,
                    kp=36.0, kd=12.0) -> Vector:
    """计算力矩控制。"""
    q, dq, qd, dqd, ddqd = (np.asarray(v, dtype=float) for v in (q, dq, qd, dqd, ddqd))
    virtual_acc = ddqd + np.asarray(kd) * (dqd - dq) + np.asarray(kp) * (qd - q)
    return arm.inverse_dynamics(q, dq, virtual_acc)


def save_plot(name: str) -> None:
    out = Path("figures")
    out.mkdir(exist_ok=True)
    path = out / name
    plt.tight_layout()
    plt.savefig(path, dpi=160)
    print(f"已保存 {path}")


def demo_pendulum() -> None:
    pendulum = Pendulum(damping=0.08)
    def dynamics(_t: float, x: Vector) -> Vector:
        return np.array([x[1], pendulum.forward_dynamics(x[0], x[1], 0.0)])
    t, x = simulate(dynamics, [np.deg2rad(20), 0.0], 8.0, 0.005)
    plt.plot(t, np.rad2deg(x[:, 0]))
    plt.xlabel("时间 / s"); plt.ylabel("角度 / deg"); plt.grid(True)
    save_plot("pendulum.png")


def demo_gravity() -> None:
    arm = TwoLinkArm()
    initial = np.deg2rad([35.0, -50.0])
    def dynamics(_t: float, x: Vector) -> Vector:
        q, dq = x[:2], x[2:]
        tau = arm.gravity_vector(q)
        return np.r_[dq, arm.forward_dynamics(q, dq, tau)]
    t, x = simulate(dynamics, np.r_[initial, [0.0, 0.0]], 4.0, 0.005)
    plt.plot(t, np.rad2deg(x[:, :2])); plt.xlabel("时间 / s"); plt.ylabel("角度 / deg")
    plt.legend(["q1", "q2"]); plt.grid(True); save_plot("gravity_compensation.png")


def demo_control() -> None:
    arm = TwoLinkArm()
    target = np.deg2rad([60.0, -30.0])
    def dynamics(_t: float, x: Vector) -> Vector:
        q, dq = x[:2], x[2:]
        tau = computed_torque(arm, q, dq, target, [0, 0], [0, 0])
        return np.r_[dq, arm.forward_dynamics(q, dq, tau)]
    t, x = simulate(dynamics, np.zeros(4), 3.0, 0.002)
    plt.plot(t, np.rad2deg(x[:, :2])); plt.axhline(60, ls="--"); plt.axhline(-30, ls="--")
    plt.xlabel("时间 / s"); plt.ylabel("角度 / deg"); plt.legend(["q1", "q2"]); plt.grid(True)
    save_plot("computed_torque.png")


def main() -> None:
    parser = argparse.ArgumentParser(description="机器人动力学教学示例")
    parser.add_argument("demo", choices=["pendulum", "gravity", "control"])
    args = parser.parse_args()
    {"pendulum": demo_pendulum, "gravity": demo_gravity, "control": demo_control}[args.demo]()


if __name__ == "__main__":
    main()
