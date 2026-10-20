import numpy as np

from robot_dynamics import Pendulum, TwoLinkArm, rk4_step


def test_rk4_solves_exponential():
    x = rk4_step(lambda _t, y: y, 0.0, np.array([1.0]), 0.1)
    assert np.allclose(x, np.exp(0.1), rtol=1e-6)


def test_pendulum_forward_inverse_round_trip():
    model = Pendulum(mass=1.2, length=0.7, damping=0.1)
    q, dq, ddq = 0.4, -0.2, 1.3
    tau = model.inverse_dynamics(q, dq, ddq)
    assert np.isclose(model.forward_dynamics(q, dq, tau), ddq)


def test_mass_matrix_is_symmetric_positive_definite():
    arm = TwoLinkArm()
    for q2 in np.linspace(-np.pi, np.pi, 13):
        matrix = arm.mass_matrix([0.3, q2])
        assert np.allclose(matrix, matrix.T)
        assert np.all(np.linalg.eigvalsh(matrix) > 0)


def test_two_link_forward_inverse_round_trip():
    arm = TwoLinkArm()
    q = np.array([0.4, -0.7])
    dq = np.array([0.2, -0.3])
    ddq = np.array([1.1, -0.9])
    tau = arm.inverse_dynamics(q, dq, ddq)
    assert np.allclose(arm.forward_dynamics(q, dq, tau), ddq)


def test_gravity_is_potential_gradient():
    arm = TwoLinkArm(damping=(0.0, 0.0))
    q = np.array([0.3, -0.6])
    eps = 1e-6
    gradient = np.empty(2)
    for i in range(2):
        delta = np.zeros(2); delta[i] = eps
        gradient[i] = (arm.energy(q + delta, [0, 0]) - arm.energy(q - delta, [0, 0])) / (2 * eps)
    assert np.allclose(gradient, arm.gravity_vector(q), rtol=1e-6, atol=1e-6)


def test_forward_kinematics_straight_arm():
    arm = TwoLinkArm(l1=1.0, l2=0.5)
    elbow, tip = arm.forward_kinematics([0.0, 0.0])
    assert np.allclose(elbow, [1.0, 0.0])
    assert np.allclose(tip, [1.5, 0.0])
