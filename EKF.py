import numpy as np


class EKF:
    """
    EKF для относительного движения:
    state = [d, d_dot]
    d     = x_j - x_i
    d_dot = v_j - v_i
    """

    def __init__(self, dt, q_d=1e-4, q_v=1e-2, q_a=1e-1, r_d=1e-3):
        self.dt = dt

        # состояние
        self.x = np.zeros(3)  # [d, d_dot]

        # ковариация
        self.P = np.eye(3) * 1.0

        # шумы
        self.Q = np.diag([q_d, q_v, q_a])
        self.R = np.array([[r_d]])

        self.last_t = None

    def predict(self, dt):
        F = np.array([
            [1.0, dt, 0.5* dt**2],
            [0.0, 1.0, dt],
            [0.0, 0.0, 1.0]
        ])

        self.x = F @ self.x
        self.P = F @ self.P @ F.T + self.Q

    def update(self, d_meas):
        H = np.array([[1.0, 0.0, 0.0]])

        y = np.array([[d_meas]]) - H @ self.x
        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)

        self.x = self.x + (K @ y).flatten()
        self.P = (np.eye(3) - K @ H) @ self.P

    @property
    def d(self):
        return self.x[0]

    @property
    def d_dot(self):
        return self.x[1]

    @property
    def d_ddot(self):
        return self.x[2]
