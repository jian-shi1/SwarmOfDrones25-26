import numpy as np
import heapq
from dataclasses import dataclass
from typing import List, Tuple

class EKF:
    def __init__(self, x: float, v: float, a: float, P_diag: List[float], q_a: float = 1.0):
        """
        Инициализация EKF для дрона.
        P_diag = [σ_x², σ_v², σ_a²] — начальная ковариация
        q_a — интенсивность шума в ускорении (для процессной модели)
        """
        self.x = np.array([x, v, a], dtype=float)
        self.P = np.diag(P_diag)
        self.q_a = q_a
        self.last_update_t = 0.0

    def _state_transition(self, dt: float) -> Tuple[np.ndarray, np.ndarray]:
        """Возвращает F и Q для шага dt"""
        F = np.array([
            [1, dt, 0.5 * dt**2],
            [0, 1,  dt],
            [0, 0,  1]
        ], dtype=float)

        dt2 = dt**2
        dt3 = dt**3
        dt4 = dt**4
        dt5 = dt**5
        Q = self.q_a * np.array([
            [dt5/20, dt4/8,  dt3/6],
            [dt4/8,  dt3/3,  dt2/2],
            [dt3/6,  dt2/2,  dt]
        ], dtype=float)
        return F, Q

    def predict_to(self, t: float):
        """Прогнозировать состояние до момента t"""
        if t <= self.last_update_t:
            return
        dt = t - self.last_update_t
        F, Q = self._state_transition(dt)
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q
        self.last_update_t = t

    def update(self, z: np.ndarray, t_meas: float, R_diag: List[float]):
        """
        Обновить фильтр измерением z = [x, v, a] в момент t_meas
        R_diag = [σ_x², σ_v², σ_a²] — ковариация измерения
        """
        self.predict_to(t_meas)
        H = np.eye(3)  # измеряем всё состояние
        R = np.diag(R_diag)

        y = z - self.x  # невязка
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)

        self.x = self.x + K @ y
        self.P = (np.eye(3) - K @ H) @ self.P
        self.last_update_t = t_meas