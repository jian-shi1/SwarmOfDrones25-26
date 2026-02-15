from typing import List
import heapq
import numpy as np

class Swarm:
    """
    Рой, хранящий историю наблюдений
    """
    def __init__(self, start_coordinates: np.ndarray, N: int =5, m: float =15.0, s0: float =2.0, H: int = 20, alpha_idx: int = None, a_max: float=9.81, v_max:float = 33.3, R0: float = 100):
        self.N = N
        self.H = H
        self.m = m
        self.dt = 1/H
        self.t = 0.0
        self.s0 = s0
        self.alpha_idx = alpha_idx if alpha_idx is not None else N-1
        self.a_max = a_max
        self.v_max = v_max
        self.R0 = R0

        # Истинные параметры
        # Координаты агентов
        self.x = start_coordinates.copy() # np.arange(0, N * s0, s0)
        # Скорости
        self.v = np.zeros(N).astype('float')
        # Ускорения
        self.a = np.zeros(N).astype('float')
        # Метки времени
        self.timestamps = [self.t]
        # История состояний
        self.trajectory = [self.x.copy()]
        self.velocities = [self.v.copy()]
        self.accelerations = [self.a.copy()]

    def step(self, acc_cmd: np.ndarray):
        """
        Тик времени - обновление состояний
        """
        acc_cmd = np.clip(acc_cmd, -self.a_max, self.a_max)
        self.x += self.v * self.dt + 0.5 * acc_cmd * self.dt**2

        # v_new = np.clip(self.v + acc_cmd * self.dt, -self.v_max, self.v_max)
        self.v = self.v + acc_cmd * self.dt
        self.a = acc_cmd.copy()
        self.t += self.dt
        self.timestamps.append(self.t)

        self.trajectory.append(self.x.copy())
        self.velocities.append(self.v.copy())
        self.accelerations.append(self.a.copy())
