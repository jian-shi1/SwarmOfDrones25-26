import numpy as np
from Swarm import Swarm
from copy import copy
from EKF import EKF

class SMDController:
    """
    Вся логика получения ускорений здесь.
    Рой получает только итоговые ускорения
    """
    def __init__(self, ddot_x, swarm: Swarm, a_max=3.7, H = 20, tau_lag = 0.5, signals_order = list(range(4))):
        # Максимальное ускорение
        self.a_max = a_max
        # Минимальная безопасная дистанция
        # Тик времени
        self.dt = 1/H
        self.H = H
        # Функция ускорения для альфы
        self.ddot_x = ddot_x
        # Время лага (реагирования)
        self.tau_lag = tau_lag
        self.signals_order = copy(signals_order)
        self.turn = -1
        self.swarm = swarm
        self.initial_distances = swarm.trajectory[0]
        self.friction_coeff = 0.04

        self.rel_filters = {} # (i, j) -> EKF

    def _get_optimal_distances(self, i, indexes):
        return np.abs(self.initial_distances[indexes] - self.initial_distances[i])

    def control_step(self):
        swarm = self.swarm
        N = swarm.N
        t = swarm.t
        dt = swarm.dt
        R0 = swarm.R0

        # итоговые ускорения
        acc = np.copy(self.swarm.a) # не зерос - у нас движение альфы непрерывно, не должны отставать

        alpha = swarm.alpha_idx
        acc[alpha] = self.ddot_x(t + dt)

        self.turn = (self.turn + 1) % len(self.signals_order)
        turn = self.signals_order[self.turn]

        x_i = swarm.x[turn]
        v_i = swarm.v[turn]

        xdiff = np.abs(swarm.x - x_i)
        mask = (xdiff <= R0) & (np.arange(N) != turn)
        # mask[turn] = False
        idxs = np.where(mask)[0]

        acc_turn = 0.0
        i = turn
        current_step = []

        direction_to_alpha = np.sign(swarm.x[alpha] - x_i) # -1 if turn > alpha else 1
        for j in idxs:
            key = (turn, j)

            # получаем измерение расстояния с прибора
            d_measure = swarm.x[j] - x_i

            # инит EKF если еще нет для этой пары
            if key not in self.rel_filters:
                kf = EKF(dt, s_0=d_measure)
                self.rel_filters[key] = kf
            else:
                kf = self.rel_filters[key]
            dt_ekf = dt if kf.last_t is None else t - kf.last_t
            # 1. предсказание
            kf.predict(dt_ekf)
            # 2. обновление
            kf.update(d_measure)
            kf.last_t = t
            # получение данных
            d_ij = kf.d # x_j - x_i
            dv = kf.d_dot # v_j - v_i

            ell = swarm.start_coordinates[j] - swarm.start_coordinates[i]
            k = swarm.m * swarm.a_max/np.abs(ell)
            b = max(swarm.m / self.tau_lag, 2*np.sqrt(np.abs(k) * swarm.m))
            force = (k * (d_ij - ell) + b * dv)
            acc_turn += force

        acc[turn] = acc_turn / swarm.m
        return acc
