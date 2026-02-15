import numpy as np
from Swarm import Swarm
from copy import copy

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
        self.friction_coeff = 0.4

        self.rel_filters = {} # (i, j) -> EKF
        self.eps_vel = 0.05 # порог "живости" соседа
        self.dead_time = 2.0   # если давно не видели — игнор
        self.b_scale = 1.0     # масштаб демпфера

    def _get_optimal_distances(self, i, indexes):
        return np.abs(self.initial_distances[indexes] - self.initial_distances[i])

    def control_step(self):
        swarm = self.swarm
        N = swarm.N
        t = swarm.t
        dt = swarm.dt
        R0 = swarm.R0

        # итоговые ускорения
        acc = np.copy(self.swarm.a) # np.zeros(N)

        # ========== 1. Альфа-агент ==========
        alpha = swarm.alpha_idx
        acc[alpha] = self.ddot_x(t + dt)

        # ========== 2. Кто опрашивается сейчас ==========
        self.turn = (self.turn + 1) % len(self.signals_order)
        turn = self.signals_order[self.turn]

        x_i = swarm.x[turn]

        # ========== 3. Соседи в радиусе ==========
        xdiff = np.abs(swarm.x - x_i)
        mask = (xdiff <= R0) & (np.arange(N) != turn)
        idxs = np.where(mask)[0]

        acc_turn = 0.0

        # ========== 4. Обработка каждого соседа ==========
        for j in idxs:
            key = (turn, j)

            # --- EKF инициализация ---
            if key not in self.rel_filters:
                kf = EKF(dt)
                kf.x[0] = swarm.x[j] - x_i   # начальная дистанция
                self.rel_filters[key] = kf
            else:
                kf = self.rel_filters[key]

            # --- predict ---
            if kf.last_t is None:
                dt_ekf = dt
            else:
                dt_ekf = t - kf.last_t
            kf.predict(dt_ekf)

            # --- update ---
            d_meas = swarm.x[j] - x_i
            kf.update(d_meas)
            kf.last_t = t

            # --- timeout "призраков" ---
            if dt_ekf > self.dead_time:
                continue

            # ========== 5. Контроллер ==========
            # желаемая дистанция (из начальной конфигурации)
          
            # оценка относительной скорости
            dv = kf.d_dot

            # ell = 0
            # s0_base = 1.3 * self.swarm.s0
            # if dv < -1e-3:  # сближение
            #     ttc = abs(kf.d) / abs(dv)
            #     # Увеличить зазор при малом TTC
            #     beta = 1.0
            #     ell = s0_base * (1.0 + beta * max(0.0, 1.0 - ttc / self.tau_lag))
            # else:
            #     ell = s0_base
            ell = abs(self.initial_distances[j] - self.initial_distances[turn])
            # ell = 0.6 * ell + self.tau_lag * 
            # ошибка по расстоянию
            tau_pred = dt
            d_pred = kf.d + kf.d_dot * tau_pred + 0.5 * kf.d_ddot * tau_pred**2
            dx = d_pred - np.sign(kf.d) * ell # 

            # жёсткость
            k = swarm.m * self.a_max / ell

            # гейтинг демпфера
            if abs(dv) < self.eps_vel: #  or abs(kf.d) <= ell * 0.4
                b = 0.0
            else:
                b = self.b_scale * swarm.m / self.tau_lag

            acc_turn += k * dx + b * dv

        v_i = swarm.v[turn]
        F_fric = -self.friction_coeff * v_i * abs(v_i)
        acc_turn += F_fric
        acc[turn] = acc_turn / swarm.m

        return acc