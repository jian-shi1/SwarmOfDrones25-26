from typing import List, Optional, Tuple

class Drone:
    def __init__(self, idx: int, x0: float, leader_idx: Optional[int], alpha_idx: int):
        self.idx = idx
        self.alpha_idx = alpha_idx
        self.leader_idx = leader_idx  # None for alpha
        
        # Текущее состояние
        self.x = x0
        self.v = 0.0
        self.a = 0.0
        self.t_last = 0.0
        
        # История для визуализации и EKF
        self.t_history = [0.0]
        self.x_history = [x0]
        self.v_history = [0.0]
        self.a_history = [0.0]
        
        # Для не-альф: EKF соседей
        self.ekfs = {}
        self.ekf_P0 = [1.0, 0.1, 0.1]
        self.ekf_R = [0.01, 0.01, 0.1]
        self.ekf_q_a = 0.5

    def integrate_to(self, t: float, a_func=None):
        """
        Интегрировать движение до момента t.
        Если a_func задан — использовать его (для альфы).
        Иначе — использовать текущее ускорение (для остальных).
        """
        if t <= self.t_last:
            return
        
        dt_sub = 0.001  # 1 мс — достаточно для точности
        t_curr = self.t_last
        x_curr, v_curr = self.x, self.v
        
        while t_curr < t:
            dt = min(dt_sub, t - t_curr)
            if a_func is not None:
                a_curr = a_func(t_curr)
            else:
                a_curr = self.a
            v_curr += a_curr * dt
            x_curr += v_curr * dt
            t_curr += dt
        
        # Обновить состояние
        self.x, self.v = x_curr, v_curr
        if a_func is not None:
            self.a = a_func(t)
        self.t_last = t
        
        # Сохранить в историю
        self.t_history.append(t)
        self.x_history.append(self.x)
        self.v_history.append(self.v)
        self.a_history.append(self.a)

    def set_acceleration(self, t: float, new_a: float):
        """Для не-альф: установить новое ускорение"""
        self.integrate_to(t)
        self.a = new_a

    def get_state_at(self, t: float) -> Tuple[float, float, float]:
        """Интерполировать состояние в момент t по сохранённой истории"""
        if not self.t_history:
            return self.x, self.v, self.a
        
        # Экстраполяция вперёд (после последней точки)
        if t >= self.t_history[-1]:
            dt = t - self.t_history[-1]
            x0, v0, a0 = self.x_history[-1], self.v_history[-1], self.a_history[-1]
            v = v0 + a0 * dt
            x = x0 + v0 * dt + 0.5 * a0 * dt**2
            return x, v, a0
        
        # Экстраполяция назад (до первой точки)
        if t <= self.t_history[0]:
            dt = t - self.t_history[0]
            x0, v0, a0 = self.x_history[0], self.v_history[0], self.a_history[0]
            v = v0 + a0 * dt
            x = x0 + v0 * dt + 0.5 * a0 * dt**2
            return x, v, a0
        
        # Интерполяция внутри диапазона
        idx = 0
        while idx < len(self.t_history) - 1 and self.t_history[idx + 1] < t:
            idx += 1
        
        t0, t1 = self.t_history[idx], self.t_history[idx + 1]
        x0, x1 = self.x_history[idx], self.x_history[idx + 1]
        v0, v1 = self.v_history[idx], self.v_history[idx + 1]
        a0, a1 = self.a_history[idx], self.a_history[idx + 1]
        
        alpha = (t - t0) / (t1 - t0)
        x = x0 + alpha * (x1 - x0)
        v = v0 + alpha * (v1 - v0)
        a = a0 + alpha * (a1 - a0)
        return x, v, a

    def check_safety_with_neighbors(self, all_drones: List['Drone'], s_min: float, R_max: float) -> Tuple[bool, str]:
        neighbor_indices = []
        N = len(all_drones)
        if self.idx > 0:
            neighbor_indices.append(self.idx - 1)
        if self.idx < N - 1:
            neighbor_indices.append(self.idx + 1)
        
        for j in neighbor_indices:
            dist = abs(all_drones[j].x - self.x)
            if dist < s_min:
                return False, f"Collision: drone {self.idx} and {j}, distance={dist:.3f} < {s_min}"
            if dist > R_max:
                return False, f"Dispersal: drone {self.idx} and {j}, distance={dist:.3f} > {R_max}"
        return True, ""
