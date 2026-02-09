class SMDController:
    def __init__(self, ddot_x_alpha, m: float, a_max: float, s0: float, tau: float, C_friction: float):
        self.ddot_x_alpha = ddot_x_alpha
        self.m = m
        self.a_max = a_max
        self.s0 = s0
        self.tau = tau
        self.C = C_friction

    def compute_acceleration(self, drone_id: int, leader_state: dict, own_state: dict, t: float) -> float:
        delta_dir = 1.0 if leader_state['x'] > own_state['x'] else -1.0
        dx = abs(leader_state['x'] - own_state['x'])
        dv = leader_state['v'] - own_state['v']
        
        ell = 2 * self.s0 + 4 * (dv**2) / self.a_max
        ell = max(ell, self.s0)
        ell = min(ell, 10 * self.s0)
        
        k = self.m * self.a_max / ell
        b = max(self.m / self.tau, np.sqrt(k * self.m))
        
        delta_x = (dx - ell) * delta_dir
        delta_v = dv * delta_dir
        friction = -self.C * own_state['v'] * abs(own_state['v'])
        
        a_cmd = (k * delta_x + b * delta_v + friction) / self.m
        return np.clip(a_cmd, -self.a_max, self.a_max)
