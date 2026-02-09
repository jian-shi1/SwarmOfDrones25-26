class Swarm:
    def __init__(self,
                 initial_positions: List[float],
                 alpha_idx: int,
                 emission_schedule: List[int],
                 H: float,
                 R0: float,
                 c: float = 3e8,
                 s_min: float = 0.1,
                 R_max: float = 100.0,
                 alpha_update_freq: float = 100.0):
        self.N = len(initial_positions)
        self.alpha_idx = alpha_idx
        self.R0 = R0
        self.c = c
        self.H = H
        self.dt_emit = 1.0 / H
        self.s_min = s_min
        self.R_max = R_max
        self.alpha_update_freq = alpha_update_freq
        self.alpha_update_dt = 1.0 / alpha_update_freq
        self.emission_schedule = emission_schedule
        
        # Создание дронов
        self.drones: List[Drone] = []
        for i in range(self.N):
            leader_idx = None if i == alpha_idx else (i + 1 if i < alpha_idx else i - 1)
            self.drones.append(Drone(i, initial_positions[i], leader_idx, alpha_idx))
        
        # Событийная очередь
        self.event_queue = []
        self.current_time = 0.0
        self.simulation_stopped = False
        self.stop_message = ""
        
        # Планирование первых событий
        for step in range(len(emission_schedule)):
            t_emit = step * self.dt_emit
            drone_id = emission_schedule[step % len(emission_schedule)]
            heapq.heappush(self.event_queue, (t_emit, 'EMISSION', drone_id))
        
        # Первое обновление альфы
        heapq.heappush(self.event_queue, (0.0, 'ALPHA_UPDATE', alpha_idx))

    def _handle_emission(self, drone_id: int, t_emit: float, controller: 'SMDController'):
        if self.simulation_stopped:
            return
            
        emitter = self.drones[drone_id]
        # Для не-альф: интегрируем с текущим ускорением
        if drone_id != self.alpha_idx:
            emitter.integrate_to(t_emit)
        payload = SignalPayload(drone_id, t_emit, emitter.x, emitter.v, emitter.a)
        
        for j in range(self.N):
            if j == drone_id or self.simulation_stopped:
                continue
                
            receiver = self.drones[j]
            if j != self.alpha_idx:
                receiver.integrate_to(t_emit)
            distance = abs(receiver.x - emitter.x)
            
            if distance <= self.R0:
                t_arrival = t_emit + distance / self.c
                
                # Обновить получателя до момента прибытия
                if j != self.alpha_idx:
                    receiver.integrate_to(t_arrival)
                
                # Обновить ускорение, если нужно
                if receiver.leader_idx is not None and drone_id == receiver.leader_idx:
                    leader_state = {'x': emitter.x, 'v': emitter.v, 'a': emitter.a}
                    own_state = {'x': receiver.x, 'v': receiver.v, 'a': receiver.a}
                    new_a = controller.compute_acceleration(
                        receiver.idx, leader_state, own_state, t_arrival
                    )
                    receiver.set_acceleration(t_arrival, new_a)
                
                # Проверка безопасности
                is_safe, msg = receiver.check_safety_with_neighbors(self.drones, self.s_min, self.R_max)
                if not is_safe:
                    self.simulation_stopped = True
                    self.stop_message = f"Safety violation at t={t_arrival:.6f}: {msg}"
                    print(self.stop_message)
                    return
        
        # Запланировать следующее излучение
        next_step = int(t_emit / self.dt_emit) + len(self.emission_schedule)
        t_next = next_step * self.dt_emit
        heapq.heappush(self.event_queue, (t_next, 'EMISSION', drone_id))

    def _handle_alpha_update(self, drone_id: int, t: float, controller: 'SMDController'):
        if self.simulation_stopped:
            return
        alpha = self.drones[drone_id]
        alpha.integrate_to(t, a_func=controller.ddot_x_alpha)
        # Запланировать следующее обновление
        t_next = t + self.alpha_update_dt
        heapq.heappush(self.event_queue, (t_next, 'ALPHA_UPDATE', drone_id))

    def run_until(self, T_max: float, controller: 'SMDController'):
        while self.event_queue and not self.simulation_stopped:
            t_event, event_type, data = heapq.heappop(self.event_queue)
            if t_event > T_max:
                break
            self.current_time = t_event  # ← ИСПРАВЛЕНО: было current_t
            
            if event_type == 'EMISSION':
                self._handle_emission(data, t_event, controller)
            elif event_type == 'ALPHA_UPDATE':
                self._handle_alpha_update(data, t_event, controller)
        
        if self.simulation_stopped:
            print(f"Simulation stopped early at t={self.current_time:.6f}: {self.stop_message}")
        else:
            print(f"Simulation completed up to t={T_max:.6f}")
        
        return self.drones