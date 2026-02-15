from SMDController import SMDController
from input_data.ddotx_alpha_functions import RectlinearFlucts, RoughBreak
from Swarm import Swarm
from cyclogram import cyclogram
from matplotlib import pyplot as plt
import numpy as np
import tqdm

CycType = Literal["Trajectories", "Velocities", "Accelerations"]
g = 9.81
v_max = 100/3

def run_simulation(N, alpha_idx, s0, H, T, a_max, m, ddotx_func, start_coordinates, R0, signals_order, tau_lag=1.0):
    """
    Функция для полноценного запуска одной симуляции
    """
    maxd = N * R0
    swarm = Swarm(start_coordinates, N, m, s0, H, alpha_idx, a_max, v_max, R0)
    ctrl = SMDController(ddotx_func, swarm, a_max, H, tau_lag, signals_order,)
    steps = int(T * H)
    for _ in tqdm(range(steps)):
        acc = ctrl.control_step()
        swarm.step(acc)
        # После шага алгоритма проверяем условия поддержания роя
        broke = False
        for i in range(swarm.N-1):
            # Опасное сближение
            if swarm.x[i] >= swarm.x[i+1] or swarm.x[i+1] - swarm.x[i] < s0:
              print(f'\n{i} x {i+1}: BREAK')
              broke = True
              break
        if broke:
            break
        # Рой разлетелся
        if np.abs(np.max(swarm.x) - np.min(swarm.x)) > maxd:
          print('\nflew away')
          break
    return swarm

flights_params = [
    # N, alpha_idx, s0, H, T, a_max, m, ddotx_func, start_coordinates, R0, signals_order
    (10, 9, 0.5, 60, 1200, g, 5.0, RectlinearFlucts(a_max=g), np.arange(0.0, 29, 3), 100, np.arange(8, -1, -1)),
     (7, 6, 0.5, 40, 1200, g, 5.0, RectlinearFlucts(a_max=g), np.arange(0.0, 19, 3), 100, np.arange(5, -1, -1)),
    (6, 5, 0.5, 40, 1200, g*0.2, 5.0, RectlinearFlucts(a_max=g*0.4), np.arange(0.0, 16, 3), 100, np.arange(4, -1, -1)),
     (6, 5, 0.5, 40, 1200, g*0.5, 5.0, RectlinearFlucts(a_max=g*0.4), np.arange(0.0, 16, 3), 100, np.arange(4, -1, -1)),
    (6, 5, 0.5, 40, 1200, g*0.7, 5.0, RectlinearFlucts(a_max=g*0.4), np.arange(0.0, 16, 3), 100, np.arange(4, -1, -1)),
    (6, 5, 0.5, 40, 1200, g, 5.0, RectlinearFlucts(a_max=g), np.arange(0.0, 16, 3), 100, np.arange(4, -1, -1)),
    (6, 5, 0.5, 40, 1200, g, 5.0, RectlinearFlucts(a_max=g), np.array([0.0, 5.0, 10.0, 15.0, 20.0, 25.0]), 100, np.arange(4, -1, -1)),
    (6, 5, 0.5, 40, 1200, g, 5.0, RectlinearFlucts(a_max=g), np.array([0.0, 8.0, 16.0, 24.0, 32.0, 40.0]), 100, np.arange(4, -1, -1)),
]

flights_results = []

for i, (N, alpha_idx, s0, H, T, a_max, m, ddotx_func, start_coordinates, R0, signals_order) in enumerate(flights_params):
    flights_results.append(run_simulation(N, alpha_idx, s0, H, T, a_max, m, ddotx_func, start_coordinates, R0, signals_order))