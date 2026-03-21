from SMDController import SMDController
from Swarm import Swarm
# from matplotlib import pyplot as plt
import numpy as np
from tqdm import tqdm
import dill
from cyclogram import CycType

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

if __name__ == "__main__":
    g = 9.81
    v_max = 100/3
    with open("sims_configs.pkl", "rb") as f:
        flights_params = dill.load(f)

    flights_results = []

    for i, (N, alpha_idx, s0, H, T, a_max, m, ddotx_func, start_coordinates, R0, signals_order) in enumerate(flights_params):
        flights_results.append(run_simulation(N, alpha_idx, s0, H, T, a_max, m, ddotx_func, start_coordinates, R0, signals_order))
