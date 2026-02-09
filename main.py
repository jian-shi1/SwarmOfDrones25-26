from SMDController import SMDController
from input_data.ddotx_alpha_functions import rectlinear_flucts, rough_break
from Swarm import Swarm
from cyclogram import cyclogram
from matplotlib import pyplot as plt
import numpy as np

initial_x = [0.0, 2.0, 4.0, 6.0, 8.0]
alpha_idx = 2
schedule = [2, 1, 3, 0, 4]  # альфа первым
H = 20.0  # Гц
R0 = 10.0  # метров
T_max=200.0

# Контроллер
controller = SMDController(
    ddot_x_alpha=rectlinear_flucts,  # попробуй также rough_break
    m=10.0,
    a_max=10.0,
    s0=2.0,
    tau=0.5,
    C_friction=0.1
)

# Рой
swarm = Swarm(
    initial_positions=initial_x,
    alpha_idx=alpha_idx,
    emission_schedule=schedule,
    H=H,
    R0=R0,
    s_min=0.1,
    R_max=200.0
)

# Запуск
drones = swarm.run_until(T_max, controller=controller)

# Визуализация
T_actual = swarm.current_time if swarm.simulation_stopped else T_max
t_plot = np.linspace(0, T_actual, int(T_actual * 100))

fig, axes = plt.subplots(1, 5, figsize=(30, 6))
cyclogram(drones, t_plot, alpha_idx=swarm.alpha_idx, ax=axes[0], relative=False)
cyclogram(drones, t_plot, alpha_idx=swarm.alpha_idx, ax=axes[1], relative=False, limit=100)
cyclogram(drones, t_plot, alpha_idx=swarm.alpha_idx, ax=axes[2], relative=True)
cyclogram(drones, t_plot, alpha_idx=swarm.alpha_idx, ax=axes[3], relative=True, limit=100)
cyclogram(drones, t_plot, alpha_idx=swarm.alpha_idx, ax=axes[4], relative=False, type_data="Velocities")
plt.tight_layout()
plt.show()