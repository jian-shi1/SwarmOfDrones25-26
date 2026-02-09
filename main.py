# Параметры
initial_x = [0.0, 2.0, 4.0, 6.0, 8.0]
alpha_idx = 2
schedule = [2, 0, 1, 3, 4]  # альфа первым
H = 20.0  # Гц
R0 = 10.0  # метров

# Контроллер
controller = SMDController(
    ddot_x_alpha=rectlinear_flucts,
    m=10.0,
    a_max=10.0,
    s0=2.0,
    tau=0.5,
    C_friction=0.1
)

# Рой
swarm = Swarm(initial_x, alpha_idx, schedule, H, R0)

# Запуск
drones = swarm.run_until(T_max=150.0, controller=controller)

# Создаём времена для сэмплирования (например, 10 Гц)
t_plot = np.arange(0, 150.0, 0.1)

# Визуализация
fig, axes = plt.subplots(1, 5, figsize=(30, 10))
cyclogram(drones, t_plot, alpha_idx=swarm.alpha_idx, ax=axes[0], relative=False)
cyclogram(drones, t_plot, alpha_idx=swarm.alpha_idx, ax=axes[1], relative=False, limit=100)  # последние 100 сек при dt=0.1
cyclogram(drones, t_plot, alpha_idx=swarm.alpha_idx, ax=axes[2], relative=True)
cyclogram(drones, t_plot, alpha_idx=swarm.alpha_idx, ax=axes[3], relative=True, limit=100)
cyclogram(drones, t_plot, alpha_idx=swarm.alpha_idx, ax=axes[4], relative=False, type_data="Velocities")
plt.tight_layout()
plt.show()
