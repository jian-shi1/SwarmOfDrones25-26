import numpy as np
import matplotlib.pyplot as plt

def cyclogram(traj, times, alpha_idx=0, ax=None, relative=True, limit=None, idx=None, type_data: CycType ='Trajectories'):
    """
    Визуализируем траекторию, скорость или ускорение (для двух последних - только альфу)
    """
    fig = None
    if ax is None:
        fig, ax = plt.subplots(figsize=(12, 8))

    ticks_fontsize=16
    labels_fontsize=16
    legend_fontsize=16

    traj = np.asarray(traj)
    times = np.asarray(times)
    n_steps = len(times)

    # Обрезаем, если смотрим конечный участок
    if limit is not None and limit > 0:
        start = max(0, n_steps - limit)
        plot_traj = traj[start:]
        plot_times = times[start:]
    else:
        plot_traj = traj
        plot_times = times

    N = plot_traj.shape[1]

    # сдвигаем наблюдения если смотрим относительно альфы
    if relative:
        alpha_col = plot_traj[:, alpha_idx][:, np.newaxis]
        positions = plot_traj - alpha_col
    else:
        positions = plot_traj

    # Цвета траекторий: черный у альфы, у остальных разноцветные
    cmap = plt.cm.tab20
    forward = [cmap(i % 20) for i in range(alpha_idx)][::-1]
    backward = [cmap(i % 20) for i in range(N - alpha_idx - 1)]
    colors = forward + ['black'] + backward

    # Отрисовка
    if type_data == "Trajectories":
        for i in range(N):
            label = "Alpha" if i == alpha_idx else f"{i+1}"
            # Жирность линии для альфы
            ms = 0.5 if i == alpha_idx else 1
            ax.plot(plot_times, positions[:, i],
                label=label, marker='o', markersize=ms,
                color=colors[i],#  if i != alpha_idx else 'black'
                linewidth=1.0)
    else:
        ax.plot(plot_times, positions[:, alpha_idx], color='black')

    # Оси
    ax.set_xlabel('$t$ [s]', loc='right', fontsize=labels_fontsize)

    y_label = "$x$ [m]" if type_data=="Trajectories" else "$v$ [m/s]" if type_data=="Velocities" else "$a$ [$m/s^2$]"
    ax.set_ylabel(y_label, loc='top', fontsize=labels_fontsize, rotation=0)
    ax.yaxis.set_label_coords(x=-0.1, y=0.94, transform=ax.transAxes)

    ax.tick_params(axis='both', which='major', labelsize=ticks_fontsize)

    # Заголовок
    title = f'Cyclogram {idx if idx is not None else ""}: Agents {type_data}'
    if relative:
        title += ", relative to Alpha"
    if limit is not None:
        title += f", last {limit} steps"
    # ax.set_title(title)
    ax.grid(True, linestyle='--', alpha=0.6)

    # Легенда за пределами картинки
    handles, labels = ax.get_legend_handles_labels()
    ax.legend(handles[::-1], labels[::-1], loc='upper left', fontsize=legend_fontsize,
              bbox_to_anchor=(1.05, 1), borderaxespad=0)

    # начало координат без смещения
    ax.margins(x=0, y=0)
    return ax