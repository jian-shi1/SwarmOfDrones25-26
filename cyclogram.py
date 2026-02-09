import numpy as np
from Drone import Drone
from typing import List, Dict, Optional, Tuple
import matplotlib.pyplot as plt

def cyclogram(drones: List[Drone],
              t_plot: np.ndarray,
              alpha_idx: int = 0,
              ax=None,
              relative: bool = True,
              limit: Optional[int] = None,
              idx: Optional[int] = None,
              type_data: str = 'Trajectories',
              drone_idx: Optional[int] = None):
    """
    Визуализация траекторий/скоростей/ускорений.

    Если drone_idx задан — рисуем только этот дрон (в абсолютных координатах).
    """
    if ax is None:
        fig, ax = plt.subplots(figsize=(8, 5))

    ticks_fontsize = 16
    labels_fontsize = 16
    legend_fontsize = 16

    t_plot = np.asarray(t_plot)
    N = len(drones)

    # Обрезаем времена
    if limit is not None and limit > 0:
        start = max(0, len(t_plot) - limit)
        plot_times = t_plot[start:]
    else:
        plot_times = t_plot

    # Собираем данные
    if drone_idx is not None:
        # Режим одного дрона
        data = np.zeros(len(plot_times))
        for j, t in enumerate(plot_times):
            x, v, a = drones[drone_idx].get_state_at(t)
            if type_data == "Trajectories":
                data[j] = x
            elif type_data == "Velocities":
                data[j] = v
            elif type_data == "Accelerations":
                data[j] = a
            else:
                raise ValueError("type_data must be 'Trajectories', 'Velocities', or 'Accelerations'")

        # Рисуем один дрон
        label = "Alpha" if drone_idx == alpha_idx else f"Drone {drone_idx}"
        ax.plot(plot_times, data, color='black', linewidth=2, label=label)
        ax.legend(fontsize=legend_fontsize)

    else:
        # Режим всех дронов (как раньше)
        data = np.zeros((len(plot_times), N))
        for i, drone in enumerate(drones):
            for j, t in enumerate(plot_times):
                x, v, a = drone.get_state_at(t)
                if type_data == "Trajectories":
                    data[j, i] = x
                elif type_data == "Velocities":
                    data[j, i] = v
                elif type_data == "Accelerations":
                    data[j, i] = a
                else:
                    raise ValueError("type_data must be 'Trajectories', 'Velocities', or 'Accelerations'")

        if relative:
            alpha_col = data[:, alpha_idx][:, np.newaxis]
            positions = data - alpha_col
        else:
            positions = data

        # Цвета
        cmap = plt.cm.tab20
        forward = [cmap(i % 20) for i in range(alpha_idx)][::-1]
        backward = [cmap(i % 20) for i in range(N - alpha_idx - 1)]
        colors = forward + ['black'] + backward

        if type_data == "Trajectories":
            for i in range(N):
                label = "Alpha" if i == alpha_idx else f"{i+1}"
                ms = 3 if i == alpha_idx else 1
                ax.plot(plot_times, positions[:, i],
                        label=label, marker='o', markersize=ms,
                        color=colors[i], linewidth=1.0)
        else:
            ax.plot(plot_times, positions[:, alpha_idx], color='black')

        if type_data == "Trajectories":
            handles, labels = ax.get_legend_handles_labels()
            ax.legend(handles[::-1], labels[::-1], loc='upper left', fontsize=legend_fontsize,
                      bbox_to_anchor=(1.05, 1), borderaxespad=0)

    # Оформление
    ax.set_xlabel('$t$ [s]', loc='right', fontsize=labels_fontsize)
    y_label = "$x$ [m]" if type_data == "Trajectories" else \
              "$v$ [m/s]" if type_data == "Velocities" else \
              "$a$ [$m/s^2$]"
    ax.set_ylabel(y_label, loc='top', fontsize=labels_fontsize, rotation=0)
    ax.yaxis.set_label_coords(x=-0.1, y=0.94, transform=ax.transAxes)
    ax.tick_params(axis='both', which='major', labelsize=ticks_fontsize)

    title = f'Cyclogram {idx if idx is not None else ""}: '
    if drone_idx is not None:
        title += f"Drone {drone_idx} ({type_data})"
    else:
        title += f"Agents {type_data}"
        if relative:
            title += ", relative to Alpha"
        if limit is not None:
            title += f", last {limit} steps"
    ax.set_title(title)
    ax.grid(True, linestyle='--', alpha=0.6)
    ax.margins(x=0, y=0)
    return ax
