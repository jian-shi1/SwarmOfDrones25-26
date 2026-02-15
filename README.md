# SwarmOfDrones 25-26

## Подготовительные задачи вне основной модели взаимодействия дронов
1. [Рой в одномерном пространстве. Моделирование прямолинейного движения агентов, связанных пружинками.](./springs-1d-basics.ipynb)
2. [Оценка скоростей через моменты. Фильтр Калмана для сглаживания.](./measure_limitations.ipynb)
3. [Анализ траекторий с очень резкими поворотами при допущении о возможности передачи точных скоростей.](./Rough-trajectories.ipynb)

## Основная модель
 Наборы входных данных, на которых 20 симуляция проходит успешно
```py
    # N, alpha_idx, s0, H, T, a_max, m, ddotx_func, start_coordinates, R0, signals_order
    (10, 9, 0.5, 60, 1200, g, 5.0, RectlinearFlucts(a_max=g), np.arange(0.0, 29, 3), 100, np.arange(8, -1, -1)),
     (7, 6, 0.5, 40, 1200, g, 5.0, RectlinearFlucts(a_max=g), np.arange(0.0, 19, 3), 100, np.arange(5, -1, -1)),
    (6, 5, 0.5, 40, 1200, g*0.2, 5.0, RectlinearFlucts(a_max=g*0.4), np.arange(0.0, 16, 3), 100, np.arange(4, -1, -1)),
     (6, 5, 0.5, 40, 1200, g*0.5, 5.0, RectlinearFlucts(a_max=g*0.4), np.arange(0.0, 16, 3), 100, np.arange(4, -1, -1)),
    (6, 5, 0.5, 40, 1200, g*0.7, 5.0, RectlinearFlucts(a_max=g*0.4), np.arange(0.0, 16, 3), 100, np.arange(4, -1, -1)),
    (6, 5, 0.5, 40, 1200, g, 5.0, RectlinearFlucts(a_max=g), np.arange(0.0, 16, 3), 100, np.arange(4, -1, -1)),
    (6, 5, 0.5, 40, 1200, g, 5.0, RectlinearFlucts(a_max=g), np.array([0.0, 5.0, 10.0, 15.0, 20.0, 25.0]), 100, np.arange(4, -1, -1)),
    (6, 5, 0.5, 40, 1200, g, 5.0, RectlinearFlucts(a_max=g), np.array([0.0, 8.0, 16.0, 24.0, 32.0, 40.0]), 100, np.arange(4, -1, -1))
```