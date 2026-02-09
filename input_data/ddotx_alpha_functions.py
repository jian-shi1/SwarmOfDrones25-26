import numpy as np


def rectlinear_flucts(t):
    """
    функция с макс ускорением 10 м/с^2
    движемся всегда вправо, иногда замедляемся до 0, но плавно
    """
    return 10.0 * np.sin(2.0 * np.pi * t / 14.0)


def rough_break(t):
    """
    Сначала неспеша разгоняемся, 100 секунд гоним на максимальной скорости, потом очень резко тормозим
    Суммарно альфа агент в движении около 125 секунд
    """
    t1 = 34 / 3
    t2 = t1 + 100
    t3 = t2 + t1
    a_max = np.float128(10.0)
    zero = np.float128(0.0)

    if t < 0:
        return zero
    elif t < t1:
        return a_max
    elif t < t2:
        return zero
    elif t < t3:
        return -a_max
    else:
        return zero
