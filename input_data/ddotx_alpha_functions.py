import numpy as np


class RectlinearFlucts:
    """
    Пусть мы допускаем какую-то верхнюю (грубую) оценку скорости
    до которой будет разгоняться дрон. если мы хотим чтобы он всегда двигался в одном направлении
    то пусть его скорость будет v_max * sin^2 (pi/N * t), тогда ускорение модулируется
    как A * sin(2 pi /N * t). Отсюда A >= pi * v/N => v_max * pi / A <= N
    """
    def __init__(self, a_max = 10.0, v_max=33.3):
        self.a_max = a_max
        self.v_max = v_max
        self.period = np.pi * self.v_max / self.a_max
    
    def __call__(self, t):
        return self.a_max * np.sin(2.0 * np.pi * t / self.period)
    

class RoughBreak:
    def __init__(self, a_max = 10.0, v_max=40.0, t_plateau = 100):
        self.a_max = a_max
        self.v_max = v_max
        self.t1 = v_max / (a_max / 3)
        self.t2 = self.t1 + t_plateau
        self.t3 = self.t2 + self.t1
        self.zero = 0.0

    def __call__(self, t):
        if t < 0:
            return self.zero
        elif t < self.t1:
            return self.a_max
        elif t < self.t2:
            return self.zero
        elif t < self.t3:
            return -self.a_max
        else:
            return self.zero