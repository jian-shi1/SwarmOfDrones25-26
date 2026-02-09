def compute_direction(i: int, j: int) -> float:
    """Направление: +1 если j > i, -1 если j < i"""
    return 1.0 if j > i else -1.0

def safe_divide(a, b, eps=1e-8):
    return a / max(b, eps)
