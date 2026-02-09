@dataclass
class SignalPayload:
    def __init__(self, sender_id: int, t_send: float, x: float, v: float, a: float):
        self.sender_id = sender_id
        self.t_send = t_send
        self.x = x
        self.v = v
        self.a = a