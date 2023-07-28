from typing import Optional


class LP:
    def __init__(self, alpha, x: Optional[float] = None):
        self.alpha = alpha
        self.x = x

    def __call__(self, x: float) -> float:
        if self.x is None:
            self.x = x
            return self.x

        self.x += self.alpha * (x - self.x)
        return self.x  # type: ignore

    def reset(self, x: float = None):
        self.x = x

    def get(self) -> Optional[float]:
        return self.x
