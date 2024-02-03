from abc import ABC, abstractmethod
import numpy as np


class Dynamics(ABC):
    def __init__(self) -> None:
        super().__init__()

    @property
    @abstractmethod
    def free_dynamics(self) -> np.ndarray:
        pass

    @property
    @abstractmethod
    def controlled_dynamics(self) -> np.ndarray:
        pass


class InPlaneDynamics(Dynamics):
    def __init__(self, orbital_rate: float) -> None:
        self.n = orbital_rate

    @property
    def free_dynamics(self) -> np.ndarray:
        return np.array(
            [
                [0, 0, 1, 0],
                [0, 0, 0, 1],
                [3 * self.n**2, 0, 0, -2 * self.n],
                [0, 0, 2 * self.n, 0],
            ]
        )

    @property
    def controlled_dynamics(self) -> np.ndarray:
        return np.array([[0, 0], [0, 0], [1, 0], [0, 1]])


class OutOfPlaneDynamics(Dynamics):
    def __init__(self, orbital_rate: float) -> None:
        self.n = orbital_rate

    @property
    def free_dynamics(self) -> np.ndarray:
        return np.array([[0, 1], [-self.n**2, 0]])

    @property
    def controlled_dynamics(self) -> np.ndarray:
        return np.array([[0], [1]])
