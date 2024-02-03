import numpy as np
from control.matlab import lqr
from src.gnc.cw_linear_dynamics import Dynamics
from dataclasses import dataclass
from abc import ABC, abstractmethod


@dataclass
class LQRCost:
    Q: float
    R: float


class Control(ABC):
    def __init__(self) -> None:
        super().__init__()

    @abstractmethod
    def control(self, state: np.ndarray, ref: np.ndarray) -> np.ndarray:
        pass


class LQRControl:
    def __init__(self, costs: LQRCost, dynamics: Dynamics) -> None:
        self.optimal_gain, _, _ = lqr(
            dynamics.free_dynamics,
            dynamics.controlled_dynamics,
            costs.Q * np.eye(dynamics.free_dynamics.shape[0]),
            costs.R * np.eye(dynamics.controlled_dynamics.shape[1]),
        )
        # print(f"Q matrix: {costs.Q * np.eye(dynamics.free_dynamics.shape[0])}")
        # print(f"R matrix: {costs.R * np.eye(dynamics.controlled_dynamics.shape[1])}")

    def control(self, state: np.ndarray, ref: np.ndarray) -> np.ndarray:
        return -self.optimal_gain @ (state - ref)
