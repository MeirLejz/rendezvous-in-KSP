from numpy import ndarray, array, cos, sin
from abc import ABC, abstractmethod
from typing import List

from src.gnc.guidance.guidance_profiles import Profile


class Guidance(ABC):
    def __init__(self) -> None:
        super().__init__()

    @abstractmethod
    def ref_signal(self, tau: float) -> ndarray:
        pass


class SmoothGuidance(Guidance):
    def __init__(
        self,
        profiles: List[Profile],
    ):
        self.profiles = profiles

    def ref_signal(self):
        def func(tau) -> (ndarray, ndarray):
            in_plane = array(
                [
                    [self.profiles[0].pos()(tau)],
                    [self.profiles[1].pos()(tau)],
                    [self.profiles[0].vel()(tau)],
                    [self.profiles[1].vel()(tau)],
                ]
            )
            out_of_plane = array(
                [
                    [self.profiles[2].pos()(tau)],
                    [self.profiles[2].vel()(tau)],
                ]
            )
            return in_plane, out_of_plane

        return func


class CWGuidance(Guidance):
    def __init__(self, orbital_rate: float):
        self.n = orbital_rate

    # anti-radial, R-bar (z in the paper)
    def ref_signal(
        self,
        initial_pos: tuple,
        initial_vel: tuple,
    ):
        (x_0, y_0, z_0) = initial_pos
        (x_dot_0, y_dot_0, z_dot_0) = initial_vel

        def func(tau) -> (ndarray, ndarray):
            x = (
                (2 * y_dot_0 / self.n - 3 * x_0) * cos(self.n * tau)
                + x_dot_0 / self.n * sin(self.n * tau)
                + (4 * x_0 - 2 * y_dot_0 / self.n)
            )
            y = (
                (4 * y_dot_0 / self.n - 6 * x_0) * sin(self.n * tau)
                - 2 * x_dot_0 / self.n * cos(self.n * tau)
                + (6 * self.n * x_0 - 3 * y_dot_0) * tau
                + (y_0 + 2 * x_dot_0 / self.n)
            )
            z = z_0 * cos(self.n * tau) + z_dot_0 / self.n * sin(self.n * tau)
            x_dot = -self.n * (2 * y_dot_0 / self.n - 3 * x_0) * sin(
                self.n * tau
            ) + x_dot_0 * cos(self.n * tau)
            y_dot = (
                self.n * (4 * y_dot_0 / self.n - 6 * x_0) * cos(self.n * tau)
                + 2 * x_dot_0 * sin(self.n * tau)
                + (6 * self.n * x_0 - 3 * y_dot_0)
            )
            z_dot = -self.n * z_0 * sin(self.n * tau) + z_dot_0 * cos(self.n * tau)
            return array([[x], [y], [x_dot], [y_dot]]), array([[z], [z_dot]])

        return func
