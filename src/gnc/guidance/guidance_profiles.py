from numpy.polynomial import Polynomial
from numpy import ndarray
from dataclasses import dataclass
from abc import ABC, abstractmethod


@dataclass
class GuidanceParameters:
    # polynomial coefficients for the desired trajectory
    norm_pos_pol_coeff: ndarray  # [0, 0, 0, 10, -15, 6]
    norm_vel_pol_coeff: ndarray  # [0, 0, 30, -60, 30]
    norm_acc_pol_coeff: ndarray  # [0, 60, -180, 120]


class Profile(ABC):
    def __init__(self) -> None:
        super().__init__()

    @abstractmethod
    def config_profile(self) -> None:
        pass

    @abstractmethod
    def pos(self):
        pass

    @abstractmethod
    def vel(self):
        pass


class SmoothProfile(Profile):
    def __init__(self, guid_params: GuidanceParameters) -> None:
        self.guid_params = guid_params
        self.normalized_pos = Polynomial(self.guid_params.norm_pos_pol_coeff)
        self.normalized_vel = Polynomial(self.guid_params.norm_vel_pol_coeff)

    def config_profile(
        self,
        p_i: float,
        p_f: float,
        duration: float,
    ) -> None:
        self.p_i = p_i
        self.p_f = p_f
        self.duration = duration

    # , p_f: float, p_i: float
    def pos(self) -> Polynomial:
        return self.normalized_pos * (self.p_f - self.p_i) + self.p_i

    # , p_f: float, p_i: float, duration: float
    def vel(self) -> Polynomial:
        return self.normalized_vel * (self.p_f - self.p_i) / self.duration
