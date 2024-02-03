import math
from dataclasses import dataclass


@dataclass
class CelestialBodyParameters:
    gravitational_parameter: float  # m^3/s^2
    body_surface_gravity: float  # m/s^2
    body_equatorial_radius: float  # m


class OrbitalDynamicsUtils:
    def __init__(self, celestial_body_params: CelestialBodyParameters) -> None:
        self.params = celestial_body_params

    def phase_offset(
        self,
        target_ta_at_chaser_apoapsis: float,
        target_arg_of_periapsis: float,
        chaser_arg_of_periapsis: float,
    ) -> float:
        Phi = (
            target_ta_at_chaser_apoapsis
            + target_arg_of_periapsis
            - (math.pi + chaser_arg_of_periapsis)
        )
        # print(
        #     f"target_ta_at_chaser_apoapsis: {target_ta_at_chaser_apoapsis * 180 / math.pi:.2f}"
        # )
        # print(f"target_arg_of_periapsis: {target_arg_of_periapsis * 180 / math.pi:.2f}")
        # print(f"chaser_arg_of_periapsis: {chaser_arg_of_periapsis * 180 / math.pi:.2f}")
        # print(f"Phi: {Phi * 180 / math.pi:.2f}")
        return Phi

    def eccentric_anomaly_difference(
        self, target_eccentricity: float, phase_offset: float
    ) -> float:
        delta_E = 2 * math.atan(
            math.sqrt((1 - target_eccentricity) / (1 + target_eccentricity))
            * math.tan(phase_offset / 2)
        )
        return delta_E

    def time_difference(
        self,
        eccentric_anomaly_difference: float,
        target_eccentricity: float,
        target_period: float,
    ) -> float:
        dE = eccentric_anomaly_difference
        e = target_eccentricity
        T_t = target_period
        # time needed to cover difference of phase angle
        t = (dE - e * math.sin(dE)) * T_t / (2 * math.pi)
        return t

    def phasing_period_validity(self, T_phasing: float, apoapsis: float) -> bool:
        a = self.semi_maj_axis_from_period(T_phasing)
        periapsis = 2 * a - apoapsis
        return periapsis > (self.params.body_equatorial_radius + 100000)

    def orbital_velocity(self, radii: float, semi_major_axis: float) -> float:
        """
        Calculate orbital velocity at radius r given semi-major axis a and gravitational parameter mu
        inputs: radii in m
                semi_major_axis in m
        output: orbital velocity in m/s
        """
        return math.sqrt(
            self.params.gravitational_parameter
            * ((2.0 / radii) - (1.0 / semi_major_axis))
        )

    def delta_v(
        self,
        radii: float,
        current_semi_major_axis: float,
        new_semi_major_axis: float,
    ) -> float:
        """
        Calculate delta-v
        inputs: radii in m
                current_semi_major_axis in m
                new_semi_major_axis in m
        output: delta-v in m/s
        """
        v1 = self.orbital_velocity(radii, current_semi_major_axis)
        v2 = self.orbital_velocity(radii, new_semi_major_axis)
        return v2 - v1

    def orbital_rate(self, semi_major_axis: float) -> float:
        """Calculate orbital rate

        Args:
            semi_major_axis (float): semi-major axis in m

        Returns:
            float: orbital rate in rad/s
        """
        return math.sqrt(self.params.gravitational_parameter / semi_major_axis**3)

    def semi_maj_axis_from_apsises(self, periapsis: float, apoapsis: float) -> float:
        """Calculate semi-major axis from periapsis and apoapsis

        Args:
            periapsis (float): periapsis in meters
            apoapsis (float): apoapsis in meters

        Returns:
            float: semi-major axis in meters
        """
        return (periapsis + apoapsis) / 2.0

    def semi_maj_axis_from_period(self, period: float) -> float:
        return (
            period * math.sqrt(self.params.gravitational_parameter) / (2 * math.pi)
        ) ** (2.0 / 3.0)

    def apsis_from_period_and_2nd_apsis(self, period: float, apsis: float) -> float:
        return 2 * self.semi_maj_axis_from_period(period) - apsis

    def format_time(self, time: float) -> str:
        """Format time in seconds to a string of the form 'x h, y min, z s'

        Args:
            time (float): time in seconds

        Returns:
            str: formatted period
        """
        hours = int(time // 3600)
        minutes = int((time % 3600) // 60)
        seconds = int(time % 60)
        return f"{hours} h, {minutes} min, {seconds} s"
