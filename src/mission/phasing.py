import math
import logging

from src.initialization.game_helper_init import GameHelper
from src.mission.rdv_phase import RDVPhase


class OrbitPhasing(RDVPhase):
    def __init__(
        self,
        game_helper: GameHelper,
        desired_apoapsis: float,
        desired_final_phase: float,  # degrees
    ):
        self.game_helper = game_helper
        self.desired_apoapsis = desired_apoapsis
        self.desired_final_phase = desired_final_phase
        self.delta_t = 0
        self.T_phasing = 0
        self.behind = True
        self.phi = 0

    @property
    def behind(self):
        return self.phi >= 0

    @behind.setter
    def behind(self, value: bool):
        self._behind = value

    def phase_difference(self) -> float:
        # true anomaly of target when chaser reaches apoapsis
        vessel_time_at_apoapsis = self.game_helper.chaser.orbit.time_to_apoapsis
        target_ta_at_chaser_apoapsis = (
            self.game_helper.target.orbit.true_anomaly_at_ut(
                self.game_helper.stream_helper.ut() + vessel_time_at_apoapsis
            )
            - self.desired_final_phase * math.pi / 180
        )
        if target_ta_at_chaser_apoapsis < 0:
            target_ta_at_chaser_apoapsis += 2 * math.pi

        self.phi = self.game_helper.orb_dyn.phase_offset(
            target_ta_at_chaser_apoapsis=target_ta_at_chaser_apoapsis,
            target_arg_of_periapsis=self.game_helper.target.orbit.argument_of_periapsis,
            chaser_arg_of_periapsis=self.game_helper.chaser.orbit.argument_of_periapsis,
        )

    def time_difference(self) -> None:
        # difference of eccentric anomaly between target and vessel at apoapsis
        delta_E = self.game_helper.orb_dyn.eccentric_anomaly_difference(
            target_eccentricity=self.game_helper.target.orbit.eccentricity,
            phase_offset=abs(self.phi),
        )

        self.delta_t = self.game_helper.orb_dyn.time_difference(
            eccentric_anomaly_difference=delta_E,
            target_eccentricity=self.game_helper.target.orbit.eccentricity,
            target_period=self.game_helper.target.orbit.period,
        )

    def compute_phasing_period(self, n_phasing_orbits: int = 3) -> None:
        T_target = self.game_helper.target.orbit.period
        self.T_phasing = (
            T_target - self.delta_t / float(n_phasing_orbits)
            if self.behind
            else T_target + self.delta_t / float(n_phasing_orbits)
        )

    def update_phasing_period(self) -> int:
        k = 1
        ref_apsis = self.game_helper.orb_dyn.apsis_from_period_and_2nd_apsis(
            self.T_phasing, self.game_helper.chaser.orbit.apoapsis
        )

        while True:
            if (
                ref_apsis
                < self.game_helper.orb_dyn_params.body_equatorial_radius + 100000
            ) or (ref_apsis > self.game_helper.target.orbit.apoapsis * 1.3):
                k += 1
                self.compute_phasing_period(k)
                logging.info(
                    f"Adding phasing orbit execution until rdv (total: {k} orbit executions) by making it closer to target orbit"
                )
                ref_apsis = self.game_helper.orb_dyn.apsis_from_period_and_2nd_apsis(
                    self.T_phasing, self.game_helper.chaser.orbit.apoapsis
                )
            else:
                break

        return k

    def execute_phasing_maneuvers(
        self,
        a_phasing: float,
        n_phasing_orbit: int,
        circ_maneuver_pos: str,
    ) -> None:
        self.game_helper.node_helper.plan_and_execute_node(
            radii=self.desired_apoapsis,
            new_semi_maj_ax=a_phasing,
            time=self.game_helper.chaser.orbit.time_to_apoapsis,
            absolute=False,
            direction="prograde",
        )

        if circ_maneuver_pos == "apoapsis":
            circ_maneuver_time = (
                self.game_helper.chaser.orbit.time_to_apoapsis
                + self.T_phasing * (n_phasing_orbit - 1)
            )
        else:
            circ_maneuver_time = (
                self.game_helper.chaser.orbit.time_to_periapsis
                + self.T_phasing * (n_phasing_orbit - 1)
            )

        height_difference = (
            self.game_helper.target.orbit.apoapsis - self.desired_apoapsis
        )
        self.game_helper.node_helper.plan_and_execute_node(
            radii=self.desired_apoapsis,
            new_semi_maj_ax=self.game_helper.target.orbit.semi_major_axis
            - 2 * height_difference,
            time=circ_maneuver_time,  # if chaser was ahead of target, maneuver node will be at new periapsis
            absolute=False,
            direction="prograde",
        )

    def execute_phase(self) -> None:
        logging.info("===== Phasing phase =====")
        self.phase_difference()
        self.time_difference()
        self.compute_phasing_period(n_phasing_orbits=3)

        # n_phasing_orbit = self.update_phasing_period()
        n_phasing_orbit = 3

        # log if chaser is ahead or behind of target, phase difference, time difference and phasing period
        logging.info("Summary of calculations for phasing:")
        logging.info(f"Chaser is {'behind' if self.behind else 'ahead'} of target")
        logging.info(
            f"Phase difference between target and chaser at chaser apoapsis: {(self.phi * 180 / math.pi):.2f} degrees"
        )
        logging.info(
            f"Time needed to cover difference of phase angle: {self.delta_t} s or {self.game_helper.orb_dyn.format_time(self.delta_t)}"
        )
        logging.info(
            f"Phasing orbit period: {self.game_helper.orb_dyn.format_time(self.T_phasing)}"
        )
        # log number of phasing orbits until rdv
        logging.info(f"Number of phasing orbits needed: {n_phasing_orbit}")
        logging.info(
            f"Target orbit period: {self.game_helper.orb_dyn.format_time(self.game_helper.target.orbit.period)}"
        )
        self.execute_phasing_maneuvers(
            a_phasing=self.game_helper.orb_dyn.semi_maj_axis_from_period(
                self.T_phasing
            ),
            n_phasing_orbit=n_phasing_orbit,
            circ_maneuver_pos="apoapsis" if self.behind else "periapsis",
        )
        logging.info("===== Phasing phase finished =====")
