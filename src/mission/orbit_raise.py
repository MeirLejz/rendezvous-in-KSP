import logging
import numpy as np

from src.initialization.game_helper_init import GameHelper
from src.mission.rdv_phase import RDVPhase


class OrbitRaise(RDVPhase):
    def __init__(self, game_helper: GameHelper, desired_apoapsis: float):
        self.game_helper = game_helper
        self.desired_apoapsis = desired_apoapsis

    def execute_phase(self) -> None:
        logging.info("===== Orbit raise phase =====")
        logging.info(f"Orbit raise phase performed to {self.desired_apoapsis} m")
        ta_raise_maneuver = (
            self.game_helper.target.orbit.argument_of_periapsis
            + self.game_helper.target.orbit.longitude_of_ascending_node
            - self.game_helper.chaser.orbit.argument_of_periapsis
            - self.game_helper.chaser.orbit.longitude_of_ascending_node
        )

        ta_raise_maneuver += 2 * np.pi if ta_raise_maneuver < 0 else ta_raise_maneuver

        desired_periapsis = self.game_helper.chaser.orbit.radius_at_true_anomaly(
            ta_raise_maneuver
        )
        maneuver_time = self.game_helper.chaser.orbit.ut_at_true_anomaly(
            ta_raise_maneuver
        )
        orbit_raise_semi_major_axis = (
            self.game_helper.orb_dyn.semi_maj_axis_from_apsises(
                periapsis=desired_periapsis,
                apoapsis=self.desired_apoapsis,
            )
        )
        logging.info(f"Planning and executing orbit raise node")
        self.game_helper.node_helper.plan_and_execute_node(
            radii=desired_periapsis,
            new_semi_maj_ax=orbit_raise_semi_major_axis,
            time=maneuver_time,
            absolute=True,
        )
        logging.info("===== Orbit raise phase finished =====")
