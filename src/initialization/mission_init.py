from src.mission.orbit_raise import OrbitRaise
from src.mission.phasing import OrbitPhasing
from src.mission.homing import Homing
from src.mission.close_range import CloseRangeManeuver

from src.initialization.game_helper_init import GameHelper
from src.initialization.gnc_init import GNCHelper
from src.mission.rdv_phase import RDVPhase


class Mission:
    def __init__(
        self,
        orbit_raise: RDVPhase,
        orbit_phasing: RDVPhase,
        homing: RDVPhase,
        close_range_maneuver: RDVPhase,
    ):
        self.orbit_raise = orbit_raise
        self.orbit_phasing = orbit_phasing
        self.homing = homing
        self.close_range_maneuver = close_range_maneuver


class MissionInit:
    @classmethod
    def mission_init(
        cls,
        game_helper: GameHelper,
        gnc_helper: GNCHelper,
        desired_apoapsis: float,
        phase_offset_end_phasing: float,
    ):
        raiser = OrbitRaise(
            game_helper=game_helper,
            desired_apoapsis=desired_apoapsis,
        )
        phaser = OrbitPhasing(
            game_helper=game_helper,
            desired_apoapsis=desired_apoapsis,
            desired_final_phase=phase_offset_end_phasing,
        )
        homer = Homing(game_helper=game_helper, gnc_helper=gnc_helper)

        close_range_maneuver = CloseRangeManeuver(
            game_helper=game_helper,
            gnc_helper=gnc_helper,
        )

        return Mission(
            orbit_raise=raiser,
            orbit_phasing=phaser,
            homing=homer,
            close_range_maneuver=close_range_maneuver,
        )
