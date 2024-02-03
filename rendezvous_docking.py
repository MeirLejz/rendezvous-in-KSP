import logging
import argparse
import time

from src.game_connector import KRPCConnector
from src.initialization.game_helper_init import GameHelperInit
from src.initialization.gnc_init import GNCInit
from src.initialization.mission_init import MissionInit

# class MissionFileHelper:
#     pass


def main():
    parser = argparse.ArgumentParser(
        description="Perform autonomous rendezvous between satellites in KSP using kRPC"
    )
    parser.add_argument(
        "--r_bar_safety_distance",
        type=float,
        help="height difference after phasing in meters",
        default=2000,
    )
    args = parser.parse_args()

    # Create a new rendezvous and docking mission
    connector = KRPCConnector("Rendezvous & Docking")
    logging.info("===== Input parameters =====")
    logging.info(
        f"R bar safety distance at the end of orbital phasing: {args.r_bar_safety_distance} m"
    )

    game_helper = GameHelperInit.game_helper_init(connector=connector)
    desired_apoapsis = game_helper.target.orbit.periapsis - args.r_bar_safety_distance
    logging.info(f"Chaser orbit desired apoapsis: {desired_apoapsis} m")

    # GN&C objects initialization
    gnc_helper = GNCInit.init_gnc_classes(
        game_helper=game_helper,
        connector=connector,
    )

    mission_phases = MissionInit.mission_init(
        game_helper=game_helper,
        gnc_helper=gnc_helper,
        desired_apoapsis=desired_apoapsis,
        phase_offset_end_phasing=3.0,
    )

    logging.info("===== Mission execution =====")

    # Mission file
    mission_phases.orbit_raise.execute_phase()
    mission_phases.orbit_phasing.execute_phase()
    # manually separate vessel stage
    game_helper.chaser.control.activate_next_stage()
    for antenna in game_helper.chaser.parts.antennas:
        if antenna.deployable:
            antenna.deployed = True
    for solar_panel in game_helper.chaser.parts.solar_panels:
        if solar_panel.deployable:
            solar_panel.deployed = True
    time.sleep(10)
    mission_phases.homing.execute_phase()

    mission_phases.close_range_maneuver.execute_phase(
        final_state=(0, 500, 0), duration=90
    )
    mission_phases.close_range_maneuver.execute_phase(
        final_state=(0, 100, 0), duration=60
    )
    mission_phases.close_range_maneuver.execute_phase(
        final_state=(0, 0, 30), duration=45
    )


if __name__ == "__main__":
    main()
