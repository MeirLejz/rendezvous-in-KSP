import time
import logging

from src.helpers.space_center_helper import SpaceCenterHelper


class AttCtrlHelper:
    def __init__(self, vessel, space_center_helper: SpaceCenterHelper):
        self.vessel = vessel
        self.space_center_helper = space_center_helper

    def enable_sas(self) -> None:
        self.vessel.control.sas = True
        time.sleep(1)
        logging.info("SAS enabled")

    def disable_sas(self) -> None:
        self.vessel.control.sas = False
        time.sleep(1)
        logging.info("SAS disabled")

    def change_sas_mode(self, mode) -> None:
        if self.vessel.control.sas:
            self.vessel.control.sas_mode = self.space_center_helper.sas_mode(mode)
            logging.info(f"SAS mode changed to {mode}")
        else:
            pass  # TODO raise exception

    def orient_vessel(self, direction: tuple) -> None:
        ap = self.vessel.auto_pilot
        rf = self.vessel.orbit.body.reference_frame

        ap.reference_frame = rf
        ap.engage()
        time.sleep(0.1)
        logging.info("Autopilot engaged")
        ap.target_direction = direction
        # ap.target_direction = self.vessel.control.nodes[0].remaining_burn_vector(rf)
        time.sleep(0.1)
        ap.wait()
        time.sleep(0.1)
        logging.info("Finished autopilot wait")
        while ap.error > 0.5:  # deg
            pass
        ap.disengage()
        time.sleep(0.1)
        logging.info("Autopilot disengaged")

    def change_speed_mode(self, mode) -> None:
        self.vessel.control.speed_mode = self.space_center_helper.speed_mode(mode)
        logging.info(f"Speed mode changed to {mode}")
