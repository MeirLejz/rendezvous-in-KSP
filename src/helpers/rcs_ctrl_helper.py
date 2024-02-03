import time
import logging

from src.helpers.space_center_helper import SpaceCenterHelper


class RCSCtrlHelper:
    def __init__(self, vessel, space_center_helper: SpaceCenterHelper) -> None:
        self.vessel = vessel
        self.space_center_helper = space_center_helper

    def enable_rcs(self) -> None:
        self.vessel.control.rcs = True
        time.sleep(1)
        logging.info("RCS activated")

    def disable_rcs(self) -> None:
        self.vessel.control.rcs = False
        time.sleep(1)
        logging.info("RCS deactivated")

    def compute_available_acceleration(self) -> (tuple, tuple):
        # Unpack the tuples
        tuple1, tuple2 = self.vessel.available_rcs_force

        # Divide elements in inner tuples by the divisor
        right_forward_bottom = tuple(value / self.vessel.mass for value in tuple1)
        left_backward_up = tuple(value / self.vessel.mass for value in tuple2)

        return (right_forward_bottom, left_backward_up)

    def rcs_actuation(self, U_BODY: tuple) -> None:
        # available_acceleration = tuple(
        #     element / self.vessel.mass for element in self.vessel.available_rcs_force
        # )
        available_acceleration = self.compute_available_acceleration()

        controls = [
            self.vessel.control.right,
            self.vessel.control.forward,
            self.vessel.control.up,
        ]
        u_values = [
            U_BODY[0],
            U_BODY[1],
            -U_BODY[2],
        ]
        for i in range(3):
            if u_values[i] >= 0:
                controls[i] = u_values[i] / abs(available_acceleration[0][i])
            else:
                controls[i] = u_values[i] / abs(available_acceleration[1][i])

        (
            self.vessel.control.right,
            self.vessel.control.forward,
            self.vessel.control.up,
        ) = controls

        # if U_BODY[0] >= 0:
        #     self.vessel.control.right = U_BODY[0] / abs(
        #         available_acceleration[0][0]
        #     )  # right

        # else:
        #     self.vessel.control.right = U_BODY[0] / abs(
        #         available_acceleration[1][0]
        #     )  # left

        # if U_BODY[1] >= 0:
        #     self.vessel.control.forward = U_BODY[1] / abs(
        #         available_acceleration[0][1]  # forward
        #     )
        # else:
        #     self.vessel.control.forward = U_BODY[1] / abs(
        #         available_acceleration[1][1]  # backward
        #     )

        # if U_BODY[2] >= 0:
        #     self.vessel.control.up = -U_BODY[2] / abs(
        #         available_acceleration[1][2]  # top
        #     )
        # else:
        #     self.vessel.control.up = -U_BODY[2] / abs(
        #         available_acceleration[0][2]  # bottom
        #     )
