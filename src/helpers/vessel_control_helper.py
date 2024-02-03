import math
import logging
import time

from physics.orb_dyn_utils import OrbitalDynamicsUtils, CelestialBodyParameters
from src.helpers.space_center_helper import SpaceCenterHelper
from src.helpers.stream_helper import StreamHelper


class VesselController:
    def __init__(
        self,
        vessel,
        params: CelestialBodyParameters,
        orb_dyn: OrbitalDynamicsUtils,
        space_center_helper: SpaceCenterHelper,
        stream_helper: StreamHelper,
    ) -> None:
        self.vessel = vessel
        self.params = params
        self.orb_dyn = orb_dyn
        self.space_center_helper = space_center_helper
        self.stream_helper = stream_helper

    @property
    def next_node_burn_time(self) -> float:
        vessel = self.vessel
        node = vessel.control.nodes[0]

        mass = vessel.mass
        isp = vessel.specific_impulse
        dv = node.delta_v
        avail_thrust = vessel.available_thrust
        g = self.params.body_surface_gravity

        burn_time = (mass - (mass / math.exp(dv / (isp * g)))) / (
            avail_thrust / (isp * g)
        )
        return burn_time

    def plan_and_execute_node(
        self,
        radii: float,
        new_semi_maj_ax: float,
        time: float,
        absolute: bool = False,
        direction: str = "prograde",
    ) -> None:
        dv = self.orb_dyn.delta_v(
            radii=radii,
            current_semi_major_axis=self.vessel.orbit.semi_major_axis,
            new_semi_major_axis=new_semi_maj_ax,
        )
        self.add_node(
            dv=dv,
            time=time,
            absolute=absolute,
            direction=direction,
        )

        self.execute_next_node()

    def thrust_controller(self) -> float:
        TWR = self.vessel.available_thrust / self.vessel.mass
        remain_dv = self.vessel.control.nodes[0].remaining_delta_v
        if remain_dv / TWR > 1:
            return 1.0
        else:
            return math.log10(remain_dv / TWR * 9 + 1)

    def orient_vessel_towards_node(self) -> None:
        ap = self.vessel.auto_pilot
        rf = self.vessel.orbit.body.reference_frame

        ap.reference_frame = rf
        ap.engage()
        time.sleep(0.1)
        logging.info("Autopilot engaged")
        ap.target_direction = self.vessel.control.nodes[0].remaining_burn_vector(rf)
        time.sleep(0.1)
        ap.wait()
        time.sleep(0.1)
        logging.info("Finished autopilot wait")
        while ap.error > 0.5:  # deg
            pass
        ap.disengage()
        time.sleep(0.1)
        logging.info("Autopilot disengaged")

    def execute_next_node(self) -> None:
        logging.info("----- Node execution ------")
        node = self.vessel.control.nodes[0]
        self.orient_vessel_towards_node()
        self.enable_sas()
        self.change_sas_mode("Maneuver")

        next_node_burn_time = self.next_node_burn_time
        logging.info(f"Nominal burn time: {next_node_burn_time} s")

        not_warped_time_before_burn = 5.0
        self.space_center_helper.warp_time(
            node.ut - (next_node_burn_time / 2.0) - not_warped_time_before_burn,
            absolute=True,
        )
        while node.time_to > (next_node_burn_time / 2.0):
            pass

        previous = node.remaining_delta_v + 1
        while node.remaining_delta_v > 0.1 and node.remaining_delta_v <= previous:
            self.vessel.control.throttle = self.thrust_controller()
            previous = node.remaining_delta_v

        self.vessel.control.throttle = 0.0
        time.sleep(0.1)
        logging.info(
            f"remaining dv = {node.remaining_delta_v}, throttle = {self.vessel.control.throttle}"
        )

        logging.info("Burn completed")

        self.change_sas_mode("Stability Assist")
        node.remove()

        logging.info("Maneuver node removed")
        logging.info("----- End of node execution -----")

    def enable_sas(self) -> None:
        self.vessel.control.sas = True
        time.sleep(1)
        logging.info("SAS enabled")

    def disable_sas(self) -> None:
        self.vessel.control.sas = False
        time.sleep(1)
        logging.info("SAS disabled")

    def enable_rcs(self) -> None:
        self.vessel.control.rcs = True
        time.sleep(1)
        logging.info("RCS activated")

    def disable_rcs(self) -> None:
        self.vessel.control.rcs = False
        time.sleep(1)
        logging.info("RCS deactivated")

    def change_sas_mode(self, mode) -> None:
        if self.vessel.control.sas:
            self.vessel.control.sas_mode = self.space_center_helper.sas_mode(mode)
            logging.info(f"SAS mode changed to {mode}")
        else:
            pass  # TODO raise exception

    def change_speed_mode(self, mode) -> None:
        self.vessel.control.speed_mode = self.space_center_helper.speed_mode(mode)
        logging.info(f"Speed mode changed to {mode}")

    def rotate_orbital_to_body(self, U):
        U_BODY = self.space_center_helper.space_center.transform_position(
            U,
            self.vessel.orbital_reference_frame,
            self.vessel.reference_frame,
        )

        return U_BODY

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

        if U_BODY[0] >= 0:
            self.vessel.control.right = U_BODY[0] / abs(
                available_acceleration[0][0]
            )  # right

        else:
            self.vessel.control.right = U_BODY[0] / abs(
                available_acceleration[1][0]
            )  # left

        if U_BODY[1] >= 0:
            self.vessel.control.forward = U_BODY[1] / abs(
                available_acceleration[0][1]  # forward
            )
        else:
            self.vessel.control.forward = U_BODY[1] / abs(
                available_acceleration[1][1]  # backward
            )

        if U_BODY[2] >= 0:
            self.vessel.control.up = -U_BODY[2] / abs(
                available_acceleration[1][2]  # top
            )
        else:
            self.vessel.control.up = -U_BODY[2] / abs(
                available_acceleration[0][2]  # bottom
            )
