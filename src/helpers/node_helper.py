import math
import logging
import time

from src.physics.orb_dyn_utils import OrbitalDynamicsUtils, CelestialBodyParameters
from src.helpers.stream_helper import StreamHelper
from src.helpers.att_ctrl_helper import AttCtrlHelper
from src.helpers.space_center_helper import SpaceCenterHelper
from src.helpers.rcs_ctrl_helper import RCSCtrlHelper


class NodeHelper:
    def __init__(
        self,
        vessel,
        orb_dyn: OrbitalDynamicsUtils,
        params: CelestialBodyParameters,
        stream_helper: StreamHelper,
        space_center_helper: SpaceCenterHelper,
        att_ctrl_helper: AttCtrlHelper,
        rcs_ctrl_helper: RCSCtrlHelper,
    ) -> None:
        self.vessel = vessel
        self.orb_dyn = orb_dyn
        self.params = params
        self.stream_helper = stream_helper
        self.space_center_helper = space_center_helper
        self.att_ctrl_helper = att_ctrl_helper
        self.rcs_ctrl_helper = rcs_ctrl_helper

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

    def add_node(
        self,
        dv: float,
        time: float,
        absolute: bool = False,
        direction: str = "prograde",
    ) -> None:
        if not absolute:
            time += self.stream_helper.ut()
        if direction == "prograde":
            self.vessel.control.add_node(time, prograde=dv)
        elif direction == "normal":
            self.vessel.control.add_node(time, normal=dv)
        elif direction == "radial":
            self.vessel.control.add_node(time, radial=dv)
        else:
            raise ValueError("Invalid direction")
        logging.info(
            f"Maneuver node with delta V = {dv:.2f} m/s added in the {direction} direction"
        )

    def thrust_controller(self) -> float:
        TWR = self.vessel.available_thrust / self.vessel.mass
        remain_dv = self.vessel.control.nodes[0].remaining_delta_v
        if remain_dv / TWR > 1:
            return 1.0
        else:
            return math.log10(remain_dv / TWR * 9 + 1)

    def rcs_node_execution(self) -> None:
        logging.info("----- Node execution - with RCS ------")

        self.att_ctrl_helper.enable_sas()
        self.att_ctrl_helper.change_sas_mode("Prograde")
        self.rcs_ctrl_helper.enable_rcs()

        node = self.vessel.control.nodes[0]
        burn_direction = node.direction(self.vessel.reference_frame)  # unit vector

        if self.stream_helper.ut() < node.ut - 30:
            self.space_center_helper.warp_time(
                node.ut - 30,
                absolute=True,
            )

        while self.stream_helper.ut() < node.ut - 5:
            pass

        previous = node.remaining_delta_v + 1

        while node.remaining_delta_v > 0.1:  # and node.remaining_delta_v <= previous:
            self.vessel.control.right = burn_direction[0]
            self.vessel.control.forward = burn_direction[1]
            self.vessel.control.up = -burn_direction[2]
            previous = node.remaining_delta_v

        self.vessel.control.right = 0.0
        self.vessel.control.forward = 0.0
        self.vessel.control.up = 0.0

        logging.info("Burn completed")
        logging.info("----- End of node execution -----")
        node.remove()

    def execute_next_node(self) -> None:
        logging.info("----- Node execution ------")
        node = self.vessel.control.nodes[0]
        rf = self.vessel.orbit.body.reference_frame

        self.att_ctrl_helper.orient_vessel(direction=node.remaining_burn_vector(rf))
        self.att_ctrl_helper.enable_sas()
        self.att_ctrl_helper.change_sas_mode("Maneuver")

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
        while node.remaining_delta_v > 0.1:  # and node.remaining_delta_v <= previous:
            self.vessel.control.throttle = self.thrust_controller()
            # previous = node.remaining_delta_v

        self.vessel.control.throttle = 0.0
        time.sleep(0.1)
        logging.info(
            f"remaining dv = {node.remaining_delta_v}, throttle = {self.vessel.control.throttle}"
        )

        logging.info("Burn completed")

        self.att_ctrl_helper.change_sas_mode("Stability Assist")
        node.remove()

        logging.info("Maneuver node removed")
        logging.info("----- End of node execution -----")
