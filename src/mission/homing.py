import math
import logging
import time
from numpy import array

from src.mission.rdv_phase import RDVPhase
from src.initialization.game_helper_init import GameHelper
from src.initialization.gnc_init import GNCHelper


class Homing(RDVPhase):
    def __init__(self, game_helper: GameHelper, gnc_helper: GNCHelper) -> None:
        self.game_helper = game_helper
        self.gnc_helper = gnc_helper

    # def prograde_drift_time(
    #     self,
    #     height_difference: float,
    #     prograde_drift_distance: float,
    # ) -> float:
    #     """Calculate time to drift in prograde direction

    #     Args:
    #         target_period (float): target orbit period in seconds
    #         height_difference (float): height difference between target and chaser in meters
    #         prograde_drift_distance (float): distance to drift in prograde direction in meters

    #     Returns:
    #         float: time to drift in prograde direction in seconds
    #     """
    #     w = 2 * math.pi / self.game_helper.target.orbit.period
    #     return 2.0 / 3.0 * prograde_drift_distance / (height_difference * w)

    def cw_hohmann_transfer(self) -> tuple:
        w = 2 * math.pi / self.game_helper.target.orbit.period
        delta_h = (
            self.game_helper.target.orbit.radius - self.game_helper.chaser.orbit.radius
        )
        if delta_h < 0:
            logging.error("Chaser orbit is above target orbit")
            raise ValueError("Chaser orbit is above target orbit")

        delta_y = abs(delta_h) * 3 * math.pi / 4
        delta_v = w / 4 * abs(delta_h)
        y_f = 1000  # hardcoded
        y_i = y_f - delta_y
        return y_i, delta_h, delta_v, delta_y

    def execute_phase(self) -> None:
        logging.info("===== Homing Phase =====")

        T_target = self.game_helper.target.orbit.period
        w = 2 * math.pi / T_target

        self.game_helper.rcs_ctrl_helper.enable_rcs()
        self.game_helper.att_ctrl_helper.enable_sas()
        self.game_helper.att_ctrl_helper.change_sas_mode("Prograde")
        self.game_helper.att_ctrl_helper.change_speed_mode("Orbit")

        y_i, delta_h, delta_v, delta_y = self.cw_hohmann_transfer()
        logging.info(f"Initial position for maneuver in prograde direction = {y_i} m")
        logging.info(f"Altitude difference = {delta_h} m")
        logging.info(f"Delta V in the prograde direction = {delta_v} m/s")
        logging.info(f"Distance in prograde direction as a result of dv = {delta_y} m")

        time.sleep(5)
        self.game_helper.space_center_helper.warp_factor(2)

        while self.game_helper.stream_helper.rel_pos()[1] < y_i:
            time.sleep(1.0)
            y_i, _, _, _ = self.cw_hohmann_transfer()
        self.game_helper.space_center_helper.warp_factor(0)
        time.sleep(3.0)
        y_i, delta_h, delta_v, delta_y = self.cw_hohmann_transfer()

        # print(f"y={self.game_helper.stream_helper.rel_pos()[1]} m")
        # print(
        #     f"y_i={y_i} m, delta_h={delta_h} m, delta_v={delta_v} m/s, delta_y={delta_y} m"
        # )

        in_plane_nav, out_of_plane_nav = self.gnc_helper.navigation.output()
        ref_signal = self.gnc_helper.cw_guidance.ref_signal(
            initial_pos=(abs(delta_h), y_i, 0),
            initial_vel=(0, 7 * w / 4 * abs(delta_h), 0),
        )
        self.game_helper.node_helper.add_node(
            dv=delta_v,
            time=5,
            absolute=False,
            direction="prograde",
        )

        circ_burn_done = False
        t_0 = self.game_helper.stream_helper.ut()
        tau = 0
        self.game_helper.node_helper.rcs_node_execution()
        self.game_helper.space_center_helper.warp_time(
            warping_time=T_target / 4, absolute=False
        )
        while tau < T_target:
            tau = self.game_helper.stream_helper.ut() - t_0
            if tau <= T_target / 4:
                pass
            else:
                in_plane_nav, out_of_plane_nav = self.gnc_helper.navigation.output()
                logging.info(
                    f"Nav, {tau:.2f}, {in_plane_nav[0][0]:.2f}, {in_plane_nav[1][0]:.2f}, {out_of_plane_nav[0][0]:.2f}, {in_plane_nav[2][0]:.2f}, {in_plane_nav[3][0]:.2f}, {out_of_plane_nav[1][0]:.2f}"
                )
                if tau <= T_target / 2:
                    in_plane_ref, out_of_plane_ref = ref_signal(tau)
                elif not circ_burn_done:
                    self.game_helper.node_helper.add_node(
                        dv=delta_v,
                        time=5,
                        absolute=False,
                        direction="prograde",
                    )
                    self.game_helper.node_helper.rcs_node_execution()
                    circ_burn_done = True
                logging.info(
                    f"Guid, {in_plane_ref[0][0]:.2f}, {in_plane_ref[1][0]:.2f}, {in_plane_ref[2][0]:.2f}, {in_plane_ref[3][0]:.2f}"
                )

                u_plane = self.gnc_helper.in_plane_controller.control(
                    in_plane_nav, in_plane_ref
                )
                u_out_of_plane = self.gnc_helper.out_of_plane_controller.control(
                    out_of_plane_nav, out_of_plane_ref
                )
                u = (float(u_plane[0]), float(u_plane[1]), float(u_out_of_plane))
                u_body = self.game_helper.space_center_helper.transform_position(
                    vector=u,
                    from_frame=self.game_helper.chaser.orbital_reference_frame,
                    to_frame=self.game_helper.chaser.reference_frame,
                )
                self.game_helper.rcs_ctrl_helper.rcs_actuation(u_body)
                logging.info(
                    f"Control, {u[0]:.2f}, {u[1]:.2f}, {u[2]:.2f}, {u_body[0]:.2f}, {u_body[1]:.2f}, {u_body[2]:.2f}"
                )
                time.sleep(0.2)
        logging.info("===== Homing Phase finished =====")
