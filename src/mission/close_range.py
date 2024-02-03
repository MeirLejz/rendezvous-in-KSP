import time
import numpy as np
import logging

from src.initialization.game_helper_init import GameHelper
from src.initialization.gnc_init import GNCHelper

from src.mission.rdv_phase import RDVPhase


class CloseRangeManeuver(RDVPhase):
    def __init__(
        self,
        game_helper: GameHelper,
        gnc_helper: GNCHelper,
    ) -> None:
        self.game_helper = game_helper
        self.in_plane_control = gnc_helper.in_plane_controller
        self.out_of_plane_control = gnc_helper.out_of_plane_controller
        self.guidance = gnc_helper.smooth_guidance
        self.navigation = gnc_helper.navigation

        @property
        def final_state(self) -> tuple[float, float, float]:
            return self._final_state

        @final_state.setter
        def final_state(self, state: tuple[float, float, float]) -> None:
            self._final_state = state

        @property
        def duration(self) -> float:
            return self._duration

        @duration.setter
        def duration(self, duration: float) -> None:
            self._duration = duration

        @property
        def tolerance(self) -> float:
            return self._tolerance

        @tolerance.setter
        def tolerance(self, tolerance: float) -> None:
            self._tolerance = tolerance

    # def ref_signal(self, tau: float) -> np.ndarray:
    #     in_plane = np.array(
    #         [
    #             [self.ref[0].pos()(tau)],
    #             [self.ref[1].pos()(tau)],
    #             [self.ref[0].vel()(tau)],
    #             [self.ref[1].vel()(tau)],
    #         ]
    #     )
    #     out_of_plane = np.array(
    #         [
    #             [self.ref[2].pos()(tau)],
    #             [self.ref[2].vel()(tau)],
    #         ]
    #     )
    #     return in_plane, out_of_plane

    def norm_time(self, time: float, t_0: float) -> float:
        return (time - t_0) / self.duration

    def inside_tolerance(self, tau: float, tolerance: float = 3) -> bool:
        if tau < 1:
            return False
        else:
            return (
                abs(self.game_helper.stream_helper.rel_pos()[0] - self.final_state[0])
                < tolerance
                and abs(
                    self.game_helper.stream_helper.rel_pos()[1] - self.final_state[1]
                )
                < tolerance
                and abs(
                    self.game_helper.stream_helper.rel_pos()[2] - self.final_state[2]
                )
                < tolerance
            )

    def execute_phase(
        self,
        final_state: tuple,  # (x, y, z)
        duration: float = 60,  # s
        tolerance: float = 3,  # m
    ) -> None:
        self.final_state = final_state
        self.duration = duration
        self.tolerance = tolerance

        logging.info("===== Closed loop proximity maneuver phase =====")

        self.game_helper.rcs_ctrl_helper.enable_rcs()
        self.game_helper.att_ctrl_helper.enable_sas()
        self.game_helper.att_ctrl_helper.change_sas_mode("Target")
        self.game_helper.att_ctrl_helper.change_speed_mode("Target")

        t_0 = self.game_helper.stream_helper.ut()
        tod = t_0

        for i, profile in enumerate(self.guidance.profiles):
            profile.config_profile(
                p_i=self.game_helper.stream_helper.rel_pos()[i],
                p_f=final_state[i],
                duration=duration,
            )
        ref_signal = self.guidance.ref_signal()

        logging.info(
            f"Starting maneuver from state: {self.navigation.output()} [m, m/s] towards position: {final_state} [m]"
        )

        while tod < t_0 + 2 * self.duration:
            # time of day == current time
            tod = self.game_helper.stream_helper.ut()
            # navigation
            x, z = self.navigation.output()
            logging.info(
                f"Nav, {tod:.2f}, {x[0][0]:.2f}, {x[1][0]:.2f}, {z[0][0]:.2f}, {x[2][0]:.2f}, {x[3][0]:.2f}, {z[1][0]:.2f}"
            )
            # guidance
            tau = self.norm_time(time=tod, t_0=t_0)
            in_plane_ref, out_of_plane_ref = ref_signal(tau=min(tau, 1))
            logging.info(
                f"Guid, {min(tau, 1):.2f}, {in_plane_ref[0][0]:.2f}, {in_plane_ref[1][0]:.2f}, {out_of_plane_ref[0][0]:.2f}, {in_plane_ref[2][0]:.2f}, {in_plane_ref[3][0]:.2f}, {out_of_plane_ref[1][0]:.2f}"
            )
            if not self.inside_tolerance(tau=tau, tolerance=tolerance):
                # control
                U_LVLH = self.in_plane_control.control(state=x, ref=in_plane_ref)
                U_Z = self.out_of_plane_control.control(state=z, ref=out_of_plane_ref)
                U = (float(U_LVLH[0]), float(U_LVLH[1]), float(U_Z))
                # change of reference frame
                U_BODY = self.game_helper.space_center_helper.transform_position(
                    vector=U,
                    from_frame=self.game_helper.chaser.orbital_reference_frame,
                    to_frame=self.game_helper.chaser.reference_frame,
                )
                # actuation
                self.game_helper.rcs_ctrl_helper.rcs_actuation(U_BODY)
                logging.info(
                    f"Control, {U[0]:.2f}, {U[1]:.2f}, {U[2]:.2f}, {U_BODY[0]:.2f}, {U_BODY[1]:.2f}, {U_BODY[2]:.2f}"
                )
            else:
                logging.info("Inside tolerance, not controlling")
            time.sleep(0.1)

        logging.info("===== End of closed loop proximity maneuver phase =====")
