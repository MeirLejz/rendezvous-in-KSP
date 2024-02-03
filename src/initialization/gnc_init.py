import logging
import numpy as np

from src.gnc.cw_linear_dynamics import (
    InPlaneDynamics,
    OutOfPlaneDynamics,
)
from src.gnc.guidance.guidance import SmoothGuidance, CWGuidance
from src.gnc.guidance.guidance_profiles import SmoothProfile, GuidanceParameters
from src.gnc.navigation import FullKnowledgeNavigation
from src.gnc.lqr_continuous_ctrl import LQRControl, LQRCost

from src.game_connector import KRPCConnector
from src.initialization.game_helper_init import GameHelper


class GNCHelper:
    def __init__(
        self,
        in_plane_controller: LQRControl,
        out_of_plane_controller: LQRControl,
        smooth_guidance: SmoothGuidance,
        cw_guidance: CWGuidance,
        navigation: FullKnowledgeNavigation,
    ):
        self.in_plane_controller = in_plane_controller
        self.out_of_plane_controller = out_of_plane_controller
        self.smooth_guidance = smooth_guidance
        self.cw_guidance = cw_guidance
        self.navigation = navigation


class GNCInit:
    """GNC initialization class."""

    @classmethod
    def init_gnc_classes(
        cls,
        game_helper: GameHelper,
        connector: KRPCConnector,
    ) -> GNCHelper:
        # Clohessy Wiltshire linearized dynamics
        n = game_helper.orb_dyn.orbital_rate(connector.target.orbit.semi_major_axis)
        logging.info(f"Target orbital rate: {n} rad/s")
        in_plane_dynamics = InPlaneDynamics(orbital_rate=n)
        out_of_plane_dynamics = OutOfPlaneDynamics(orbital_rate=n)

        # Continuous thrust controllers for closing phase
        lqr_cost = LQRCost(Q=10**3, R=10**5)
        in_plane_controller = LQRControl(costs=lqr_cost, dynamics=in_plane_dynamics)
        out_of_plane_controller = LQRControl(
            costs=lqr_cost, dynamics=out_of_plane_dynamics
        )
        print(f"in plane gain: {in_plane_controller.optimal_gain}")
        print(f"out of plane gain: {out_of_plane_controller.optimal_gain}")

        logging.info("===== Optimal control parameters for closed loop maneuvers =====")
        logging.info(f"state cost (Q): {lqr_cost.Q}")
        logging.info(f"control cost (R): {lqr_cost.R}")

        # Forced guidance for closing phase
        guidance_params = GuidanceParameters(
            norm_pos_pol_coeff=np.array([0, 0, 0, 10, -15, 6]),
            norm_vel_pol_coeff=np.array([0, 0, 30, -60, 30]),
            norm_acc_pol_coeff=np.array([0, 60, -180, 120]),
        )
        x = SmoothProfile(guid_params=guidance_params)
        y = SmoothProfile(guid_params=guidance_params)
        z = SmoothProfile(guid_params=guidance_params)
        smooth_guidance = SmoothGuidance(profiles=[x, y, z])
        cw_guidance = CWGuidance(orbital_rate=n)

        logging.info("===== Guidance parameters =====")
        logging.info(
            f"Position polynomial coefficients: {guidance_params.norm_pos_pol_coeff}"
        )
        logging.info(
            f"Velocity polynomial coefficients: {guidance_params.norm_vel_pol_coeff}"
        )

        # Navigation
        navigation = FullKnowledgeNavigation(stream_helper=game_helper.stream_helper)

        return GNCHelper(
            in_plane_controller=in_plane_controller,
            out_of_plane_controller=out_of_plane_controller,
            smooth_guidance=smooth_guidance,
            cw_guidance=cw_guidance,
            navigation=navigation,
        )
