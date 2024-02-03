from src.physics.orb_dyn_utils import OrbitalDynamicsUtils, CelestialBodyParameters
from src.helpers.stream_helper import StreamHelper
from src.helpers.space_center_helper import SpaceCenterHelper
from src.helpers.att_ctrl_helper import AttCtrlHelper
from src.helpers.rcs_ctrl_helper import RCSCtrlHelper
from src.helpers.node_helper import NodeHelper
from src.game_connector import KRPCConnector


class GameHelper:
    def __init__(
        self,
        chaser,
        target,
        orb_dyn_params: CelestialBodyParameters,
        orb_dyn: OrbitalDynamicsUtils,
        stream_helper: StreamHelper,
        node_helper: NodeHelper,
        space_center_helper: SpaceCenterHelper,
        att_ctrl_helper: AttCtrlHelper,
        rcs_ctrl_helper: RCSCtrlHelper,
    ):
        self.chaser = chaser
        self.target = target
        self.orb_dyn_params = orb_dyn_params
        self.orb_dyn = orb_dyn
        self.stream_helper = stream_helper
        self.node_helper = node_helper
        self.space_center_helper = space_center_helper
        self.att_ctrl_helper = att_ctrl_helper
        self.rcs_ctrl_helper = rcs_ctrl_helper


class GameHelperInit:
    @classmethod
    def game_helper_init(cls, connector: KRPCConnector) -> GameHelper:
        stream_helper = StreamHelper(
            conn=connector.conn, chaser=connector.chaser, target=connector.target
        )
        space_center_helper = SpaceCenterHelper(
            space_center=connector.space_center, stream=stream_helper
        )
        orb_dyn_params = CelestialBodyParameters(
            connector.chaser.orbit.body.gravitational_parameter,
            connector.chaser.orbit.body.surface_gravity,
            connector.chaser.orbit.body.equatorial_radius,
        )
        orb_dyn = OrbitalDynamicsUtils(celestial_body_params=orb_dyn_params)
        att_ctrl_helper = AttCtrlHelper(
            vessel=connector.chaser, space_center_helper=space_center_helper
        )
        rcs_ctrl_helper = RCSCtrlHelper(
            vessel=connector.chaser, space_center_helper=space_center_helper
        )
        node_helper = NodeHelper(
            vessel=connector.chaser,
            orb_dyn=orb_dyn,
            params=orb_dyn_params,
            stream_helper=stream_helper,
            space_center_helper=space_center_helper,
            att_ctrl_helper=att_ctrl_helper,
            rcs_ctrl_helper=rcs_ctrl_helper,
        )
        return GameHelper(
            chaser=connector.chaser,
            target=connector.target,
            orb_dyn_params=orb_dyn_params,
            orb_dyn=orb_dyn,
            stream_helper=stream_helper,
            node_helper=node_helper,
            space_center_helper=space_center_helper,
            att_ctrl_helper=att_ctrl_helper,
            rcs_ctrl_helper=rcs_ctrl_helper,
        )
