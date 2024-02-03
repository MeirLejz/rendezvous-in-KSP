import logging
import krpc
import sys


class KRPCConnector:
    def __init__(self, mission_name="mission name"):
        logging.basicConfig(
            filename="rendezvous_docking.log",
            filemode="w",
            level=logging.DEBUG,
            format="%(levelname)s - %(message)s",
        )
        self.mission_name = mission_name
        logging.info(f"Log file created. Mission name: {mission_name}")

        self.conn = krpc.connect(name=self.mission_name)
        self.space_center = self.conn.space_center
        self.target = self.get_target_vessel()
        self.chaser = self.conn.space_center.active_vessel

    def get_target_vessel(self):
        target = self.conn.space_center.target_vessel
        if target is None:
            logging.critical("No target vessel found. Exiting.")
            sys.exit()
        else:
            logging.info("Valid target acquired.")
        return target
