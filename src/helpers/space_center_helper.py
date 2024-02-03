import logging
from src.helpers.stream_helper import StreamHelper


class SpaceCenterHelper:
    def __init__(self, space_center, stream: StreamHelper):
        self.space_center = space_center
        self.stream = stream

    def warp_time(self, warping_time: float, absolute: bool = False) -> None:
        starting_time = self.stream.ut()

        if absolute:
            final_time = warping_time
            duration = final_time - starting_time
        else:
            duration = warping_time
            final_time = starting_time + duration

        logging.info(f"Warping {duration} s. Warping to time {final_time} s.")

        self.space_center.warp_to(final_time)
        while self.stream.ut() < final_time:
            pass
        logging.info("Finished warping time")

    def warp_factor(self, warp_factor: int = 2) -> None:
        self.space_center.rails_warp_factor = warp_factor

    def speed_mode(self, mode: str) -> None:
        if mode == "Target":
            return self.space_center.SpeedMode.target
        elif mode == "Orbit":
            return self.space_center.SpeedMode.orbit
        elif mode == "Surface":
            return self.space_center.SpeedMode.surface

    def sas_mode(self, mode: str) -> None:
        if mode == "Stability Assist":
            return self.space_center.SASMode.stability_assist
        elif mode == "Maneuver":
            return self.space_center.SASMode.maneuver
        elif mode == "Target":
            return self.space_center.SASMode.target
        elif mode == "Anti-Target":
            return self.space_center.SASMode.anti_target
        elif mode == "Prograde":
            return self.space_center.SASMode.prograde
        elif mode == "Retrograde":
            return self.space_center.SASMode.retrograde
        elif mode == "Normal":
            return self.space_center.SASMode.normal
        elif mode == "Anti-Normal":
            return self.space_center.SASMode.anti_normal
        elif mode == "Radial":
            return self.space_center.SASMode.radial
        elif mode == "Anti-Radial":
            return self.space_center.SASMode.anti_radial

    def transform_position(self, vector: tuple, from_frame, to_frame) -> tuple:
        return self.space_center.transform_position(vector, from_frame, to_frame)
