from numpy import ndarray, array
from abc import ABC, abstractmethod
from src.helpers.stream_helper import StreamHelper


class Navigation(ABC):
    def __init__(self) -> None:
        super().__init__()

    @abstractmethod
    def output(self) -> ndarray:
        pass


class FullKnowledgeNavigation(Navigation):
    def __init__(self, stream_helper: StreamHelper) -> None:
        self.stream_helper = stream_helper

    def output(self) -> ndarray:
        in_plane_full_state = array(
            [
                [self.stream_helper.rel_pos()[0]],  # radial
                [self.stream_helper.rel_pos()[1]],  # prograde
                [self.stream_helper.rel_vel()[0]],  # radial vel
                [self.stream_helper.rel_vel()[1]],  # prograde vel
            ]
        )
        out_of_plane_full_state = array(
            [
                [self.stream_helper.rel_pos()[2]],
                [self.stream_helper.rel_vel()[2]],
            ]
        )
        return in_plane_full_state, out_of_plane_full_state
