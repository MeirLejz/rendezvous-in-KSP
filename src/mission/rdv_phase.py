from abc import ABC, abstractmethod


class RDVPhase(ABC):
    def __init__(self) -> None:
        super().__init__()

    @abstractmethod
    def execute_phase():
        pass
