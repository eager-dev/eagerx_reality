from typing import Any
from eagerx import EngineState
from eagerx.core.specs import EngineStateSpec, ObjectSpec
import time


class DummyReset(EngineState):
    @classmethod
    def make(cls, sleep_time: float = 1., repeat: int = 1) -> EngineStateSpec:
        spec = cls.get_specification()
        spec.config.sleep_time = sleep_time
        spec.config.repeat = repeat
        return spec

    def initialize(self, spec: EngineStateSpec, object_spec: ObjectSpec, simulator: Any):
        self.sleep_time = spec.config.sleep_time
        self.repeat = spec.config.repeat

    def reset(self, state):
        for i in range(self.repeat):
            time.sleep(self.sleep_time)
