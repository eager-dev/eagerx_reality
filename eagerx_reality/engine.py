# OTHER
from typing import Optional

# RX IMPORTS
import eagerx
import eagerx.core.register as register
from eagerx.core.entities import Engine
from eagerx.core.specs import EngineSpec


class RealEngine(Engine):
    @classmethod
    def make(
        cls,
        rate,
        process: Optional[int] = eagerx.NEW_PROCESS,
        sync: Optional[bool] = False,
        log_level: Optional[int] = eagerx.ERROR,
    ) -> EngineSpec:
        """
        Spec of the RealEngine

        :param rate: Rate of the engine
        :param process: {0: NEW_PROCESS, 1: ENVIRONMENT, 2: ENGINE, 3: EXTERNAL}
        :param sync: A boolean flag to set the mode of synchronization.
        :param log_level: {0: SILENT, 10: DEBUG, 20: INFO, 30: WARN, 40: ERROR, 50: FATAL}
        :return: EngineSpec
        """
        spec = cls.get_specification()

        # Modify default engine params
        spec.config.update(rate=rate, process=process, sync=sync, real_time_factor=1, simulate_delays=False)
        spec.config.update(log_level=log_level, color="magenta")
        return spec

    def initialize(self, spec: EngineSpec):
        pass

    def add_object(self, name: str):
        # add object to simulator (we have a ref to the simulator with self.simulator)
        self.backend.loginfo(f'Adding object "{name}" to the simulator.')

        # Nothing to be added to the simulator
        self.simulator[name].update()

    def pre_reset(self):
        pass

    @register.states()
    def reset(self):
        pass

    def callback(self, t_n: float):
        pass
