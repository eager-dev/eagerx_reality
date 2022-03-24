import rospy
from eagerx import EngineState
import eagerx.core.register as register


class DummyReset(EngineState):
    @staticmethod
    @register.spec('DummyResetState', EngineState)
    def spec(spec, sleep_time: float = 1., repeat: int = 1):
        spec.initialize(DummyReset)

        spec.config.sleep_time = sleep_time
        spec.config.repeat = repeat

    def initialize(self, sleep_time: float, repeat: int):
        self.sleep_time = sleep_time
        self.repeat = repeat

    def reset(self, state, done):
        for i in range(self.repeat):
            rospy.sleep(self.sleep_time)
