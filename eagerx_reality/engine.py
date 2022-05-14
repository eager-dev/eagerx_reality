# OTHER
from typing import Optional, Dict, Union, List


# ROS IMPORTS
import rospy
from std_msgs.msg import UInt64
from genpy.message import Message

# RX IMPORTS
from eagerx.core.constants import process, ERROR
import eagerx.core.register as register
from eagerx.utils.utils import Msg
from eagerx.core.entities import Engine
from eagerx.core.specs import EngineSpec


class RealEngine(Engine):
    @staticmethod
    @register.spec("RealEngine", Engine)
    def spec(
        spec: EngineSpec,
        rate,
        process: Optional[int] = process.NEW_PROCESS,
        sync: Optional[bool] = False,
        log_level: Optional[int] = ERROR,
    ):
        """
        Spec of the RealEngine

        :param spec: Not provided by the user.
        :param rate: Rate of the engine
        :param process: {0: NEW_PROCESS, 1: ENVIRONMENT, 2: ENGINE, 3: EXTERNAL}
        :param sync: Run reactive or async
        :param log_level: {0: SILENT, 10: DEBUG, 20: INFO, 30: WARN, 40: ERROR, 50: FATAL}
        :return: EngineSpec
        """
        # Modify default engine params
        spec.config.rate = rate
        spec.config.process = process
        spec.config.sync = sync
        spec.config.real_time_factor = 1
        spec.config.simulate_delays = False
        spec.config.log_level = log_level
        spec.config.color = "magenta"

    def initialize(self):
        self.simulator = dict()

    @register.engine_config(driver_launch_file=None, launch_args=[])
    def add_object(self, config, engine_config, node_params, state_params):
        # add object to simulator (we have a ref to the simulator with self.simulator)
        rospy.loginfo(f'Adding object "{config["name"]}" of type "{config["entity_id"]}" to the simulator.')

        # Extract relevant agnostic_params
        obj_name = config["name"]

        # Create new env, and add to simulator
        self.simulator[obj_name] = dict(state=None, input=None)

    def pre_reset(self, **kwargs: Optional[Msg]):
        pass

    @register.states()
    def reset(self, **kwargs: Optional[Msg]):
        pass

    @register.outputs(tick=UInt64)
    def callback(self, t_n: float, **kwargs: Dict[str, Union[List[Message], float, int]]):
        pass
