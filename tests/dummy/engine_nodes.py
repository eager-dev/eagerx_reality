from typing import Optional
import numpy as np
import rospy

# IMPORT ROS
from std_msgs.msg import UInt64, Float32MultiArray

# IMPORT EAGERX
import eagerx.core.register as register
from eagerx.utils.utils import Msg
from eagerx import EngineNode
from eagerx import process

# random number generator
from random import random

class DummyOutput(EngineNode):
    @staticmethod
    @register.spec('DummyOutput', EngineNode)
    def spec(spec, name: str, rate: float, process: Optional[int] = process.NEW_PROCESS, color: Optional[str] = 'cyan'):
        """DummyOutput spec"""
        # Modify default node params
        spec.config.name = name
        spec.config.rate = rate
        spec.config.process = process
        spec.config.color = color
        spec.config.inputs = ["tick"]
        spec.config.outputs = ["dummy_output"]

    def initialize(self):
        pass

    @register.states()
    def reset(self):
        pass

    @register.inputs(tick=UInt64)
    @register.outputs(dummy_output=Float32MultiArray)
    def callback(self, t_n: float, tick: Optional[Msg] = None):
        data = [random(), random()]
        return dict(dummy_output=Float32MultiArray(data=data))


class DummyInput(EngineNode):
    @staticmethod
    @register.spec('DummyInput', EngineNode)
    def spec(spec, name: str, rate: float, process: Optional[int] = process.NEW_PROCESS, color: Optional[str] = 'green'):
        """DummyInput spec"""
        # Modify default node params
        params = dict(name=name, rate=rate, process=process, color=color, inputs=['tick', 'dummy_input'], outputs=[])
        spec.config.update(params)

        # Set component parameter
        spec.inputs.dummy_input.window = 1

    def initialize(self):
        pass

    @register.states()
    def reset(self):
        pass

    @register.inputs(tick=UInt64, dummy_input=Float32MultiArray)
    def callback(self, t_n: float, tick: Optional[Msg] = None,
                 dummy_input: Optional[Msg] = None):
        pass
