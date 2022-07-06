from typing import Any
import numpy as np
from gym.spaces import Box, Discrete

# IMPORT EAGERX
import eagerx
from eagerx.core.specs import NodeSpec, ObjectSpec
import eagerx.core.register as register
from eagerx.utils.utils import Msg

# random number generator
from random import random


class DummyOutput(eagerx.EngineNode):
    @classmethod
    def make(cls, name: str, rate: float, process: int = eagerx.NEW_PROCESS, color: str = 'cyan') -> NodeSpec:
        """DummyOutput spec"""
        spec = cls.get_specification()

        # Modify default node params
        spec.config.update(name=name, rate=rate, process=process, color=color, inputs=["tick"], outputs=["dummy_output"])
        return spec

    def initialize(self, spec: NodeSpec, object_spec: ObjectSpec, simulator: Any):
        pass

    @register.states()
    def reset(self):
        pass

    @register.inputs(tick=Discrete(9999))
    @register.outputs(dummy_output=Box(low=np.array([-1], dtype="float32"), high=np.array([1], dtype="float32")))
    def callback(self, t_n: float, tick: Msg):
        data = np.array([random()], dtype="float32")
        return dict(dummy_output=data)


class DummyInput(eagerx.EngineNode):
    @classmethod
    def make(cls, name: str, rate: float, process: int = eagerx.NEW_PROCESS, color: str = 'green') -> NodeSpec:
        """DummyInput spec"""
        spec = cls.get_specification()

        # Modify default node params
        spec.config.update(name=name, rate=rate, process=process, color=color, inputs=['tick', 'dummy_input'], outputs=[])
        return spec

    def initialize(self, spec: NodeSpec, object_spec: ObjectSpec, simulator: Any):
        pass

    @register.states()
    def reset(self):
        pass

    @register.inputs(tick=Discrete(9999),
                     dummy_input=Box(low=np.array([-1], dtype="float32"), high=np.array([1], dtype="float32")))
    @register.outputs(dummy_output=Box(low=np.array([-1], dtype="float32"), high=np.array([1], dtype="float32")))
    def callback(self, t_n: float, tick: Msg, dummy_input: Msg):
        data = np.array([random()], dtype="float32")
        return dict(dummy_output=data)
