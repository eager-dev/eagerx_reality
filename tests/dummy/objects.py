import numpy as np
from eagerx.core.space import Space
from eagerx.core.entities import Object
from eagerx_reality.engine import RealEngine
from eagerx.core.specs import ObjectSpec
from eagerx.core.graph_engine import EngineGraph
import eagerx.core.register as register


class Dummy(Object):
    @classmethod
    @register.sensors(dummy_output=Space(low=np.array([-1], dtype="float32"), high=np.array([1], dtype="float32")))
    @register.actuators(dummyy_input=Space(low=np.array([-1], dtype="float32"), high=np.array([1], dtype="float32")))
    @register.engine_states(dummy_state=Space(low=np.array([-1], dtype="float32"), high=np.array([1], dtype="float32")))
    def make(cls, name: str, sensors=None, states=None, rate=30) -> ObjectSpec:
        """Object spec of dummy object"""
        spec = cls.get_specification()

        # Set parameters
        spec.config.update(name=name, rate=rate)
        spec.config.sensors = sensors if sensors else ["dummy_output"]
        spec.config.actuators = ["dummyy_input"]
        spec.config.states = states if states else ["dummy_state"]

        # Set rates
        spec.sensors.dummy_output.rate = rate
        spec.actuators.dummyy_input.rate = rate
        return spec

    @staticmethod
    @register.engine(RealEngine)  # This decorator pre-initializes engine implementation with default object_params
    def real_engine(spec: ObjectSpec, graph: EngineGraph):
        """Engine-specific implementation (RealEngine) of the object."""
        try:
            from tests.dummy.engine_states import DummyReset
            from tests.dummy.engine_nodes import DummyOutput
            from tests.dummy.engine_nodes import DummyInput
        except ImportError:
            try:
                from .dummy.engine_states import DummyReset
                from .dummy.engine_nodes import DummyOutput
                from .dummy.engine_nodes import DummyInput
            except ImportError:
                from dummy.engine_states import DummyReset
                from dummy.engine_nodes import DummyOutput
                from dummy.engine_nodes import DummyInput

        # Couple engine states
        spec.engine.states.dummy_state = DummyReset.make(sleep_time=1.0, repeat=1)

        # Create sensor engine nodes
        # Rate=None, because we will connect them to sensors (thus uses the rate set in the agnostic specification)
        obs = DummyOutput.make("dummy_output", rate=spec.sensors.dummy_output.rate, process=0)

        # Create actuator engine nodes
        # Rate=None, because we will connect it to an actuator (thus uses the rate set in the agnostic specification)
        action = DummyInput.make("dummyy_input", rate=spec.actuators.dummyy_input.rate, process=0)

        # Connect all engine nodes
        graph.add([obs, action])
        graph.connect(source=obs.outputs.dummy_output, sensor="dummy_output")
        graph.connect(actuator="dummyy_input", target=action.inputs.dummy_input)
