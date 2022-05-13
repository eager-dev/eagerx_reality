# ROS IMPORTS
from std_msgs.msg import Float32MultiArray

# EAGERx IMPORTS
from eagerx_reality.bridge import RealBridge
from eagerx.core.entities import (
    Object,
    EngineNode,
    SpaceConverter,
    EngineState,
)
from eagerx.core.specs import ObjectSpec
from eagerx.core.graph_engine import EngineGraph
import eagerx.core.register as register


class Dummy(Object):
    entity_id = "Eagerx_Real_Dummy"

    @staticmethod
    @register.sensors(dummy_output=Float32MultiArray)
    @register.actuators(dummy_input=Float32MultiArray)
    @register.engine_states(dummy_state=Float32MultiArray)
    @register.config()
    def agnostic(spec: ObjectSpec, rate):
        """Agnostic definition of the dummy object"""
        # Register standard converters, space_converters, and processors
        import eagerx.converters  # noqa # pylint: disable=unused-import

        # Set observation properties: (space_converters, rate, etc...)
        spec.sensors.dummy_output.rate = rate
        spec.sensors.dummy_output.space_converter = SpaceConverter.make(
            "Space_Float32MultiArray", low=[-1, -1], high=[1, 1], dtype="float32"
        )

        # Set actuator properties: (space_converters, rate, etc...)
        spec.actuators.dummy_input.rate = rate
        spec.actuators.dummy_input.window = 1
        spec.actuators.dummy_input.space_converter = SpaceConverter.make(
            "Space_Float32MultiArray", low=[-1], high=[1], dtype="float32"
        )

        # Set model_state properties: (space_converters)
        spec.states.dummy_state.space_converter = SpaceConverter.make(
            "Space_Float32MultiArray", low=[-1, -1], high=[1, 1], dtype="float32"
        )

    @staticmethod
    @register.spec(entity_id, Object)
    def spec(spec: ObjectSpec, name: str, sensors=None, states=None, rate=30):
        """Object spec of dummy object"""
        # Modify default agnostic params
        # Only allow changes to the agnostic params (rates, windows, (space)converters, etc...
        spec.config.name = name
        spec.config.sensors = sensors if sensors else ["dummy_output"]
        spec.config.actuators = ["dummy_input"]
        spec.config.states = states if states else ["dummy_state"]

        # Add bridge implementation
        Dummy.agnostic(spec, rate)

    @staticmethod
    @register.bridge(entity_id, RealBridge)  # This decorator pre-initializes bridge implementation with default object_params
    def real_bridge(spec: ObjectSpec, graph: EngineGraph):
        """Engine-specific implementation (RealBridge) of the object."""
        # Couple engine states
        from tests.dummy.engine_states import DummyReset # noqa # pylint: disable=unused-import
        spec.RealBridge.states.dummy_state = EngineState.make("DummyResetState", sleep_time=1.0, repeat=1)

        # Create sensor engine nodes
        # Rate=None, because we will connect them to sensors (thus uses the rate set in the agnostic specification)
        from tests.dummy.engine_nodes import DummyOutput  # noqa # pylint: disable=unused-import
        obs = EngineNode.make("DummyOutput", "dummy_output", rate=spec.sensors.dummy_output.rate, process=0)

        # Create actuator engine nodes
        # Rate=None, because we will connect it to an actuator (thus uses the rate set in the agnostic specification)
        from tests.dummy.engine_nodes import DummyInput  # noqa # pylint: disable=unused-import
        action = EngineNode.make("DummyInput", "dummy_input", rate=spec.actuators.dummy_input.rate, process=0)

        # Connect all engine nodes
        graph.add([obs, action])
        graph.connect(source=obs.outputs.dummy_output, sensor="dummy_output")
        graph.connect(actuator="dummy_input", target=action.inputs.dummy_input)
