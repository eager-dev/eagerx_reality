# ROS packages required
from eagerx import Object, Bridge, Node, initialize, log, process

# Environment
from eagerx.core.env import EagerxEnv
from eagerx.core.graph import Graph
from eagerx.wrappers import Flatten

# Implementation specific
import eagerx.nodes  # Registers butterworth_filter # noqa # pylint: disable=unused-import
import eagerx_reality  # Registers RealBridge # noqa # pylint: disable=unused-import

# Other
import numpy as np
import rospy

# Import dummy object
import tests.dummy.objects # noqa # pylint: disable=unused-import

import pytest

NP = process.NEW_PROCESS
ENV = process.ENVIRONMENT

@pytest.mark.parametrize(
    "eps, steps, sync, p",
    [(3, 3, False, ENV)],
)
def test_real_bridge(eps, steps, sync, p):
    # Start roscore
    roscore = initialize("eagerx_core", anonymous=True, log_level=log.WARN)

    # Define unique name for test environment
    name = f"{eps}_{steps}_{sync}_{p}"
    bridge_p = p
    rate = 30

    # Initialize empty graph
    graph = Graph.create()

    # Create dummy
    dummy = Object.make(
        "Eagerx_Real_Dummy",
        "dummy",
        sensors=["dummy_output"],
        states=["dummy_state"],
    )
    graph.add(dummy)

    # Connect the nodes
    graph.connect(action="action", target=dummy.actuators.dummy_input)
    graph.connect(source=dummy.sensors.dummy_output, observation="observation", window=1)

    # Define bridges
    bridge = Bridge.make("RealBridge", rate=rate, sync=sync, process=bridge_p)

    # Define step function
    def step_fn(prev_obs, obs, action, steps):
        # Calculate dummy reward
        if len(obs["observation"][0]) == 2:
            cost = np.sum(obs["observation"][0])
        else:
            cost = 0
        # Determine done flag
        done = steps > 500
        # Set info:
        info = dict()
        return obs, -cost, done, info

    # Initialize Environment
    env = Flatten(EagerxEnv(name=name, rate=rate, graph=graph, bridge=bridge, step_fn=step_fn))

    # First reset
    env.reset()
    action = env.action_space.sample()
    for j in range(eps):
        print("\n[Episode %s]" % j)
        for i in range(steps):
            env.step(action)
        env.reset()
    print("\n[Finished]")
    env.shutdown()
    if roscore:
        roscore.shutdown()
    print("\n[Shutdown]")