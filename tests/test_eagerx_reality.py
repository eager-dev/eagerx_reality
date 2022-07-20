import eagerx

import pytest

NP = eagerx.NEW_PROCESS
ENV = eagerx.ENVIRONMENT


@pytest.mark.parametrize(
    "eps, steps, sync, p",
    [(3, 3, False, ENV)],
)
def test_real_engine(eps, steps, sync, p):
    # Start roscore
    eagerx.set_log_level(eagerx.WARN)

    # Define unique name for test environment
    name = f"{eps}_{steps}_{sync}_{p}"
    engine_p = p
    rate = 30

    # Initialize empty graph
    graph = eagerx.Graph.create()

    # Create dummy 
    try:
        from tests.dummy.objects import Dummy
    except ImportError:
        try:
            from .dummy.objects import Dummy
        except ImportError:
            from dummy.objects import Dummy
    dummy = Dummy.make("dummy", sensors=["dummy_output"], states=["dummy_state"])
    graph.add(dummy)

    # Connect the nodes
    graph.connect(action="action", target=dummy.actuators.dummy_input)
    graph.connect(source=dummy.sensors.dummy_output, observation="observation", window=1)

    # Define engines
    from eagerx_reality.engine import RealEngine
    engine = RealEngine.make(rate=rate, sync=sync, process=engine_p)

    # Define backend
    from eagerx.backends.single_process import SingleProcess
    backend = SingleProcess.make()

    # Define environment
    class TestEnv(eagerx.BaseEnv):
        def __init__(self, name, rate, graph, engine, backend, force_start):
            super().__init__(name, rate, graph, engine, backend=backend, force_start=force_start)

        def step(self, action):
            obs = self._step(action)
            # Determine done flag
            done = steps > 500
            # Set info:
            info = dict()
            return obs, 0., done, info

        def reset(self):
            states = self.state_space.sample()
            obs = self._reset(states)
            return obs

    # Initialize Environment
    env = TestEnv(name, rate, graph, engine, backend, force_start=True)

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
    print("\n[Shutdown]")


if __name__ == "__main__":
    test_real_engine(3, 3, False, ENV)
