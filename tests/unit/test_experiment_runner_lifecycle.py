from argparse import Namespace
from pathlib import Path
import subprocess
import sys

import pytest

import simulator.experiment_runner as experiment_runner
from simulator.simulation import ParkingLot
from simulator.simulation.scenarios.parking import ParkingScenario


def _runner_args() -> Namespace:
    return Namespace(
        max_steps=None,
        scenario_artifact=[],
        scenario_suite="smoke",
        host="127.0.0.1",
        port=0,
        connect_host=None,
        wait_for_external_stack=False,
        adas_stack="test",
        adas_stack_catalog=Path("unused.json"),
        accept_timeout=0.01,
        socket_timeout=1.0,
        slots=None,
        all_empty_slots=False,
        log_interval=0,
        render=False,
        render_every=1,
        save_render_dir=None,
        trace_dir=None,
    )


class _FakeProcess:
    def __init__(self) -> None:
        self.returncode = None
        self.terminated = False
        self.killed = False

    def poll(self):
        return self.returncode

    def wait(self, timeout=None):
        if timeout in (5.0, 2.0):
            raise subprocess.TimeoutExpired("fake-stack", timeout)
        self.returncode = -9 if self.killed else 0
        return self.returncode

    def terminate(self) -> None:
        self.terminated = True

    def kill(self) -> None:
        self.killed = True


class _GracefulProcess:
    def __init__(self) -> None:
        self.returncode = None
        self.wait_timeouts = []
        self.terminated = False
        self.killed = False

    def poll(self):
        return self.returncode

    def wait(self, timeout=None):
        self.wait_timeouts.append(timeout)
        self.returncode = 0
        return self.returncode

    def terminate(self) -> None:
        self.terminated = True

    def kill(self) -> None:
        self.killed = True


class _FakeSocket:
    def __init__(self) -> None:
        self.closed = False

    def settimeout(self, _timeout) -> None:
        pass

    def close(self) -> None:
        self.closed = True


class _FakeStream:
    def __init__(self, fail_close=False) -> None:
        self.fail_close = fail_close
        self.closed = False

    def close(self) -> None:
        self.closed = True
        if self.fail_close:
            raise OSError("stream close failed")


class _FakeMakefileSocket(_FakeSocket):
    def __init__(self, streams, fail_second_makefile=False) -> None:
        super().__init__()
        self.streams = iter(streams)
        self.fail_second_makefile = fail_second_makefile
        self.makefile_calls = 0

    def makefile(self, *_args, **_kwargs):
        self.makefile_calls += 1
        if self.fail_second_makefile and self.makefile_calls == 2:
            raise OSError("writer setup failed")
        return next(self.streams)


class _FakeServer:
    def __init__(self, client_socket=None, accept_error=None) -> None:
        self.client_socket = client_socket
        self.accept_error = accept_error
        self.closed = False

    def __enter__(self):
        return self

    def __exit__(self, *_exc_info) -> None:
        self.closed = True

    def getsockname(self):
        return "127.0.0.1", 12345

    def settimeout(self, _timeout) -> None:
        pass

    def accept(self):
        if self.accept_error is not None:
            raise self.accept_error
        return self.client_socket, ("127.0.0.1", 54321)


class _FakeConnection:
    def __init__(self, sock, responses) -> None:
        self.sock = sock
        self.responses = iter(responses)
        self.closed = False

    def send(self, _payload) -> None:
        pass

    def receive(self):
        response = next(self.responses)
        if isinstance(response, BaseException):
            raise response
        return response

    def close(self) -> None:
        self.closed = True
        self.sock.close()


def _remote_slot_config() -> dict:
    return {
        "parking_lot": {
            "rows": 6,
            "cols": 1,
            "layout": "single_side",
            "origin_x": 10.0,
            "origin_y": 0.0,
            "slot_width": 2.5,
            "slot_length": 5.0,
            "gap_between_slots": 0.0,
            "aisle_width": 6.0,
            "slot_orientation_deg": 90.0,
        },
        "simulation": {
            "dt": 0.05,
            "max_episode_steps": 1,
            "goal_heading_tolerance": 0.1,
        },
    }


def _scenario() -> ParkingScenario:
    lot = ParkingLot(_remote_slot_config()["parking_lot"])
    return ParkingScenario(
        scenario_id="lifecycle",
        lot=lot,
        goal_slot_index=0,
        parked_cars=(),
    )


def test_json_connection_close_attempts_all_resources_after_stream_error() -> None:
    reader = _FakeStream(fail_close=True)
    writer = _FakeStream()
    sock = _FakeMakefileSocket([reader, writer])
    connection = experiment_runner.JsonLineConnection(sock)

    with pytest.raises(OSError, match="stream close failed"):
        connection.close()

    assert reader.closed
    assert writer.closed
    assert sock.closed


def test_json_connection_setup_failure_closes_partial_resources() -> None:
    reader = _FakeStream()
    sock = _FakeMakefileSocket([reader], fail_second_makefile=True)

    with pytest.raises(OSError, match="writer setup failed"):
        experiment_runner.JsonLineConnection(sock)

    assert reader.closed
    assert sock.closed


class _FakeSimulation:
    def __init__(self, *, fail_step=False, fail_reset=False, fail_close=False) -> None:
        self.fail_step = fail_step
        self.fail_reset = fail_reset
        self.fail_close = fail_close
        self.closed = False
        self.close_calls = 0
        self.render_enabled = True
        self.world_min_x = -2.0
        self.world_min_y = -2.0
        self.trajectory = []
        self.step_count = 0
        self.planned_path = []

    def reset(self):
        if self.fail_reset:
            raise RuntimeError("simulated reset failure")
        return {"position": [0.0, 0.0], "heading": 0.0, "velocity": [0.0, 0.0]}

    def get_goal_pose(self):
        return 1.0, 2.0, 0.0

    def get_static_obstacles(self):
        return []

    def get_dynamic_obstacles(self):
        return []

    def step(self, _acceleration, _steering):
        if self.fail_step:
            raise RuntimeError("simulated step failure")
        self.step_count += 1
        return (
            {"position": [0.0, 0.0], "heading": 0.0, "velocity": [0.0, 0.0]},
            {
                "done": True,
                "goal_reached": True,
                "collision_occurred": False,
                "stuck": False,
                "position_error": 0.0,
                "heading_error": 0.0,
            },
        )

    def close(self) -> None:
        self.close_calls += 1
        self.closed = True
        if self.fail_close:
            raise OSError("simulation close failed")


class _FakeVisualizer:
    def __init__(self, _config, *, fail_close=False) -> None:
        self.fail_close = fail_close
        self.closed = False
        self.close_calls = 0
        self.fig = None

    def render(self, **_kwargs) -> None:
        pass

    def close(self) -> None:
        self.close_calls += 1
        self.closed = True
        if self.fail_close:
            raise OSError("visualizer close failed")


def _run_slot(connection, trace_dir, monkeypatch, *, fail_step=False):
    sim = _FakeSimulation(fail_step=fail_step)
    visualizer = _FakeVisualizer(None)
    traces = []
    real_trace_logger = experiment_runner.ExperimentTraceLogger

    def create_trace(*args):
        trace = real_trace_logger(*args)
        traces.append(trace)
        original_close = trace.close
        trace.close_calls = 0

        def close_trace():
            trace.close_calls += 1
            original_close()

        trace.close = close_trace
        return trace

    monkeypatch.setattr(
        experiment_runner,
        "ParkingSimulationEnvironment",
        lambda **_kwargs: sim,
    )
    monkeypatch.setattr(
        experiment_runner,
        "SimulationVisualizer",
        lambda _config: visualizer,
    )
    monkeypatch.setattr(experiment_runner, "ExperimentTraceLogger", create_trace)
    result = experiment_runner.run_remote_slot(
        connection=connection,
        config=_remote_slot_config(),
        scenario=_scenario(),
        log_interval=0,
        live_render=True,
        render_every=1,
        save_render_dir=None,
        trace_dir=trace_dir,
    )
    return result, sim, visualizer, traces


def test_accept_timeout_stops_launched_stack(monkeypatch) -> None:
    server = _FakeServer(accept_error=TimeoutError("accept timed out"))
    process = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(60)"])
    monkeypatch.setattr(experiment_runner.socket, "create_server", lambda *_a, **_k: server)
    monkeypatch.setattr(experiment_runner, "get_adas_stack", lambda *_a: object())
    monkeypatch.setattr(experiment_runner, "launch_adas_stack", lambda *_a: process)

    with pytest.raises(TimeoutError, match="accept timed out"):
        experiment_runner.run_experiment(_runner_args())

    assert server.closed
    assert process.poll() is not None


def test_stack_process_is_allowed_to_exit_gracefully() -> None:
    process = _GracefulProcess()

    experiment_runner._stop_stack_process(process)

    assert process.wait_timeouts == [5.0]
    assert not process.terminated
    assert not process.killed


def test_handshake_failure_closes_socket_and_stops_stack(monkeypatch) -> None:
    server_socket = _FakeSocket()
    server = _FakeServer(client_socket=server_socket)
    process = _FakeProcess()
    connection = _FakeConnection(server_socket, [RuntimeError("handshake failed")])
    monkeypatch.setattr(experiment_runner.socket, "create_server", lambda *_a, **_k: server)
    monkeypatch.setattr(experiment_runner, "get_adas_stack", lambda *_a: object())
    monkeypatch.setattr(experiment_runner, "launch_adas_stack", lambda *_a: process)
    monkeypatch.setattr(
        experiment_runner,
        "JsonLineConnection",
        lambda _sock: connection,
    )

    with pytest.raises(RuntimeError, match="handshake failed"):
        experiment_runner.run_experiment(_runner_args())

    assert server.closed
    assert connection.closed
    assert server_socket.closed
    assert process.poll() is not None


@pytest.mark.parametrize(
    ("responses", "fail_step", "expected_error"),
    [
        ([ConnectionError("handshake disconnected")], False, "handshake disconnected"),
        (
            [{"type": "ready"}, ConnectionError("episode protocol failed")],
            False,
            "episode protocol failed",
        ),
        (
            [
                {"type": "ready"},
                {
                    "type": "command",
                    "longitudinal_acceleration_mps2": 0.0,
                    "front_mid_axle_steering_angle_rad": 0.0,
                },
            ],
            True,
            "simulated step failure",
        ),
        (
            [
                {"type": "ready"},
                {
                    "type": "command",
                    "longitudinal_acceleration_mps2": 0.0,
                    "front_mid_axle_steering_angle_rad": 0.0,
                },
                ConnectionError("result acknowledgement failed"),
            ],
            False,
            "result acknowledgement failed",
        ),
    ],
)
def test_remote_slot_failure_closes_simulator_visualizer_and_trace(
    tmp_path, monkeypatch, responses, fail_step, expected_error
) -> None:
    socket = _FakeSocket()
    connection = _FakeConnection(socket, responses)
    sim = _FakeSimulation(fail_step=fail_step)
    visualizer = _FakeVisualizer(None)
    traces = []
    real_trace_logger = experiment_runner.ExperimentTraceLogger

    def create_trace(*args):
        trace = real_trace_logger(*args)
        traces.append(trace)
        original_close = trace.close
        trace.close_calls = 0

        def close_trace():
            trace.close_calls += 1
            original_close()

        trace.close = close_trace
        return trace

    monkeypatch.setattr(
        experiment_runner,
        "ParkingSimulationEnvironment",
        lambda **_kwargs: sim,
    )
    monkeypatch.setattr(
        experiment_runner,
        "SimulationVisualizer",
        lambda _config: visualizer,
    )
    monkeypatch.setattr(experiment_runner, "ExperimentTraceLogger", create_trace)

    with pytest.raises((ConnectionError, RuntimeError), match=expected_error):
        experiment_runner.run_remote_slot(
            connection=connection,
            config=_remote_slot_config(),
            scenario=_scenario(),
            log_interval=0,
            live_render=True,
            render_every=1,
            save_render_dir=None,
            trace_dir=tmp_path,
        )

    assert sim.closed
    assert sim.close_calls == 1
    assert visualizer.closed
    assert visualizer.close_calls == 1
    assert len(traces) == 1
    assert traces[0]._file.closed
    assert traces[0].close_calls == 1


def test_remote_slot_success_preserves_result_and_closes_resources(tmp_path, monkeypatch) -> None:
    connection = _FakeConnection(
        _FakeSocket(),
        [
            {"type": "ready", "planned_path": []},
            {
                "type": "command",
                "longitudinal_acceleration_mps2": 0.0,
                "front_mid_axle_steering_angle_rad": 0.0,
            },
            {"type": "ack"},
        ],
    )

    result, sim, visualizer, traces = _run_slot(
        connection,
        tmp_path,
        monkeypatch,
    )

    assert result["success"]
    assert result["steps"] == 1
    assert result["trace_path"] == str(traces[0].path)
    assert sim.closed
    assert sim.close_calls == 1
    assert visualizer.closed
    assert visualizer.close_calls == 1
    assert traces[0]._file.closed
    assert traces[0].close_calls == 1


def test_reset_failure_closes_partially_initialized_simulator(monkeypatch) -> None:
    sim = _FakeSimulation(fail_reset=True)
    monkeypatch.setattr(
        experiment_runner,
        "ParkingSimulationEnvironment",
        lambda **_kwargs: sim,
    )

    with pytest.raises(RuntimeError, match="simulated reset failure"):
        experiment_runner.run_remote_slot(
            connection=_FakeConnection(_FakeSocket(), []),
            config=_remote_slot_config(),
            scenario=_scenario(),
            log_interval=0,
            live_render=True,
            render_every=1,
            save_render_dir=None,
            trace_dir=None,
        )

    assert sim.closed
    assert sim.close_calls == 1


def test_visualizer_initialization_failure_closes_simulator(monkeypatch) -> None:
    sim = _FakeSimulation()

    def fail_visualizer(_config):
        raise RuntimeError("visualizer initialization failed")

    monkeypatch.setattr(
        experiment_runner,
        "ParkingSimulationEnvironment",
        lambda **_kwargs: sim,
    )
    monkeypatch.setattr(experiment_runner, "SimulationVisualizer", fail_visualizer)

    with pytest.raises(RuntimeError, match="visualizer initialization failed"):
        experiment_runner.run_remote_slot(
            connection=_FakeConnection(_FakeSocket(), []),
            config=_remote_slot_config(),
            scenario=_scenario(),
            log_interval=0,
            live_render=True,
            render_every=1,
            save_render_dir=None,
            trace_dir=None,
        )

    assert sim.closed
    assert sim.close_calls == 1


def test_trace_initialization_failure_closes_prior_resources(tmp_path, monkeypatch) -> None:
    sim = _FakeSimulation()
    visualizer = _FakeVisualizer(None)

    def fail_trace(*_args):
        raise OSError("trace initialization failed")

    monkeypatch.setattr(
        experiment_runner,
        "ParkingSimulationEnvironment",
        lambda **_kwargs: sim,
    )
    monkeypatch.setattr(
        experiment_runner,
        "SimulationVisualizer",
        lambda _config: visualizer,
    )
    monkeypatch.setattr(experiment_runner, "ExperimentTraceLogger", fail_trace)

    with pytest.raises(OSError, match="trace initialization failed"):
        experiment_runner.run_remote_slot(
            connection=_FakeConnection(_FakeSocket(), []),
            config=_remote_slot_config(),
            scenario=_scenario(),
            log_interval=0,
            live_render=True,
            render_every=1,
            save_render_dir=None,
            trace_dir=tmp_path,
        )

    assert sim.closed and sim.close_calls == 1
    assert visualizer.closed and visualizer.close_calls == 1


def test_primary_execution_error_survives_cleanup_errors(tmp_path, monkeypatch) -> None:
    connection = _FakeConnection(
        _FakeSocket(),
        [
            {"type": "ready"},
            {
                "type": "command",
                "longitudinal_acceleration_mps2": 0.0,
                "front_mid_axle_steering_angle_rad": 0.0,
            },
        ],
    )
    sim = _FakeSimulation(fail_step=True, fail_close=True)
    visualizer = _FakeVisualizer(None, fail_close=True)
    traces = []
    real_trace_logger = experiment_runner.ExperimentTraceLogger

    def create_trace(*args):
        trace = real_trace_logger(*args)
        traces.append(trace)
        original_close = trace.close
        trace.close_calls = 0

        def close_trace():
            trace.close_calls += 1
            original_close()

        trace.close = close_trace
        return trace

    monkeypatch.setattr(
        experiment_runner,
        "ParkingSimulationEnvironment",
        lambda **_kwargs: sim,
    )
    monkeypatch.setattr(
        experiment_runner,
        "SimulationVisualizer",
        lambda _config: visualizer,
    )
    monkeypatch.setattr(experiment_runner, "ExperimentTraceLogger", create_trace)

    with pytest.raises(RuntimeError, match="simulated step failure") as error:
        experiment_runner.run_remote_slot(
            connection=connection,
            config=_remote_slot_config(),
            scenario=_scenario(),
            log_interval=0,
            live_render=True,
            render_every=1,
            save_render_dir=None,
            trace_dir=tmp_path,
        )

    assert isinstance(error.value.__cause__, OSError)
    assert "simulation close failed" in str(error.value.__cause__)
    assert sim.closed and sim.close_calls == 1
    assert visualizer.closed and visualizer.close_calls == 1
    assert traces[0]._file.closed and traces[0].close_calls == 1


def test_cleanup_failure_is_reported_after_all_resources_are_closed(tmp_path, monkeypatch) -> None:
    connection = _FakeConnection(
        _FakeSocket(),
        [
            {"type": "ready"},
            {
                "type": "command",
                "longitudinal_acceleration_mps2": 0.0,
                "front_mid_axle_steering_angle_rad": 0.0,
            },
            {"type": "ack"},
        ],
    )
    sim = _FakeSimulation()
    visualizer = _FakeVisualizer(None, fail_close=True)
    traces = []
    real_trace_logger = experiment_runner.ExperimentTraceLogger

    def create_trace(*args):
        trace = real_trace_logger(*args)
        traces.append(trace)
        original_close = trace.close
        trace.close_calls = 0

        def close_trace():
            trace.close_calls += 1
            original_close()

        trace.close = close_trace
        return trace

    monkeypatch.setattr(
        experiment_runner,
        "ParkingSimulationEnvironment",
        lambda **_kwargs: sim,
    )
    monkeypatch.setattr(
        experiment_runner,
        "SimulationVisualizer",
        lambda _config: visualizer,
    )
    monkeypatch.setattr(experiment_runner, "ExperimentTraceLogger", create_trace)

    with pytest.raises(OSError, match="visualizer close failed"):
        experiment_runner.run_remote_slot(
            connection=connection,
            config=_remote_slot_config(),
            scenario=_scenario(),
            log_interval=0,
            live_render=True,
            render_every=1,
            save_render_dir=None,
            trace_dir=tmp_path,
        )

    assert sim.closed and sim.close_calls == 1
    assert visualizer.closed and visualizer.close_calls == 1
    assert traces[0]._file.closed and traces[0].close_calls == 1
