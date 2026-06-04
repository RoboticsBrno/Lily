from __future__ import annotations

import time
from pathlib import Path
from typing import TYPE_CHECKING, Any, Optional, Protocol

from comm.controller import Controller
from comm.messages import ArmCommand, Measurements, MoveCommand
from comm.types import Transport
from geometry.transforms import Pose
from localization.stack import LocalizationStack
from params import INITIAL_POSE_THETA, INITIAL_POSE_X, INITIAL_POSE_Y

if TYPE_CHECKING:
    from sim.server import RobotSimulatorServer
    from util.keyboard_controller import KeyboardRobotController
    from util.visualizer import Visualizer


class TargetProgram(Protocol):
    @property
    def needs_keyboard(self) -> bool:
        return False

    def setup(
        self,
        controller: Controller,
        localization: LocalizationStack,
        keyboard: Optional[KeyboardRobotController],
        sim_server: Optional[RobotSimulatorServer],
        visualizer: Optional[Visualizer],
    ) -> None:
        ...

    def on_measurements(
        self,
        measurements: Measurements,
        controller: Controller,
        localization: LocalizationStack,
    ) -> None:
        ...

    def on_ui_tick(
        self,
        dt_seconds: float,
        visualizer: Visualizer,
        localization: LocalizationStack,
        sim_server: Optional[RobotSimulatorServer],
    ) -> None:
        ...

    def on_ui_event(
        self,
        event: Any,
        visualizer: Visualizer,
        keyboard: Optional[KeyboardRobotController],
        sim_server: Optional[RobotSimulatorServer],
    ) -> None:
        ...


def run(
    target_class: type[TargetProgram],
    transport: str,
    use_sim: bool,
    use_vis: bool,
    device: str = "/dev/ttyUSB0",
    host: str = "127.0.0.1",
    map_path: str = "data/map_bear_rescue.json",
    recording_path: str | None = None,
    speed: float = 1.0,
) -> None:
    from comm.serial_transport import SerialTransport
    from comm.udp_transport import UdpTransport
    from util.init_common import (
        CONTROLLER_RECEIVE_PORT,
        SIM_PORT,
        TARGET_FPS,
        build_controller,
        build_localization_stack,
        build_replay_controller,
        connect_keyboard_ctrl,
        make_default_sim,
        resolve_map_path,
    )
    from util.keyboard_controller import KeyboardRobotController

    repo_root = Path(__file__).resolve().parent.parent
    resolved_map = resolve_map_path(repo_root, map_path)

    sim_server = None
    if use_sim:
        sim_server, _ = make_default_sim(resolved_map)

    if transport == "serial":
        raw_transport: Transport = SerialTransport(device=device, baud_rate=921600)
        controller = build_controller(raw_transport, recording_path=recording_path)
    elif transport == "udp":
        raw_transport = UdpTransport(
            host=host, port=SIM_PORT, receive_port=CONTROLLER_RECEIVE_PORT
        )
        controller = build_controller(raw_transport, recording_path=recording_path)
    elif transport == "replay":
        controller = build_replay_controller(recording_path or "recording.csv", speed=speed)
    else:
        raise ValueError(f"Unknown transport: {transport}")

    if sim_server is not None:
        initial_pose = sim_server.robot.pose
    else:
        initial_pose = Pose(INITIAL_POSE_X, INITIAL_POSE_Y, INITIAL_POSE_THETA)
    localization = build_localization_stack(resolved_map, initial_pose)

    visualizer = None
    if use_vis:
        from util.visualizer import Visualizer

        visualizer = Visualizer(title=target_class.__name__)

    target = target_class()
    keyboard = None
    if target.needs_keyboard:
        keyboard = connect_keyboard_ctrl(controller)

    target.setup(controller, localization, keyboard, sim_server, visualizer)

    def on_measurements(measurements: Measurements):
        target.on_measurements(measurements, controller, localization)

    controller.set_measurement_callback(on_measurements)

    if visualizer is not None:
        from util.vis_common import draw_sim_truth

        def on_ui_tick(dt_seconds: float) -> None:
            if sim_server is not None:
                draw_sim_truth(visualizer, sim_server)
            target.on_ui_tick(dt_seconds, visualizer, localization, sim_server)

        def on_ui_event(event) -> None:
            target.on_ui_event(event, visualizer, keyboard, sim_server)

        visualizer.on_tick = on_ui_tick
        visualizer.on_event = on_ui_event

    if visualizer is not None:
        visualizer.init()
    if sim_server is not None:
        sim_server.start()
    controller.start()
    controller.send_command(ArmCommand())

    try:
        if visualizer is not None:
            visualizer.run(target_fps=max(1, TARGET_FPS))
        else:
            while True:
                time.sleep(1)
    finally:
        controller.send_command(MoveCommand(left_speed=0, right_speed=0))
        controller.stop()
        if sim_server is not None:
            sim_server.stop()

