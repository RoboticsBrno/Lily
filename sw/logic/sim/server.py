from __future__ import annotations

import math
import threading
import time
from collections import deque
from pathlib import Path
from typing import Optional, Union

from comm.binary_serializer import BinarySerializer
from comm.messages import Command, SubscribeCommand
from comm.types import MessageCallback, Transport
from geometry.shapes import Circle, ShapeGroup
from geometry.transforms import Pose
from map.loader import load_world_from_json

from .robot import DifferentialDriveRobotSimulator, RobotConfig
from .sensors import LidarSensorConfig, MotorConfig


class RobotSimulatorServer:
    def __init__(
        self,
        world: ShapeGroup,
        robot: DifferentialDriveRobotSimulator,
        transport: Transport,
        publish_hz: float = 20.0,
    ):
        if publish_hz <= 0.0:
            raise ValueError("publish_hz must be positive")

        self.world = world
        self.robot = robot
        self.publish_hz = publish_hz

        self._serializer = BinarySerializer()
        self._transport = transport
        self._thread: Optional[threading.Thread] = None
        self._running = False
        self._subscribed = False
        self._pending_commands: deque[Command] = deque()
        self._lock = threading.Lock()

    def start(self) -> None:
        if self._running:
            return

        self._transport.connect()
        self._transport.start_receiving(_SimulatorCommandCallback(self))
        self._running = True
        self._thread = threading.Thread(target=self._serve_loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

        self._transport.close()

    def _serve_loop(self) -> None:
        dt = 1.0 / self.publish_hz
        next_publish = time.monotonic()

        while self._running:
            self._drain_pending_commands()

            now = time.monotonic()
            if now < next_publish:
                time.sleep(min(next_publish - now, 0.005))
                continue

            timestamp_ms = int(time.time() * 1000)
            measurements = self.robot.step(dt=dt, timestamp_ms=timestamp_ms)
            if self._subscribed:
                payload = self._serializer.serialize_measurements(measurements)
                self._transport.send(payload)
            next_publish += dt

    def _enqueue_command(self, command: Command) -> None:
        if isinstance(command, SubscribeCommand):
            self._subscribed = True
            return

        with self._lock:
            self._pending_commands.append(command)

    def _drain_pending_commands(self) -> None:
        with self._lock:
            while self._pending_commands:
                self.robot.handle_command(self._pending_commands.popleft())

    def get_true_pose(self) -> Pose:
        """Return a snapshot of the simulator ground-truth pose."""
        pose = self.robot.pose
        return Pose(pose.x, pose.y, pose.yaw)


def create_server_from_map(
    map_path: Union[str, Path],
    robot_config: RobotConfig,
    lidar_config: LidarSensorConfig,
    motor_config: MotorConfig,
    bear: Circle,
    transport: Transport,
    publish_hz: float = 20.0,
) -> RobotSimulatorServer:
    wall_world = load_world_from_json(map_path)
    display_world = ShapeGroup(shapes=[*wall_world.shapes, bear])
    config = robot_config
    robot = DifferentialDriveRobotSimulator(
        world=wall_world,
        config=config,
        lidar_config=lidar_config,
        motor_config=motor_config,
        initial_pose=Pose(0.2, 0.2, 0.5 * math.pi),
        bear=bear,
    )
    return RobotSimulatorServer(
        world=display_world,
        robot=robot,
        transport=transport,
        publish_hz=publish_hz,
    )


class _SimulatorCommandCallback(MessageCallback):
    def __init__(self, server: RobotSimulatorServer):
        self._server = server

    def on_message(self, data: bytes) -> None:
        try:
            command = self._server._serializer.deserialize_command(data)
        except Exception:
            return

        self._server._enqueue_command(command)

    def on_error(self, error: Exception) -> None:
        print(f"Simulator communication error: {error}")
