from __future__ import annotations

from dataclasses import dataclass
from math import cos, sin
from typing import Optional

from comm.messages import ClawCommand, Command, Measurements, MoveCommand
from geometry.shapes import Circle, Line, Point, ShapeGroup, Vector
from geometry.transforms import Pose, rotation, translation
from geometry.util import find_nearest
from .sensors import (
    LidarSensorConfig,
    LidarSensorSimulator,
    MotorConfig,
    MotorSimulator,
)


@dataclass(frozen=True)
class RobotConfig:
    wheel_base: float
    radius: float
    claw_offset: Vector
    claw_length: float
    claw_open_angle: float
    claw_closed_angle: float

    def __post_init__(self) -> None:
        if self.wheel_base <= 0.0:
            raise ValueError("wheel_base must be positive")


class DifferentialDriveRobotSimulator:
    def __init__(
        self,
        world: ShapeGroup,
        config: RobotConfig,
        lidar_config: LidarSensorConfig,
        motor_config: MotorConfig,
        initial_pose: Pose,
        bear: Circle,
    ):
        self.world = world
        self.config = config
        self.lidar_sensor = LidarSensorSimulator(world, lidar_config, bear)
        self.motor = MotorSimulator(config=motor_config)
        self.pose = initial_pose

        self.claw_open = True

    def get_body_circle(self, pose: Optional[Pose] = None) -> Circle:
        pose_to_draw = self.pose if pose is None else pose
        return Circle(
            center=Point(pose_to_draw.x, pose_to_draw.y),
            radius=self.config.radius,
        )

    def get_claw_segments(self) -> tuple[Line, Line]:
        claw_angle = (
            self.config.claw_open_angle
            if self.claw_open
            else self.config.claw_closed_angle
        )

        base_transform = self.pose.to_transform()

        left_claw_transform = (
            translation(self.config.claw_offset.x, self.config.claw_offset.y)
            .compose(rotation(-claw_angle))
        )
        right_claw_transform = (
            translation(self.config.claw_offset.x, -self.config.claw_offset.y)
            .compose(rotation(claw_angle))
        )

        left_hinge = Point(0.0, 0.0).apply_transform(base_transform.compose(left_claw_transform))
        left_tip = Point(self.config.claw_length, 0.0).apply_transform(base_transform.compose(left_claw_transform))
        right_hinge = Point(0.0, 0.0).apply_transform(base_transform.compose(right_claw_transform))
        right_tip = Point(self.config.claw_length, 0.0).apply_transform(base_transform.compose(right_claw_transform))

        return (
            Line(a=left_hinge, b=left_tip),
            Line(a=right_hinge, b=right_tip),
        )

    def handle_command(self, command: Command) -> None:
        if isinstance(command, MoveCommand):
            self.motor.set_motor_speed(
                left_speed=command.left_speed,
                right_speed=command.right_speed,
            )
        elif isinstance(command, ClawCommand):
            self.claw_open = command.action.value == "open"

    def step(self, dt: float, timestamp_ms: int) -> Measurements:
        if dt < 0.0:
            raise ValueError("dt must be non-negative")

        left_speed, right_speed, encoder = self.motor.step(
            dt=dt,
            timestamp_ms=timestamp_ms,
        )

        linear_speed = 0.5 * (left_speed + right_speed)
        angular_speed = (right_speed - left_speed) / self.config.wheel_base

        new_x = self.pose.x + linear_speed * cos(self.pose.yaw) * dt
        new_y = self.pose.y + linear_speed * sin(self.pose.yaw) * dt

        if self.world.shapes:
            _, nearest_dist = find_nearest(self.world, Point(new_x, new_y))
            if nearest_dist >= self.config.radius:
                self.pose.x = new_x
                self.pose.y = new_y
        else:
            self.pose.x = new_x
            self.pose.y = new_y

        self.pose.yaw += angular_speed * dt
        lidar = self.lidar_sensor.step(
            dt=dt,
            pose=self.pose,
            timestamp_ms=timestamp_ms,
        )
        return Measurements(timestamp=timestamp_ms, lidar=lidar, encoders=encoder)
