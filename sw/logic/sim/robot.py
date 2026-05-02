from __future__ import annotations

from dataclasses import dataclass
from math import cos, sin
import math
from typing import Optional

from comm.messages import ClawCommand, Command, Measurements, MoveCommand
from geometry.shapes import Circle, Line, Point, ShapeGroup, Vector
from geometry.transforms import Pose, rotation, translation
from .sensors import (
    LidarSensorConfig,
    LidarSensorSimulator,
    MotorConfig,
    MotorSimulator,
)


@dataclass(frozen=True)
class RobotConfig:
    wheel_base: float
    diameter: float = 0.25
    claw_offset: Vector = Vector(0.08, 0.13)
    claw_length: float = 0.15
    claw_open_angle: float = 0.0
    claw_closed_angle: float = 0.25 * math.pi

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
        self.config = config
        self.lidar_sensor = LidarSensorSimulator(world, lidar_config, bear)
        self.motor = MotorSimulator(config=motor_config)
        self.pose = initial_pose

        self.claw_open = True

    def get_body_circle(self, pose: Optional[Pose] = None) -> Circle:
        pose_to_draw = self.pose if pose is None else pose
        return Circle(
            center=Point(pose_to_draw.x, pose_to_draw.y),
            radius=self.config.diameter * 0.5,
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

        left_hinge_xy = base_transform.apply_to_point(
            *left_claw_transform.apply_to_point(0.0, 0.0)
        )
        left_tip_xy = base_transform.apply_to_point(
            *left_claw_transform.apply_to_point(self.config.claw_length, 0.0)
        )

        right_hinge_xy = base_transform.apply_to_point(
            *right_claw_transform.apply_to_point(0.0, 0.0)
        )
        right_tip_xy = base_transform.apply_to_point(
            *right_claw_transform.apply_to_point(self.config.claw_length, 0.0)
        )

        left_hinge = Point(*left_hinge_xy)
        left_tip = Point(*left_tip_xy)
        right_hinge = Point(*right_hinge_xy)
        right_tip = Point(*right_tip_xy)

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

        self.pose.x += linear_speed * cos(self.pose.yaw) * dt
        self.pose.y += linear_speed * sin(self.pose.yaw) * dt
        self.pose.yaw += angular_speed * dt
        lidar = self.lidar_sensor.step(
            dt=dt,
            pose=self.pose,
            timestamp_ms=timestamp_ms,
        )
        return Measurements(timestamp=timestamp_ms, lidar=lidar, encoders=encoder)
