import json
from typing import Any, Dict, cast
from .messages import (
    Command,
    MoveCommand,
    ClawCommand,
    SubscribeCommand,
    LidarMeasurement,
    EncodersMeasurement,
    Measurements,
    ClawAction,
)


class JsonSerializer:
    @staticmethod
    def serialize_command(command: Command) -> bytes:
        if isinstance(command, MoveCommand):
            return json.dumps({
                "command": "move",
                "left_speed": command.left_speed,
                "right_speed": command.right_speed,
            }).encode("utf-8")
        elif isinstance(command, ClawCommand):
            return json.dumps({
                "command": "claw",
                "action": command.action.value,
            }).encode("utf-8")
        elif isinstance(command, SubscribeCommand):
            return json.dumps({
                "command": "subscribe",
            }).encode("utf-8")
        else:
            raise ValueError(f"Unknown command type: {type(command)}")

    @staticmethod
    def deserialize_command(data: bytes) -> Command:
        d = cast(Dict[str, Any], json.loads(data.decode("utf-8")))

        command_type = d.get("command")

        if command_type == "move":
            return MoveCommand(
                left_speed=d["left_speed"],
                right_speed=d["right_speed"],
            )
        elif command_type == "claw":
            return ClawCommand(
                action=ClawAction(d["action"]),
            )
        elif command_type == "subscribe":
            return SubscribeCommand()
        else:
            raise ValueError(f"Unknown command type: {command_type}")

    @staticmethod
    def serialize_measurements(measurements: Measurements) -> bytes:
        return json.dumps({
            "timestamp": measurements.timestamp,
            "lidar": [
                {
                    "angle": m.angle,
                    "distance": m.distance,
                }
                for m in measurements.lidar
            ],
            "encoders": {
                "left_ticks": measurements.encoders.left_ticks,
                "right_ticks": measurements.encoders.right_ticks,
            },
        }).encode("utf-8")

    @staticmethod
    def deserialize_measurements(data: bytes) -> Measurements:
        d = cast(Dict[str, Any], json.loads(data.decode("utf-8")))

        lidar = [
            LidarMeasurement(
                angle=m["angle"],
                distance=m["distance"],
            )
            for m in d["lidar"]
        ]

        encoders = EncodersMeasurement(
            left_ticks=d["encoders"]["left_ticks"],
            right_ticks=d["encoders"]["right_ticks"],
        )

        return Measurements(timestamp=d["timestamp"], lidar=lidar, encoders=encoders)
