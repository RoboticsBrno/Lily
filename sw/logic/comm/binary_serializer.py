from __future__ import annotations

import struct

from .messages import (
    ClawAction,
    ClawCommand,
    Command,
    EncodersMeasurement,
    LidarMeasurement,
    Measurements,
    MoveCommand,
    SubscribeCommand,
)


class BinarySerializer:
    _COMMAND_MOVE = 1
    _COMMAND_CLAW = 2
    _COMMAND_SUBSCRIBE = 3

    @staticmethod
    def serialize_command(command: Command) -> bytes:
        if isinstance(command, MoveCommand):
            payload = struct.pack("<Bff", BinarySerializer._COMMAND_MOVE, command.left_power, command.right_power)
            return payload

        if isinstance(command, ClawCommand):
            return struct.pack(
                "<BB",
                BinarySerializer._COMMAND_CLAW,
                1 if command.action == ClawAction.OPEN else 0,
            )

        if isinstance(command, SubscribeCommand):
            return struct.pack("<B", BinarySerializer._COMMAND_SUBSCRIBE)

        raise ValueError(f"Unknown command type: {type(command)}")

    @staticmethod
    def deserialize_command(data: bytes) -> Command:
        if not data:
            raise ValueError("Empty command payload")

        command_type = data[0]
        body = data[1:]

        if command_type == BinarySerializer._COMMAND_MOVE:
            left_power, right_power = struct.unpack("<ff", body)
            return MoveCommand(left_power=left_power, right_power=right_power)

        if command_type == BinarySerializer._COMMAND_CLAW:
            (raw_action,) = struct.unpack("<B", body)
            return ClawCommand(action=ClawAction.OPEN if raw_action else ClawAction.CLOSE)

        if command_type == BinarySerializer._COMMAND_SUBSCRIBE:
            return SubscribeCommand()

        raise ValueError(f"Unknown command type: {command_type}")

    @staticmethod
    def serialize_measurements(measurements: Measurements) -> bytes:
        payload = bytearray()
        payload.extend(struct.pack("<q", measurements.timestamp))
        payload.extend(struct.pack("<H", len(measurements.lidar)))
        for measurement in measurements.lidar:
            # angle: degrees * 64
            # distance: millimeters * 4
            angle_raw = int(round(-measurement.angle * 64 * 57.295779513))
            distance_raw = int(round(measurement.distance * 4 * 1000.0)) & 0xFFFF
            payload.extend(struct.pack("<hH", angle_raw, distance_raw))
        payload.extend(
            struct.pack(
                "<ii",
                measurements.encoders.left_ticks,
                measurements.encoders.right_ticks,
            )
        )
        return bytes(payload)

    @staticmethod
    def deserialize_measurements(data: bytes) -> Measurements:
        offset = 0
        (timestamp,) = struct.unpack_from("<q", data, offset)
        offset += struct.calcsize("<q")

        (lidar_count,) = struct.unpack_from("<H", data, offset)
        offset += struct.calcsize("<H")

        lidar: list[LidarMeasurement] = []
        lidar_size = struct.calcsize("<hH")
        for _ in range(lidar_count):
            angle, distance = struct.unpack_from("<hH", data, offset)
            offset += lidar_size
            lidar.append(
                LidarMeasurement(
                    angle=-angle / (64 * 57.295779513),
                    distance=distance / (4 * 1000.0),
                )
            )

        left_ticks, right_ticks = struct.unpack_from("<ii", data, offset)
        encoders = EncodersMeasurement(
            left_ticks=left_ticks,
            right_ticks=right_ticks,
        )

        return Measurements(timestamp=timestamp, lidar=lidar, encoders=encoders)
