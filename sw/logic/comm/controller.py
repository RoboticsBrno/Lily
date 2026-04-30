from typing import Callable, Optional

from .messages import Command, Measurements
from .types import MessageCallback, Serializer, Transport


class MeasurementCallback(MessageCallback):
    def __init__(self, serializer: Serializer, on_measurement: Callable[[Measurements], None]):
        self.serializer = serializer
        self.on_measurement = on_measurement

    def on_message(self, data: bytes) -> None:
        measurements = self.serializer.deserialize_measurements(data)
        self.on_measurement(measurements)

    def on_error(self, error: Exception) -> None:
        print(f"Controller communication error: {error}")


class Controller:
    def __init__(self, transport: Transport, serializer: Serializer):
        self.transport = transport
        self.serializer = serializer
        self.on_measurement: Optional[Callable[[Measurements], None]] = None

    def start(self) -> None:
        self.transport.connect()
        self.transport.start_receiving(MeasurementCallback(self.serializer, self._handle_measurement))

    def stop(self) -> None:
        self.transport.close()

    def send_command(self, command: Command) -> None:
        payload = self.serializer.serialize_command(command)
        self.transport.send(payload)

    def set_measurement_callback(self, callback: Callable[[Measurements], None]) -> None:
        self.on_measurement = callback

    def _handle_measurement(self, measurements: Measurements) -> None:
        if self.on_measurement:
            self.on_measurement(measurements)
