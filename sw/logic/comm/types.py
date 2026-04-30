from typing import Protocol

from .messages import Command, Measurements


class MessageCallback(Protocol):
    """Callback interface for incoming transport payloads."""

    def on_message(self, data: bytes) -> None:
        """Called when a payload is received."""
        pass

    def on_error(self, error: Exception) -> None:
        """Called when an error occurs while receiving."""
        pass


class Transport(Protocol):
    """Transport interface for sending/receiving payloads."""

    def connect(self) -> None:
        """Initialize transport resources for sending."""
        pass

    def start_receiving(self, callback: MessageCallback) -> None:
        """Start receiving incoming payloads."""
        pass

    def stop_receiving(self) -> None:
        """Stop receiving incoming payloads."""
        pass

    def send(self, data: bytes) -> None:
        """Send payload over the transport."""
        pass

    def close(self) -> None:
        """Close transport resources."""
        pass


class Serializer(Protocol):
    """Serializer interface for command and measurement payloads."""

    def serialize_command(self, command: Command) -> bytes:
        """Serialize command object to payload bytes."""
        pass

    def deserialize_command(self, data: bytes) -> Command:
        """Deserialize payload bytes to command object."""
        pass

    def serialize_measurements(self, measurements: Measurements) -> bytes:
        """Serialize measurements object to payload bytes."""
        pass

    def deserialize_measurements(self, data: bytes) -> Measurements:
        """Deserialize payload bytes to measurements object."""
        pass
