from abc import ABC, abstractmethod
from ptypes import ProtocolType, Uint8, Uint16, Rgb332
from typing import Generic, TypeVar

T = TypeVar("T", bound=ProtocolType)


# TODO: encode optional arguments only if they are not the default value


class OptArg(Generic[T]):
    def __init__(self, default: T):
        self.default = default

    def from_bytes(self, data: bytes) -> tuple[T, bytes]:
        if not data:
            return self.default, data

        res, data = type(self.default).from_bytes(data)
        assert isinstance(res, type(self.default))  # TODO: fix type checking

        return res, data


class Command(ABC):
    """A command."""

    @staticmethod
    @abstractmethod
    def from_bytes(data: bytes) -> "Command":
        """Create a Command from bytes.

        Args:
            data: The bytes to be converted.

        Returns:
            The Command
        """

        pass

    @abstractmethod
    def to_bytes(self) -> bytes:
        """Convert a Command to bytes.

        Returns:
            The bytes.
        """

        pass

    @abstractmethod
    def __repr__(self) -> str:
        pass


COMMANDS: dict[int, type[Command]] = {}
COMMANDS_INVERSE: dict[type[Command], int] = {}


class ClearScreen(Command):
    """Clear the screen."""

    @staticmethod
    def from_bytes(data: bytes) -> "ClearScreen":
        """Create a ClearScreen from bytes.

        Args:
            data: The bytes to be converted.

        Returns:
            The ClearScreen.
        """

        return ClearScreen()

    def to_bytes(self) -> bytes:
        """Convert a ClearScreen to bytes.

        Returns:
            The bytes.
        """

        return bytes([COMMANDS_INVERSE[type(self)]])

    def __repr__(self) -> str:
        return "ClearScreen()"


class ClearLayer(Command):
    """Clear a layer."""

    def __init__(self, layer: Uint8):
        self.layer = layer

    @staticmethod
    def from_bytes(data: bytes) -> "ClearLayer":
        """Create a ClearLayer from bytes.

        Args:
            data: The bytes to be converted.

        Returns:
            The ClearLayer.
        """

        layer, data = Uint8.from_bytes(data)

        return ClearLayer(layer)

    def to_bytes(self) -> bytes:
        """Convert a ClearLayer to bytes.

        Returns:
            The bytes.
        """

        return bytes(COMMANDS_INVERSE[type(self)]) + self.layer.to_bytes()

    def __repr__(self) -> str:
        return f"ClearLayer({self.layer})"


class DrawPoint(Command):
    """Draw a point."""

    def __init__(self, x: Uint16, y: Uint16, layer: Uint8, color: Rgb332, thickness: Uint16):
        self.x = x
        self.y = y
        self.layer = layer
        self.color = color
        self.thickness = thickness

    @staticmethod
    def from_bytes(data: bytes) -> "DrawPoint":
        """Create a DrawPoint from bytes.

        Args:
            data: The bytes to be converted.

        Returns:
            The DrawPoint.
        """

        x, data = Uint16.from_bytes(data)
        y, data = Uint16.from_bytes(data)
        layer, data = Uint8.from_bytes(data)
        color, data = OptArg(Rgb332((0, 0, 0))).from_bytes(data)
        radius, data = OptArg(Uint16(1)).from_bytes(data)

        return DrawPoint(x, y, layer, color, radius)

    def to_bytes(self) -> bytes:
        """Convert a DrawPoint to bytes.

        Returns:
            The bytes.
        """

        return (
            bytes(COMMANDS_INVERSE[type(self)])
            + self.x.to_bytes()
            + self.y.to_bytes()
            + self.layer.to_bytes()
            + self.color.to_bytes()
            + self.thickness.to_bytes()
        )

    def __repr__(self) -> str:
        return f"DrawPoint({self.x}, {self.y}, {self.color}, {self.thickness})"


class DrawLine(Command):
    """Draw a line."""

    def __init__(self, x1: Uint16, y1: Uint16, x2: Uint16, y2: Uint16,
                 layer: Uint8, color: Rgb332, width: Uint16):
        self.x1 = x1
        self.y1 = y1
        self.x2 = x2
        self.y2 = y2
        self.layer = layer
        self.color = color
        self.width = width

    @staticmethod
    def from_bytes(data: bytes) -> "DrawLine":
        """Create a DrawLine from bytes.

        Args:
            data: The bytes to be converted.

        Returns:
            The DrawLine.
        """

        x1, data = Uint16.from_bytes(data)
        y1, data = Uint16.from_bytes(data)
        x2, data = Uint16.from_bytes(data)
        y2, data = Uint16.from_bytes(data)
        layer, data = Uint8.from_bytes(data)
        color, data = OptArg(Rgb332((0, 0, 0))).from_bytes(data)
        width, data = OptArg(Uint16(1)).from_bytes(data)

        return DrawLine(x1, y1, x2, y2, layer, color, width)

    def to_bytes(self) -> bytes:
        """Convert a DrawLine to bytes.

        Returns:
            The bytes.
        """

        return (
            bytes([COMMANDS_INVERSE[type(self)]])
            + self.x1.to_bytes()
            + self.y1.to_bytes()
            + self.x2.to_bytes()
            + self.y2.to_bytes()
            + self.layer.to_bytes()
            + self.color.to_bytes()
            + self.width.to_bytes()
        )

    def __repr__(self) -> str:
        return f"DrawLine({self.x1}, {self.y1}, {self.x2}, {self.y2}, {self.color}, {self.width})"


class DrawRectangle(Command):
    """Draw a rectangle."""

    def __init__(self, x1: Uint16, y1: Uint16, x2: Uint16, y2: Uint16, layer: Uint8, color: Rgb332, width: Uint16):
        self.x1 = x1
        self.y1 = y1
        self.x2 = x2
        self.y2 = y2
        self.layer = layer
        self.color = color
        self.width = width

    @staticmethod
    def from_bytes(data: bytes) -> "DrawRectangle":
        """Create a DrawRectangle from bytes.

        Args:
            data: The bytes to be converted.

        Returns:
            The DrawRectangle.
        """

        x1, data = Uint16.from_bytes(data)
        y1, data = Uint16.from_bytes(data)
        x2, data = Uint16.from_bytes(data)
        y2, data = Uint16.from_bytes(data)
        layer, data = Uint8.from_bytes(data)
        color, data = OptArg(Rgb332((0, 0, 0))).from_bytes(data)
        width, data = OptArg(Uint16(1)).from_bytes(data)

        return DrawRectangle(x1, y1, x2, y2, layer, color, width)

    def to_bytes(self) -> bytes:
        """Convert a DrawRectangle to bytes.

        Returns:
            The bytes.
        """

        return (
            bytes(COMMANDS_INVERSE[type(self)])
            + self.x1.to_bytes()
            + self.y1.to_bytes()
            + self.x2.to_bytes()
            + self.y2.to_bytes()
            + self.layer.to_bytes()
            + self.color.to_bytes()
            + self.width.to_bytes()
        )

    def __repr__(self) -> str:
        return f"DrawRectangle({self.x1}, {self.y1}, {self.x2}, {self.y2}, {self.color}, {self.width})"


class DrawCircle(Command):
    """Draw a circle."""

    def __init__(self, x: Uint16, y: Uint16, r: Uint16, layer: Uint8, color: Rgb332, width: Uint16):
        self.x = x
        self.y = y
        self.r = r
        self.layer = layer
        self.color = color
        self.width = width

    @staticmethod
    def from_bytes(data: bytes) -> "DrawCircle":
        """Create a DrawCircle from bytes.

        Args:
            data: The bytes to be converted.

        Returns:
            The DrawCircle.
        """

        x, data = Uint16.from_bytes(data)
        y, data = Uint16.from_bytes(data)
        r, data = Uint16.from_bytes(data)
        layer, data = Uint8.from_bytes(data)
        color, data = OptArg(Rgb332((0, 0, 0))).from_bytes(data)
        width, data = OptArg(Uint16(1)).from_bytes(data)

        return DrawCircle(x, y, r, layer, color, width)

    def to_bytes(self) -> bytes:
        """Convert a DrawCircle to bytes.

        Returns:
            The bytes.
        """

        return (
            bytes(COMMANDS_INVERSE[type(self)])
            + self.x.to_bytes()
            + self.y.to_bytes()
            + self.r.to_bytes()
            + self.layer.to_bytes()
            + self.color.to_bytes()
            + self.width.to_bytes()
        )

    def __repr__(self) -> str:
        return f"DrawCircle({self.x}, {self.y}, {self.r}, {self.color}, {self.width})"


COMMANDS = {
    0x01: ClearScreen,
    0x02: ClearLayer,
    0x10: DrawPoint,
    0x11: DrawLine,
    0x12: DrawRectangle,
    0x13: DrawCircle,
}
COMMANDS_INVERSE = {v: k for k, v in COMMANDS.items()}


def cmd_from_bytes(data: bytes) -> Command:
    """Create a Command from bytes.

    Args:
        data: The bytes to be converted.

    Returns:
        The Command
    """

    if not data:
        raise Exception("Command decode error: unexpected end of data")

    cmd_id = data[0]
    cmd_type = COMMANDS.get(cmd_id)

    if not cmd_type:
        raise Exception(f"Command decode error: unknown command id {cmd_id}")

    return cmd_type.from_bytes(data[1:])
