from abc import ABC, abstractmethod


class ProtocolType(ABC):
    """A protocol type."""

    @staticmethod
    @abstractmethod
    def from_bytes(data: bytes) -> tuple["ProtocolType", bytes]:
        """Parse a protocol type from bytes.

        Args:
            data: The bytes to be parsed.

        Returns:
            A tuple containing the parsed protocol type and the unused bytes.
        """

        pass

    @staticmethod
    @abstractmethod
    def from_string(string: str) -> "ProtocolType":
        """Parse a protocol type from a string.

        Args:
            string: The string to be parsed.
        """

        pass

    @abstractmethod
    def to_bytes(self) -> bytes:
        """Convert the protocol type to bytes."""

        pass

    @abstractmethod
    def __str__(self) -> str:
        pass


PROTOCOL_TYPES: dict[int, type[ProtocolType]] = {}


class Nil(ProtocolType):
    """A nil value."""

    @staticmethod
    def from_bytes(data: bytes) -> tuple["Nil", bytes]:
        """Create a Nil from bytes.

        Args:
            data: The bytes to be converted.

        Returns:
            A tuple containing the Nil and the unused bytes.
        """

        return Nil(), data

    @staticmethod
    def from_string(string: str) -> "Nil":
        """Parse a Nil from a string.

        Args:
            string: The string to be parsed.
        """

        if string != "nil":
            raise Exception("Invalid nil value")

        return Nil()

    def to_bytes(self) -> bytes:
        """Convert to bytes.

        Returns:
            The bytes.
        """

        return b""

    def __str__(self) -> str:
        return "nil"


class Uint8(ProtocolType):
    """An 8-bit unsigned integer."""

    def __init__(self, value: int) -> None:
        if value < 0 or value > 255:
            raise Exception("Value out of range")

        self.value = value

    @staticmethod
    def from_bytes(data: bytes) -> tuple["Uint8", bytes]:
        """Create a Uint8 from bytes.

        Args:
            data: The bytes to be converted.

        Returns:
            A tuple containing the Uint8 and the unused bytes.
        """

        if len(data) < 1:
            raise Exception("Not enough bytes")

        return Uint8(int.from_bytes(data[0:1], "big", signed=False)), data[1:]

    @staticmethod
    def from_string(string: str) -> "Uint8":
        """Parse a Uint8 from a string.

        Args:
            string: The string to be parsed.

        Returns:
            The parsed Uint8.
        """

        return Uint8(int(string))

    def to_bytes(self) -> bytes:
        """Convert to bytes.

        Returns:
            The bytes.
        """

        return self.value.to_bytes(1, "big", signed=False)

    def __str__(self) -> str:
        return str(self.value)


class Int8(ProtocolType):
    """An 8-bit signed integer."""

    def __init__(self, value: int) -> None:
        if value < -128 or value > 127:
            raise Exception("Value out of range")

        self.value = value

    @staticmethod
    def from_bytes(data: bytes) -> tuple["Int8", bytes]:
        """Create an Int8 from bytes.

        Args:
            data: The bytes to be converted.

        Returns:
            A tuple containing the Int8 and the unused bytes.
        """

        if len(data) < 1:
            raise Exception("Not enough bytes")

        return Int8(int.from_bytes(data[0:1], "big", signed=True)), data[1:]

    @staticmethod
    def from_string(string: str) -> "Int8":
        """Parse an Int8 from a string.

        Args:
            string: The string to be parsed.

        Returns:
            The parsed Int8.
        """

        return Int8(int(string))

    def to_bytes(self) -> bytes:
        """Convert to bytes.

        Returns:
            The bytes.
        """

        return self.value.to_bytes(1, "big", signed=True)

    def __str__(self) -> str:
        return str(self.value)


class Uint16(ProtocolType):
    """A 16-bit unsigned integer."""

    def __init__(self, value: int) -> None:
        if value < 0 or value > 65535:
            raise Exception("Value out of range")

        self.value = value

    @staticmethod
    def from_bytes(data: bytes) -> tuple["Uint16", bytes]:
        """Create a Uint16 from bytes.

        Args:
            data: The bytes to be converted.

        Returns:
            A tuple containing the Uint16 and the unused bytes.
        """

        if len(data) < 2:
            raise Exception("Not enough bytes")

        return Uint16(int.from_bytes(data[0:2], "big", signed=False)), data[2:]

    @staticmethod
    def from_string(string: str) -> "Uint16":
        """Parse a Uint16 from a string.

        Args:
            string: The string to be parsed.

        Returns:
            The parsed Uint16.
        """

        return Uint16(int(string))

    def to_bytes(self) -> bytes:
        """Convert to bytes.

        Returns:
            The bytes.
        """

        return self.value.to_bytes(2, "big", signed=False)

    def __str__(self) -> str:
        return str(self.value)


class Int16(ProtocolType):
    """A 16-bit signed integer."""

    def __init__(self, value: int) -> None:
        if value < -32768 or value > 32767:
            raise Exception("Value out of range")

        self.value = value

    @staticmethod
    def from_bytes(data: bytes) -> tuple["Int16", bytes]:
        """Create an Int16 from bytes.

        Args:
            data: The bytes to be converted.

        Returns:
            A tuple containing the Int16 and the unused bytes.
        """

        if len(data) < 2:
            raise Exception("Not enough bytes")

        return Int16(int.from_bytes(data[0:2], "big", signed=True)), data[2:]

    @staticmethod
    def from_string(string: str) -> "Int16":
        """Parse an Int16 from a string.

        Args:
            string: The string to be parsed.

        Returns:
            The parsed Int16.
        """

        return Int16(int(string))

    def to_bytes(self) -> bytes:
        """Convert to bytes.

        Returns:
            The bytes.
        """

        return self.value.to_bytes(2, "big", signed=True)

    def __str__(self) -> str:
        return str(self.value)


class Uint32(ProtocolType):
    """A 32-bit unsigned integer."""

    def __init__(self, value: int) -> None:
        if value < 0 or value > 4294967295:
            raise Exception("Value out of range")

        self.value = value

    @staticmethod
    def from_bytes(data: bytes) -> tuple["Uint32", bytes]:
        """Create a Uint32 from bytes.

        Args:
            data: The bytes to be converted.

        Returns:
            A tuple containing the Uint32 and the unused bytes.
        """

        if len(data) < 4:
            raise Exception("Not enough bytes")

        return Uint32(int.from_bytes(data[0:4], "big", signed=False)), data[4:]

    @staticmethod
    def from_string(string: str) -> "Uint32":
        """Parse a Uint32 from a string.

        Args:
            string: The string to be parsed.

        Returns:
            The parsed Uint32.
        """

        return Uint32(int(string))

    def to_bytes(self) -> bytes:
        """Convert to bytes.

        Returns:
            The bytes.
        """

        return self.value.to_bytes(4, "big", signed=False)

    def __str__(self) -> str:
        return str(self.value)


class Int32(ProtocolType):
    """A 32-bit signed integer."""

    def __init__(self, value: int) -> None:
        if value < -2147483648 or value > 2147483647:
            raise Exception("Value out of range")

        self.value = value

    @staticmethod
    def from_bytes(data: bytes) -> tuple["Int32", bytes]:
        """Create an Int32 from bytes.

        Args:
            data: The bytes to be converted.

        Returns:
            A tuple containing the Int32 and the unused bytes.
        """

        if len(data) < 4:
            raise Exception("Not enough bytes")

        return Int32(int.from_bytes(data[0:4], "big", signed=True)), data[4:]

    @staticmethod
    def from_string(string: str) -> "Int32":
        """Parse an Int32 from a string.

        Args:
            string: The string to be parsed.

        Returns:
            The parsed Int32.
        """

        return Int32(int(string))

    def to_bytes(self) -> bytes:
        """Convert to bytes.

        Returns:
            The bytes.
        """

        return self.value.to_bytes(4, "big", signed=True)

    def __str__(self) -> str:
        return str(self.value)


class String(ProtocolType):
    """A null-terminated string."""

    def __init__(self, value: str) -> None:
        self.value = value

    @staticmethod
    def from_bytes(data: bytes) -> tuple["String", bytes]:
        """Create a String from bytes.

        Args:
            data: The bytes to be converted.

        Returns:
            A tuple containing the String and the unused bytes.
        """

        index = data.find(b"\x00")
        if index == -1:
            raise Exception("Not enough bytes")

        return String(data[0:index].decode("utf-8")), data[index + 1:]

    @staticmethod
    def from_string(string: str) -> "String":
        """Parse a String from a string.

        Args:
            string: The string to be parsed.

        Returns:
            The parsed String.
        """

        return String(string)

    def to_bytes(self) -> bytes:
        """Convert to bytes.

        Returns:
            The bytes.
        """

        return self.value.encode("utf-8") + b"\x00"

    def __str__(self) -> str:
        return self.value


class Rgb332(ProtocolType):
    """An RGB color encoded as 3 bits for red, 3 bits for green, and 2 bits for blue."""

    def __init__(self, value: tuple[int, int, int]) -> None:
        red, green, blue = value

        if red < 0 or red > 7:
            raise ValueError("Red value out of range")
        if green < 0 or green > 7:
            raise ValueError("Green value out of range")
        if blue < 0 or blue > 3:
            raise ValueError("Blue value out of range")

        self.value = value

    @staticmethod
    def from_bytes(data: bytes) -> tuple["Rgb332", bytes]:
        """Create an Rgb color from bytes.

        Args:
            data: The bytes to be converted.

        Returns:
            A tuple containing the Rgb and the unused bytes.
        """

        if len(data) < 1:
            raise Exception("Not enough bytes")

        value = int.from_bytes(data[0:1], "big", signed=False)

        red = (value >> 5) & 0b111
        green = (value >> 2) & 0b111
        blue = value & 0b11

        return Rgb332((red, green, blue)), data[1:]

    @staticmethod
    def from_string(string: str) -> "Rgb332":
        """Parse an Rgb color from a string.

        Args:
            string: The string to be parsed.

        Returns:
            The parsed Rgb.
        """

        red, green, blue = string.split(",")
        return Rgb332((int(red), int(green), int(blue)))

    def to_bytes(self) -> bytes:
        """Convert an Rgb color to bytes."""

        red, green, blue = self.value

        # map 8-bit values to 3-bit values
        red = (red >> 5) & 0b111
        green = (green >> 5) & 0b111
        blue = (blue >> 6) & 0b11

        value = (red << 5) | (green << 2) | blue

        return value.to_bytes(1, "big", signed=False)

    def __str__(self) -> str:
        return f"{self.value[0]},{self.value[1]},{self.value[2]}"

    def to_rgb888(self) -> tuple[int, int, int]:
        """Convert to an RGB color encoded as 8 bits for red, 8 bits for green, and 8 bits for blue."""

        red, green, blue = self.value

        # map 3-bit values to 8-bit values
        red = (red << 5) | (red << 2) | (red >> 1)
        green = (green << 5) | (green << 2) | (green >> 1)
        blue = (blue << 6) | (blue << 4) | (blue << 2) | blue

        return red, green, blue


class Type(ProtocolType):
    """A type."""

    def __init__(self, type: type[ProtocolType]) -> None:
        self.type = type

    @staticmethod
    def from_bytes(data: bytes) -> tuple["Type", bytes]:
        """Create a Type from bytes.

        Args:
            data: The bytes to be converted.

        Returns:
            A tuple containing the Type and the unused bytes.
        """

        typeId, data = Uint8.from_bytes(data)
        if typeId.value not in PROTOCOL_TYPES:
            raise Exception("Unknown type")

        return Type(PROTOCOL_TYPES[typeId.value]), data

    @staticmethod
    def from_string(string: str) -> "Type":
        """Parse a Type from a string.

        Args:
            string: The string to be parsed.

        Returns:
            The parsed Type.
        """

        raise Exception("Cannot parse type from string")

    def to_bytes(self) -> bytes:
        """Convert to bytes.

        Returns:
            The bytes.
        """

        for typeId, type in PROTOCOL_TYPES.items():
            if type == self.type:
                return Uint8(typeId).to_bytes()

        raise Exception("Unknown type")

    def __str__(self) -> str:
        return self.type.__name__


PROTOCOL_TYPES = {
    0x00: Nil,
    0x01: Uint8,
    0x02: Int8,
    0x03: Uint16,
    0x04: Int16,
    0x05: Uint32,
    0x06: Int32,
    0x07: String,
    0x08: Rgb332,
    0x09: Type,
}
