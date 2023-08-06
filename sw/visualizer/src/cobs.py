from typing import Optional


def cobs_encode(data: bytes) -> bytes:
    """Encode a string using Consistent Overhead Byte Stuffing (COBS).

    Args:
        data: The data to be encoded.

    Returns:
        The encoded data.
    """

    data = data + b"\x00"

    encoded = bytearray()
    start = 0
    while True:
        zero_index = data.find(b"\x00", start)
        if zero_index == -1:
            break

        encoded.append(zero_index - start + 1)
        encoded.extend(data[start:zero_index])

        start = zero_index + 1

    encoded.extend(data[start:])
    encoded.append(0x00)

    return bytes(encoded)


def cobs_decode(data: bytes) -> bytes:
    """Decode a string using Consistent Overhead Byte Stuffing (COBS).

    Args:
        data: The data to be decoded.

    Returns:
        The decoded data.

    Raises:
        Exception: If the encoded data is invalid.
    """

    if not data:
        return b""

    if data[-1] != 0x00:
        raise Exception("COBS decode error: unexpected end of data")

    decoded = bytearray()
    index = 0

    while index < len(data) - 1:
        dist = data[index]
        if dist == 0:
            raise Exception("COBS decode error: unexpected 0 byte")

        index += 1
        decoded.extend(data[index:index + dist - 1])
        index += dist - 1

        if index < len(data) - 1:
            decoded.append(0)

    return bytes(decoded)


class CobsStreamDecoder:
    """A stream decoder for Consistent Overhead Byte Stuffing (COBS).

    This class is used to decode a stream of bytes that have been encoded
    using COBS.
    """

    def __init__(self) -> None:
        self._buffer = bytearray()

    def receive(self, byte: int) -> Optional[bytes]:
        """Receive a byte of encoded data.

        Args:
            byte: The byte to be received.

        Returns:
            The decoded data, if any.
        """

        self._buffer.append(byte)

        if byte == 0x00:
            data = bytes(self._buffer)
            self._buffer.clear()
            return cobs_decode(data)

        return None

    def reset(self) -> None:
        """Reset the decoder."""

        self._buffer.clear()
