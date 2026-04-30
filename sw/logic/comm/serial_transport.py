from __future__ import annotations

import struct
import threading
from typing import Any, Optional

from crc import Calculator, Configuration
import serial

from .types import MessageCallback, Transport


class SerialTransport(Transport):
    _FRAME_INIT = 0xA5
    _HEADER_SIZE = 6
    _MAX_PAYLOAD_SIZE = 2048
    _CRC = Calculator(  # Matches raw esp_rom_crc8_be(0, ...) behavior.
        Configuration(
            width=8,
            polynomial=0x07,
            init_value=0xFF,
            final_xor_value=0xFF,
            reverse_input=False,
            reverse_output=False,
        )
    )

    def __init__(
        self,
        device: str,
        baud_rate: int = 921600,
        timeout: float = 0.05,
    ) -> None:
        self.device = device
        self.baud_rate = baud_rate
        self.timeout = timeout
        self._serial: Optional[Any] = None
        self._receive_thread: Optional[threading.Thread] = None
        self._running = False
        self._callback: Optional[MessageCallback] = None
        self._tx_nonce = 0

    def connect(self) -> None:
        if self._serial is not None:
            return

        self._serial = serial.Serial(
            port=self.device,
            baudrate=self.baud_rate,
            timeout=self.timeout,
        )

    def start_receiving(self, callback: MessageCallback) -> None:
        self._callback = callback
        if self._serial is None:
            self.connect()

        if self._running:
            return

        self._running = True
        self._receive_thread = threading.Thread(target=self._receive_loop, daemon=True)
        self._receive_thread.start()

    def stop_receiving(self) -> None:
        self._running = False
        if self._receive_thread is not None:
            self._receive_thread.join(timeout=2.0)
            self._receive_thread = None

    def send(self, data: bytes) -> None:
        if self._serial is None:
            raise RuntimeError("Transport not connected. Call connect() first.")
        if len(data) > self._MAX_PAYLOAD_SIZE:
            raise ValueError(f"Payload too large: {len(data)} > {self._MAX_PAYLOAD_SIZE}")

        frame = self._frame_payload(self._tx_nonce, data)
        self._tx_nonce = (self._tx_nonce + 1) & 0xFF
        self._serial.write(frame)

    def close(self) -> None:
        self.stop_receiving()
        if self._serial is not None:
            self._serial.close()
            self._serial = None

    def _receive_loop(self) -> None:
        while self._running:
            try:
                payload = self._read_frame()
            except Exception as error:
                if self._running and self._callback is not None:
                    self._callback.on_error(error)
                continue

            if payload is not None and self._callback is not None:
                self._callback.on_message(payload)

    def _read_frame(self) -> Optional[bytes]:
        if self._serial is None:
            return None

        init = self._read_exact(1)
        if init != bytes((self._FRAME_INIT,)):
            return None

        header = self._read_exact(self._HEADER_SIZE - 1)
        if header is None:
            return None

        nonce, size, data_checksum, checksum = struct.unpack("<BHBB", header)
        if size > self._MAX_PAYLOAD_SIZE:
            return None

        expected_checksum = self._compute_checksum(
            bytes((self._FRAME_INIT, nonce)) + struct.pack("<H", size) + bytes((data_checksum,))
        )
        if checksum != expected_checksum:
            return None

        payload = self._read_exact(size)
        if payload is None:
            return None

        if self._compute_checksum(payload) != data_checksum:
            return None
        return payload

    def _read_exact(self, size: int) -> Optional[bytes]:
        if self._serial is None:
            return None

        buffer = bytearray()
        while len(buffer) < size and self._running:
            chunk = self._serial.read(size - len(buffer))
            if not chunk:
                return None
            buffer.extend(chunk)
        return bytes(buffer) if len(buffer) == size else None

    @classmethod
    def _frame_payload(cls, nonce: int, payload: bytes) -> bytes:
        data_checksum = cls._compute_checksum(payload)
        header_without_checksum = (
            bytes((cls._FRAME_INIT, nonce & 0xFF))
            + struct.pack("<H", len(payload))
            + bytes((data_checksum,))
        )
        checksum = cls._compute_checksum(header_without_checksum)
        return header_without_checksum + bytes((checksum,)) + payload

    @staticmethod
    def _compute_checksum(data: bytes) -> int:
        return SerialTransport._CRC.checksum(data)
