import base64
import csv
import threading
from datetime import datetime, timezone
from typing import Optional

from .types import MessageCallback, Transport


class RecordingTransport(Transport, MessageCallback):
    def __init__(self, transport: Transport, csv_path: str):
        self.transport = transport
        self.callback: Optional[MessageCallback] = None
        self._lock = threading.Lock()
        self._csv_file = open(csv_path, "a", newline="", encoding="utf-8")
        self._csv_writer = csv.writer(self._csv_file)
        self._closed = False

        if self._csv_file.tell() == 0:
            self._csv_writer.writerow(
                [
                    "timestamp_utc",
                    "event",
                    "payload",
                ]
            )
            self._csv_file.flush()

    def connect(self) -> None:
        self.transport.connect()

    def start_receiving(self, callback: MessageCallback) -> None:
        self.callback = callback
        self.transport.start_receiving(self)

    def stop_receiving(self) -> None:
        self.transport.stop_receiving()

    def send(self, data: bytes) -> None:
        with self._lock:
            self._write_csv_row(event="send", payload=data)
        self.transport.send(data)

    def close(self) -> None:
        with self._lock:
            if self._closed:
                return
            self._closed = True

        self.transport.close()
        self._csv_file.close()

    def on_message(self, data: bytes) -> None:
        with self._lock:
            self._write_csv_row(event="receive", payload=data)

        if self.callback is not None:
            self.callback.on_message(data)

    def on_error(self, error: Exception) -> None:
        if self.callback is not None:
            self.callback.on_error(error)

    def _write_csv_row(
        self,
        event: str,
        payload: bytes = b"",
    ) -> None:
        timestamp_utc = datetime.now(timezone.utc).isoformat()
        payload_text = self._payload_to_text(payload)
        self._csv_writer.writerow(
            [
                timestamp_utc,
                event,
                payload_text,
            ]
        )
        self._csv_file.flush()

    @staticmethod
    def _payload_to_text(payload: bytes) -> str:
        try:
            text = payload.decode("ascii")
        except UnicodeDecodeError:
            return "b64:" + base64.b64encode(payload).decode("ascii")

        if all((character.isprintable() or character in "\t\n\r") for character in text):
            return text

        return "b64:" + base64.b64encode(payload).decode("ascii")
