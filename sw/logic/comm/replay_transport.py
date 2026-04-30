import base64
import csv
import threading
from dataclasses import dataclass
from datetime import datetime
from typing import Optional

from .types import MessageCallback, Transport


@dataclass(frozen=True)
class _ReplayEvent:
    timestamp: datetime
    payload: bytes


class ReplayTransport(Transport):
    def __init__(self, csv_path: str, speed: float = 1.0):
        if speed <= 0.0:
            raise ValueError("speed must be positive")

        self.csv_path = csv_path
        self.speed = speed

        self._events = self._load_events(csv_path)
        self._callback: Optional[MessageCallback] = None
        self._thread: Optional[threading.Thread] = None
        self._running = False
        self._connected = False
        self._lock = threading.Lock()
        self._stop_event = threading.Event()

    def connect(self) -> None:
        self._connected = True

    def set_speed(self, speed: float) -> None:
        if speed <= 0.0:
            raise ValueError("speed must be positive")

        with self._lock:
            self.speed = speed

    def get_speed(self) -> float:
        with self._lock:
            return self.speed

    def start_receiving(self, callback: MessageCallback) -> None:
        with self._lock:
            self._callback = callback
            if not self._connected:
                self.connect()
            if self._running:
                return
            self._running = True
            self._stop_event.clear()

        self._thread = threading.Thread(target=self._replay_loop, daemon=True)
        self._thread.start()

    def stop_receiving(self) -> None:
        with self._lock:
            self._running = False
            self._stop_event.set()

        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

    def send(self, data: bytes) -> None:
        """read-only; outgoing sends are ignored."""
        _ = data

    def close(self) -> None:
        self.stop_receiving()
        self._connected = False

    def _replay_loop(self) -> None:
        if not self._events:
            with self._lock:
                self._running = False
            return

        previous_timestamp = self._events[0].timestamp

        for index, event in enumerate(self._events):
            with self._lock:
                if not self._running:
                    break
                callback = self._callback
                speed = self.speed

            if index > 0:
                recorded_delay = (event.timestamp - previous_timestamp).total_seconds()
                if recorded_delay > 0.0:
                    wall_delay = recorded_delay / speed
                    if self._stop_event.wait(timeout=wall_delay):
                        break

            with self._lock:
                if not self._running:
                    break
                callback = self._callback

            if callback is not None:
                callback.on_message(event.payload)

            previous_timestamp = event.timestamp

        with self._lock:
            self._running = False

    @staticmethod
    def _load_events(csv_path: str) -> list[_ReplayEvent]:
        with open(csv_path, "r", newline="", encoding="utf-8") as csv_file:
            reader = csv.DictReader(csv_file)
            events: list[_ReplayEvent] = []
            for row in reader:
                if row.get("event") != "receive":
                    continue

                ts_raw = row.get("timestamp_utc")
                payload = row.get("payload")
                if ts_raw is None or payload is None:
                    continue

                try:
                    timestamp = datetime.fromisoformat(ts_raw)
                except ValueError:
                    continue

                events.append(_ReplayEvent(timestamp=timestamp, payload=ReplayTransport._decode_payload(payload)))

        events.sort(key=lambda event: event.timestamp)
        return events

    @staticmethod
    def _decode_payload(payload: str) -> bytes:
        if payload.startswith("b64:"):
            return base64.b64decode(payload[4:])
        return payload.encode("utf-8")
