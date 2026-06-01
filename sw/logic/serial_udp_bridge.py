from __future__ import annotations

import socket
import sys
import threading
import time
from typing import Optional

from comm.serial_transport import SerialTransport
from comm.types import MessageCallback
from util.init_common import CONTROLLER_RECEIVE_PORT, SIM_PORT


class _SerialToUdpCallback(MessageCallback):
    def __init__(self, bridge: SerialUdpBridge) -> None:
        self._bridge = bridge

    def on_message(self, data: bytes) -> None:
        if self._bridge._controller_addr is None:
            return
        try:
            self._bridge._udp_send.sendto(data, self._bridge._controller_addr)
        except Exception as e:
            print(f"UDP send error: {e}", file=sys.stderr)

    def on_error(self, error: Exception) -> None:
        print(f"Serial error: {error}", file=sys.stderr)


class SerialUdpBridge:
    def __init__(self) -> None:
        self._serial = SerialTransport(device="/dev/ttyUSB0", baud_rate=921600)
        self._udp_recv = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._udp_recv.bind(("", SIM_PORT))
        self._udp_send = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._controller_addr: Optional[tuple[str, int]] = None
        self._running = False
        self._udp_thread: Optional[threading.Thread] = None

    def start(self) -> None:
        self._serial.connect()
        self._serial.start_receiving(_SerialToUdpCallback(self))
        self._running = True
        self._udp_thread = threading.Thread(target=self._udp_receive_loop, daemon=True)
        self._udp_thread.start()

    def stop(self) -> None:
        self._running = False
        try:
            self._serial.close()
        finally:
            self._udp_recv.close()
            self._udp_send.close()
        if self._udp_thread is not None:
            self._udp_thread.join(timeout=2.0)

    def _udp_receive_loop(self) -> None:
        while self._running:
            try:
                self._udp_recv.settimeout(1.0)
                data, addr = self._udp_recv.recvfrom(65535)
                self._controller_addr = (addr[0], CONTROLLER_RECEIVE_PORT)
                self._serial.send(data)
            except socket.timeout:
                continue
            except Exception as e:
                if self._running:
                    print(f"UDP receive error: {e}", file=sys.stderr)


def main() -> None:
    bridge = SerialUdpBridge()
    bridge.start()
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.stop()


if __name__ == "__main__":
    main()
