import socket
import threading
from typing import Optional
from .types import MessageCallback, Transport


class UdpTransport(Transport):
    _RECV_BUFFER_SIZE = 65535

    def __init__(self, host: str, port: int, receive_port: int):
        self.host = host
        self.send_port = port
        self.receive_port = receive_port
        self.send_socket: Optional[socket.socket] = None
        self.receive_socket: Optional[socket.socket] = None
        self.receive_thread: Optional[threading.Thread] = None
        self.running = False
        self.callback: Optional[MessageCallback] = None

    def connect(self) -> None:
        if self.send_socket is not None:
            return

        self.send_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def start_receiving(self, callback: MessageCallback) -> None:
        self.callback = callback
        if self.send_socket is None:
            self.connect()

        assert self.send_socket is not None

        # Use one socket for both directions so outgoing commands (including
        # subscribe) have the same source port we listen on.
        self.receive_socket = self.send_socket
        local_port = self.receive_socket.getsockname()[1]
        if local_port == 0:
            self.receive_socket.bind(("", self.receive_port))

        self.running = True
        self.receive_thread = threading.Thread(target=self._receive_loop, daemon=True)
        self.receive_thread.start()

    def stop_receiving(self) -> None:
        self.running = False
        if self.receive_thread:
            self.receive_thread.join(timeout=2.0)
        if self.receive_socket and self.receive_socket is not self.send_socket:
            self.receive_socket.close()
        self.receive_socket = None

    def send(self, data: bytes) -> None:
        if self.send_socket is None:
            raise RuntimeError("Transport not connected. Call connect() first.")

        self.send_socket.sendto(data, (self.host, self.send_port))

    def _receive_loop(self) -> None:
        if self.receive_socket is None or self.callback is None:
            return

        while self.running:
            try:
                self.receive_socket.settimeout(1.0)
                data, _ = self.receive_socket.recvfrom(self._RECV_BUFFER_SIZE)
                self.callback.on_message(data)
            except socket.timeout:
                continue
            except Exception as e:
                if self.running and self.callback:
                    self.callback.on_error(e)

    def close(self) -> None:
        self.stop_receiving()
        if self.send_socket:
            self.send_socket.close()
            self.send_socket = None
