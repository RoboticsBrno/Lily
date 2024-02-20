from protocol import ClearLayer, Command, ClearScreen, DrawCircle, DrawLine, DrawPoint
from ptypes import Rgb332, Uint16, Uint8
from cobs import cobs_encode
import socket


class VizCtl:
    def __init__(self, ip: str, port: int):
        self.ip = ip
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def clear_screen(self) -> None:
        self.send_command(ClearScreen())

    def clear_layer(self, layer: int) -> None:
        self.send_command(ClearLayer(Uint8(layer)))

    def draw_point(self, x: int, y: int, layer: int,
                   color: tuple[int, int, int] = (0, 0, 0), thickness: int = 1) -> None:
        self.send_command(DrawPoint(
            Uint16(x),
            Uint16(y),
            Uint8(layer),
            Rgb332(color),
            Uint16(thickness)
        ))

    def draw_line(self, x1: int, y1: int, x2: int, y2: int,
                  layer: int, color: tuple[int, int, int] = (0, 0, 0), thickness: int = 1) -> None:
        self.send_command(DrawLine(
            Uint16(x1),
            Uint16(y1),
            Uint16(x2),
            Uint16(y2),
            Uint8(layer),
            Rgb332(color),
            Uint16(thickness)
        ))

    def draw_rectangle(self, x1: int, y1: int, x2: int, y2: int,
                       layer: int, color: tuple[int, int, int] = (0, 0, 0), thickness: int = 1) -> None:
        self.send_command(DrawLine(
            Uint16(x1),
            Uint16(y1),
            Uint16(x2),
            Uint16(y2),
            Uint8(layer),
            Rgb332(color),
            Uint16(thickness)
        ))

    def draw_circle(self, x: int, y: int, radius: int,
                    layer: int, color: tuple[int, int, int] = (0, 0, 0), thickness: int = 1) -> None:
        self.send_command(DrawCircle(
            Uint16(x),
            Uint16(y),
            Uint16(radius),
            Uint8(layer),
            Rgb332(color),
            Uint16(thickness)
        ))

    def send_command(self, cmd: Command) -> None:
        self.sock.sendto(cobs_encode(cmd.to_bytes()), (self.ip, self.port))
