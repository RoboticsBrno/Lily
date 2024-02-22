from math import pi

from geometry import Point, Vector
from typing import Optional, Protocol


class World(Protocol):
    def get_pos(self) -> Point: ...
    def get_heading(self) -> Vector: ...
    def raycast(self, origin: Point, direction: Vector) -> Optional[tuple[Point, float, float]]: ...


class Link(Protocol):
    def read_all(self) -> bytes: ...
    def write(self, data: bytes) -> int: ...


class RpLidarMock:
    def __init__(self, world: World, port: Link) -> None:
        self.port = port
        self.world = world
        self.rotpos = 0.       # the current rotation position
        self.rps = 1          # rotations per second
        self.samples = 34.    # samples per rotation

        self._last_sample = 0.  # the angle of the last sample
        self._scanning = False
        self._first_sample = True
        self._rx_buffer = b''
        self._new_round = False

    def rx(self) -> None:
        data = self.port.read_all()
        self._rx_buffer += data

        done_cmd = True
        while done_cmd and len(self._rx_buffer) != 0:
            begin_pos = self._rx_buffer.find(b'\xa5')  # start of packet
            if begin_pos == -1:
                print("LIDAR: dropped", len(self._rx_buffer), "bytes", self._rx_buffer)
                self._rx_buffer = b''
                return
            print("LIDAR: dropped", begin_pos, "bytes", self._rx_buffer[:begin_pos])
            self._rx_buffer = self._rx_buffer[begin_pos:]

            if len(self._rx_buffer) < 2:
                return

            cmd = self._rx_buffer[1]
            to_slice = 0

            if cmd == 0x25:  # STOP request
                print("LIDAR: STOP request")
                self._scanning = False
                to_slice = 2

            elif cmd == 0x40:  # Core reset
                print("LIDAR: Core reset")
                self._scanning = False
                to_slice = 2

            elif cmd == 0x20:  # Start scan
                print("LIDAR: Start scan")
                self._scanning = True
                self._first_sample = True
                self._last_sample = self.rotpos
                to_slice = 2

            else:
                done_cmd = False

            self._rx_buffer = self._rx_buffer[to_slice:]

    def tx(self, angle: float, distance: float, quality: int = 63) -> None:
        # TODO: clamp the angles earlier
        while angle < 0:
            angle += 2 * pi
        while angle >= 2 * pi:
            angle -= 2 * pi

        if not (0 <= quality <= 63):
            print("LIDAR: quality out of range (" + str(quality) + "), setting to 63")
            quality = 63

        data = bytearray()
        if self._first_sample:
            data.append(0xa5)
            data.append(0x5a)

        angle_i = int(angle * 180 / pi * 64)
        distance = int(distance * 4)
        if distance > 0xffff:
            print("LIDAR: distance out of range (" + str(distance) + "), ignoring")
            return

        print("LIDAR: sending", angle, distance, quality)

        sNs = 0b01 if self._new_round else 0b10
        data.append((quality << 2) | sNs)

        data.append(((angle_i & 0x7f) << 1) | 0b1)
        data.append(angle_i >> 7 & 0xff)

        data.append(distance & 0xff)
        data.append(distance >> 8)

        self.port.write(data)

    def update(self, dt: float) -> None:
        pos = self.world.get_pos()
        heading = self.world.get_heading()

        self.rotpos += dt * self.rps * 2 * pi

        if not self._scanning:
            self._last_sample = self.rotpos
            self.rx()
            return

        while self._last_sample < self.rotpos:
            angle = self._last_sample
            direction = heading.rotate(angle)
            hit = self.world.raycast(pos, direction)
            if hit is not None:
                _, distance, refl_angle = hit
                self.tx(direction.angle(), distance, 63)
            else:
                self.tx(direction.angle(), 0)

            self._first_sample = False
            self._last_sample = angle + 2 * pi / self.samples

        self.rx()
