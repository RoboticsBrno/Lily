from math import floor
from cobs import CobsStreamDecoder, cobs_encode
from simtypes import Link, World


TicksLeft = 0
TicksRight = 1
MotorLeft = 10
MotorRight = 11


class MotorsEncodersMock:
    def __init__(self, world: World, link: Link) -> None:
        self.world = world
        self.link = link

        self._decoder = CobsStreamDecoder()

        self.powerLeft = 100.
        self.powerRight = 50.
        self.positionLeft = 0.
        self.positionRight = 0.

    def _handle_packet(self, packet: bytes) -> None:
        if packet[0] != MotorLeft and packet[0] != MotorRight:
            print("MOTORENC: invalid motor ID")
            return

        if len(packet) != 3:
            print("MOTORENC: invalid motor length")
            return

        power = int.from_bytes(packet[1:3], "little", signed=True)
        if packet[0] == MotorLeft:
            print("MOTORENC: left power", power)
            self.powerLeft = power
        else:
            print("MOTORENC: right power", power)
            self.powerRight = power

    def _rx(self) -> None:
        data = self.link.read_all()

        for byte in data:
            packet = self._decoder.receive(byte)
            if packet is not None:
                print("MOTORENC: packet", packet)
                self._handle_packet(packet)

    def _tx(self, encoder: int, deltaPos: int) -> None:
        if deltaPos < -128 or deltaPos > 127:
            print("MOTORENC: deltaPos out of range")
            return

        print("MOTORENC: tx", encoder, deltaPos)
        dp = deltaPos.to_bytes(1, "little", signed=True)

        packet = bytes([encoder, dp[0]])
        self.link.write(cobs_encode(packet))

    def update(self, dt: float) -> None:
        oldPosLeft = floor(self.positionLeft)
        oldPosRight = floor(self.positionRight)

        self.positionLeft += self.powerLeft * dt
        self.positionRight += self.powerRight * dt

        self._tx(0, floor(self.positionLeft) - oldPosLeft)
        self._tx(1, floor(self.positionRight) - oldPosRight)

        self._rx()
