import socket
from protocol import cmd_from_bytes
from visualizer import Viz
from cobs import CobsStreamDecoder
from threading import Thread
from cmd_proc import cmd_proc


visualizer = Viz(640, 480)
stop = False


def sock() -> None:
    global stop

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("", 1234))
    sock.settimeout(1)

    decoder = CobsStreamDecoder()

    while not stop:
        try:
            data, addr = sock.recvfrom(1024)
        except socket.timeout:
            continue

        print("received message from", addr)
        print("data:", data)

        for byte in data:
            cmd_data = decoder.receive(byte)
            if cmd_data:
                cmd = cmd_from_bytes(cmd_data)
                print("cmd:", cmd)
                visualizer._events.append(cmd_proc(cmd))


def main() -> None:
    global stop

    t = Thread(target=sock)
    t.start()

    visualizer.run()

    stop = True
    t.join()


if __name__ == "__main__":
    main()
