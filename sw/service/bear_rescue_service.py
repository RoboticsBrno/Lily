#!/usr/bin/env python3
import os
import signal
import subprocess
import sys
import threading
from pathlib import Path
import time


def log(tag: str, line: str):
    print(time.strftime('%Y-%m-%d %H:%M:%S'), f"[{tag}]", line)


try:
    from gpiozero import LED, Button
except ImportError:
    log("service", "Error: gpiozero is not installed. Install it with: sudo apt install python3-gpiozero")
    sys.exit(1)


LED_PIN = 18
BUTTON_PIN = 3

LOGIC_DIR = Path(__file__).parent / ".." / "logic"

BEAR_RESCUE_CMD = [
    sys.executable,
    "start.py",
    "--target", "bear_rescue",
    "--transport", "serial",
    "--device", "/dev/ttyUSB0",
    "--recording-path", "/home/pi/recordings/bear_rescue.csv",
]


class BearRescueService:
    def __init__(self):
        self.process = None
        self.restarting = False
        self.shutdown = threading.Event()
        self.restart_event = threading.Event()

    def _reader(self, proc):
        prefix = "bear_rescue"
        try:
            for line in proc.stdout:
                if self.shutdown.is_set():
                    break
                text = line.decode("utf-8", errors="replace").rstrip()
                if text:
                    log(prefix, text)
        except Exception as exc:
            log("service", f"Log reader error: {exc}")

    def _start(self):
        log("service", "Starting Bear Rescue...")
        self.process = subprocess.Popen(
            BEAR_RESCUE_CMD,
            cwd=LOGIC_DIR,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            preexec_fn=os.setsid,
        )
        t = threading.Thread(target=self._reader, args=(self.process,), daemon=True)
        t.start()

    def _stop(self):
        if self.process is None or self.process.poll() is not None:
            return

        log("service", "Stopping Bear Rescue...")
        try:
            pgid = os.getpgid(self.process.pid)
            os.killpg(pgid, signal.SIGINT)
        except ProcessLookupError:
            pass

        try:
            self.process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            log("service", "Bear Rescue did not stop gracefully, killing...")
            try:
                pgid = os.getpgid(self.process.pid)
                os.killpg(pgid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            self.process.kill()
            self.process.wait()

        self.process = None

    def _on_pressed(self):
        if self.restarting:
            return
        self.restarting = True
        self.restart_event.set()

    def run(self):
        led = LED(LED_PIN)
        button = Button(BUTTON_PIN, pull_up=True, bounce_time=0.1)
        button.when_pressed = self._on_pressed

        led.blink(on_time=1, off_time=1, background=True)

        self._start()

        try:
            while not self.shutdown.is_set():
                if self.restart_event.wait(timeout=1):
                    self.restart_event.clear()
                    self._stop()
                    self._start()
                    self.restarting = False
        except KeyboardInterrupt:
            log("service", "Interrupted by user.")
        finally:
            self.shutdown.set()
            self._stop()
            led.off()
            button.close()
            led.close()
            log("service", "Shut down.")


if __name__ == "__main__":
    service = BearRescueService()
    service.run()
