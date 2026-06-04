from __future__ import annotations

import argparse
from typing import TYPE_CHECKING

from util.launcher import run


if TYPE_CHECKING:
    from bear_rescue import BearRescueTarget
    from remote_control import RemoteControlTarget


def get_bear_rescue_target() -> type[BearRescueTarget]:
    from bear_rescue import BearRescueTarget
    return BearRescueTarget


def get_remote_control_target() -> type[RemoteControlTarget]:
    from remote_control import RemoteControlTarget
    return RemoteControlTarget


TARGETS = {
    "bear_rescue": get_bear_rescue_target,
    "remote_control": get_remote_control_target,
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run a target program with configurable transport and optional sim/vis."
    )
    parser.add_argument(
        "--target", required=True, choices=list(TARGETS.keys()), help="Target program to run"
    )
    parser.add_argument(
        "--transport", required=True, choices=["serial", "udp", "replay"], help="Transport type"
    )
    parser.add_argument("--sim", action="store_true", help="Enable simulator")
    parser.add_argument("--vis", action="store_true", help="Enable visualizer")
    parser.add_argument(
        "--recording-path", default=None, help="Path for recording (disabled if not set)"
    )
    parser.add_argument("--device", default="/dev/ttyUSB0", help="Serial device (serial only)")
    parser.add_argument("--host", default="127.0.0.1", help="UDP host (udp only)")
    parser.add_argument(
        "--map", default="data/map_bear_rescue.json", help="Path to map JSON"
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    target_class = TARGETS[args.target]()

    kwargs = {
        "target_class": target_class,
        "transport": args.transport,
        "use_sim": args.sim,
        "use_vis": args.vis,
        "map_path": args.map,
    }
    if args.recording_path is not None:
        kwargs["recording_path"] = args.recording_path
    if args.transport == "serial":
        kwargs["device"] = args.device
    elif args.transport == "udp":
        kwargs["host"] = args.host

    run(**kwargs)


if __name__ == "__main__":
    main()
