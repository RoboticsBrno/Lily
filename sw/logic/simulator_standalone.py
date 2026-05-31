from __future__ import annotations

from math import cos, sin
from pathlib import Path

from geometry.shapes import Line, Point
from params import ROBOT_BODY_RADIUS
from util.init_common import (
    SIM_PORT,
    TARGET_FPS,
    make_default_sim,
    resolve_map_path,
)
from util.vis_common import (
    draw_bear,
    handle_ui_control_event,
)
from util.visualizer import Visualizer


def main() -> None:
    repo_root = Path(__file__).resolve().parent
    map_path = resolve_map_path(repo_root, "data/map_bear_rescue.json")

    server, bear = make_default_sim(map_path)

    visualizer = Visualizer(title="Simulator")

    resizing_bear_with_right_drag = False

    def on_event(event) -> None:
        nonlocal resizing_bear_with_right_drag
        resizing_bear_with_right_drag = handle_ui_control_event(
            event,
            visualizer,
            bear,
            resizing_bear_with_right_drag,
        )

    def on_ui_tick(dt_seconds: float) -> None:
        _ = dt_seconds

        truth = server.get_true_pose()
        draw_bear(visualizer, bear)
        visualizer.draw(server.world, color=(224, 228, 236))

        heading_length = ROBOT_BODY_RADIUS
        robot_center = Point(truth.x, truth.y)
        heading_tip = Point(
            truth.x + heading_length * cos(truth.yaw),
            truth.y + heading_length * sin(truth.yaw),
        )
        visualizer.draw(server.robot.get_body_circle(), color=(255, 120, 90), width_px=2)
        visualizer.draw(Line(a=robot_center, b=heading_tip), color=(255, 200, 90), width_px=2)
        left_claw, right_claw = server.robot.get_claw_segments()
        visualizer.draw(left_claw, color=(255, 150, 120), width_px=2)
        visualizer.draw(right_claw, color=(255, 150, 120), width_px=2)
        visualizer.draw(robot_center, color=(255, 255, 255), point_radius_px=2)

    visualizer.on_tick = on_ui_tick
    visualizer.on_event = on_event

    visualizer.init()
    server.start()
    visualizer.run(target_fps=max(1, TARGET_FPS))


if __name__ == "__main__":
    main()
