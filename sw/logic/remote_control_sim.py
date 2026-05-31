from __future__ import annotations

from math import cos, sin
from pathlib import Path

from comm.udp_transport import UdpTransport
from geometry.shapes import Line, Point
from comm.messages import SubscribeCommand
from params import ROBOT_BODY_RADIUS
from util.init_common import (
    CONTROLLER_RECEIVE_PORT,
    SIM_PORT,
    TARGET_FPS,
    build_controller,
    connect_keyboard_ctrl,
    build_localization_stack,
    make_default_sim,
    resolve_map_path,
)
from util.vis_common import (
    draw_bear,
    draw_bear_detection,
    draw_candidate_points,
    draw_estimated_pose,
    draw_lidar_history,
    draw_particles,
    handle_robot_control_event,
    handle_ui_control_event,
)
from util.visualizer import Visualizer


def main() -> None:
    repo_root = Path(__file__).resolve().parent
    map_path = resolve_map_path(repo_root, "data/map_bear_rescue.json")

    server, bear = make_default_sim(map_path)

    visualizer = Visualizer(title="Keyboard Controller")

    localization = build_localization_stack(map_path, server.robot.pose)
    controller = build_controller(
        UdpTransport(
            host="127.0.0.1",
            port=SIM_PORT,
            receive_port=CONTROLLER_RECEIVE_PORT,
        )
    )
    keyboard = connect_keyboard_ctrl(controller)
    controller.set_measurement_callback(localization.on_measurements)

    resizing_bear_with_right_drag = False

    def on_event(event) -> None:
        nonlocal resizing_bear_with_right_drag
        resizing_bear_with_right_drag = handle_ui_control_event(
            event,
            visualizer,
            bear,
            resizing_bear_with_right_drag,
        )
        handle_robot_control_event(
            event,
            keyboard,
        )

    def on_ui_tick(dt_seconds: float) -> None:
        _ = dt_seconds

        truth = server.get_true_pose()
        estimated = localization.localizer.get_estimate()
        bear_detection = localization.bear_detector.get_estimate()

        visualizer.draw(localization.world, color=(224, 228, 236))
        draw_bear(visualizer, bear)

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

        draw_estimated_pose(visualizer, estimated, ROBOT_BODY_RADIUS)
        draw_bear_detection(visualizer, bear_detection, estimated)
        draw_particles(visualizer, localization.localizer)
        draw_lidar_history(visualizer, localization.lidar_history, show_magnitude=False)
        draw_candidate_points(visualizer, localization.bear_detector)

    visualizer.on_tick = on_ui_tick
    visualizer.on_event = on_event

    visualizer.init()
    server.start()
    controller.start()
    controller.send_command(SubscribeCommand())
    try:
        visualizer.run(target_fps=max(1, TARGET_FPS))
    finally:
        controller.stop()


if __name__ == "__main__":
    main()
