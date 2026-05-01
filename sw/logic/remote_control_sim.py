from __future__ import annotations

from collections import deque
from math import cos, pi, sin
from pathlib import Path

from comm.udp_transport import UdpTransport
from geometry.shapes import Line, Point
from sim.robot import RobotConfig
from sim.sensors import LidarSensorConfig, MotorConfig
from sim.server import create_server_from_map
from util.remote_control_common import (
    CONTROLLER_RECEIVE_PORT,
    LIDAR_HISTORY,
    LIDAR_HZ,
    LIDAR_MAX_DISTANCE,
    LIDAR_SAMPLE,
    SIM_PORT,
    TARGET_FPS,
    WHEEL_BASE,
    build_keyboard_controller,
    build_localization_stack,
    create_default_bear,
    draw_bear,
    draw_bear_detection,
    draw_candidate_points,
    draw_estimated_pose,
    draw_lidar_history,
    draw_particles,
    handle_robot_control_event,
    handle_ui_control_event,
    process_measurements,
    resolve_map_path,
)
from util.visualizer import Visualizer


ROBOT_BODY_RADIUS = 0.125


def main() -> None:
    repo_root = Path(__file__).resolve().parent
    map_path = resolve_map_path(repo_root, "data/map_bear_rescue.json")

    bear = create_default_bear()

    server = create_server_from_map(
        map_path=map_path,
        robot_config=RobotConfig(wheel_base=WHEEL_BASE),
        lidar_config=LidarSensorConfig(
            rotation_speed_hz=LIDAR_HZ,
            measurement_frequency_hz=LIDAR_SAMPLE,
            max_distance=LIDAR_MAX_DISTANCE,
            world_max_incidence_angle=pi / 4,
            bear_max_incidence_angle=pi / 4,
            distance_noise_stddev=0.003,
            angle_noise_stddev=0.001,
            random_distance_probability=0.1,
        ),
        motor_config=MotorConfig(
            ticks_per_meter=1000.0,
            max_speed=0.7,
        ),
        bear=bear,
        transport=UdpTransport(
            host="127.0.0.1",
            port=CONTROLLER_RECEIVE_PORT,
            receive_port=SIM_PORT,
        ),
        publish_hz=30.0,
    )

    controller = build_keyboard_controller(
        UdpTransport(
            host="127.0.0.1",
            port=SIM_PORT,
            receive_port=CONTROLLER_RECEIVE_PORT,
        )
    )

    visualizer = Visualizer(
        title="Keyboard Controller",
    )

    world, localizer, bear_detector = build_localization_stack(map_path, server.robot.pose)

    lidar_history: deque = deque(maxlen=LIDAR_HISTORY)
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
            controller,
        )

    def on_tick(dt_seconds: float) -> None:
        _ = dt_seconds

        truth = server.get_true_pose()
        estimated, bear_detection = process_measurements(controller, localizer, bear_detector, lidar_history)

        # Visualization

        visualizer.draw(world, color=(224, 228, 236))
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
        draw_particles(visualizer, localizer)
        draw_lidar_history(visualizer, lidar_history, show_magnitude=False)
        draw_candidate_points(visualizer, bear_detector)

    visualizer.on_tick = on_tick
    visualizer.on_event = on_event

    server.start()
    controller.start()
    try:
        visualizer.run(target_fps=max(1, TARGET_FPS))
    finally:
        controller.stop()


if __name__ == "__main__":
    main()
