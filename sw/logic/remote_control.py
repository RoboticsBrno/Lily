from __future__ import annotations

from collections import deque
from pathlib import Path

from comm.serial_transport import SerialTransport
from geometry.transforms import Pose
from util.remote_control_common import (
    LIDAR_HISTORY,
    TARGET_FPS,
    build_keyboard_controller,
    build_localization_stack,
    build_replay_player,
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


ROBOT_BODY_RADIUS = 0.1


def main() -> None:
    repo_root = Path(__file__).resolve().parent
    map_path = resolve_map_path(repo_root, "data/map_bear_rescue.json")

    bear = create_default_bear()

    world, localizer, bear_detector = build_localization_stack(map_path, Pose(0.7, 2, 0.0))
    controller = build_keyboard_controller(SerialTransport(device="/dev/ttyUSB0", baud_rate=921600))
    # controller = build_replay_player("recording_bear.csv", speed=1.0)

    visualizer = Visualizer(
        title="Keyboard Controller",
    )

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

        # truth = server.get_true_pose()
        estimated, bear_detection = process_measurements(controller, localizer, bear_detector, lidar_history)

        # Visualization
        visualizer.draw(world, color=(224, 228, 236))
        draw_bear(visualizer, bear)
        # print(f"Estimated pose: {estimated.x:.2f}, {estimated.y:.2f}, {estimated.yaw:.2f}")

        draw_estimated_pose(visualizer, estimated, ROBOT_BODY_RADIUS)
        draw_bear_detection(visualizer, bear_detection, estimated)
        draw_particles(visualizer, localizer)
        draw_lidar_history(visualizer, lidar_history, show_magnitude=False)
        draw_candidate_points(visualizer, bear_detector)

    visualizer.on_tick = on_tick
    visualizer.on_event = on_event

    # with server:
    controller.start()
    try:
        visualizer.run(target_fps=max(1, TARGET_FPS))
    finally:
        controller.stop()


if __name__ == "__main__":
    main()
