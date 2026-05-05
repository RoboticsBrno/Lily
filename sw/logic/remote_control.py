from __future__ import annotations

from pathlib import Path

from comm.serial_transport import SerialTransport
from geometry.transforms import Pose
from comm.messages import SubscribeCommand
from util.init_common import (
    TARGET_FPS,
    build_controller,
    connect_keyboard_ctrl,
    build_localization_stack,
    # build_replay_player,
    create_default_bear,
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


ROBOT_BODY_RADIUS = 0.1


def main() -> None:
    repo_root = Path(__file__).resolve().parent
    map_path = resolve_map_path(repo_root, "data/map_bear_rescue.json")

    bear = create_default_bear()

    localization = build_localization_stack(map_path, Pose(0.7, 2, 0.0))
    controller = build_controller(SerialTransport(device="/dev/ttyUSB0", baud_rate=921600))
    # controller = build_replay_player("recording_bear.csv", speed=1.0)
    keyboard = connect_keyboard_ctrl(controller)
    controller.set_measurement_callback(localization.on_measurements)

    visualizer = Visualizer(title="Keyboard Controller")

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
        controller.send_command(SubscribeCommand())

    def on_ui_tick(dt_seconds: float) -> None:
        _ = dt_seconds

        estimated = localization.localizer.estimate_pose()
        bear_detection = localization.bear_detector.get_estimate()

        # Visualization
        visualizer.draw(localization.world, color=(224, 228, 236))
        draw_bear(visualizer, bear)

        draw_estimated_pose(visualizer, estimated, ROBOT_BODY_RADIUS)
        draw_bear_detection(visualizer, bear_detection, estimated)
        draw_particles(visualizer, localization.localizer)
        draw_lidar_history(visualizer, localization.lidar_history, show_magnitude=False)
        draw_candidate_points(visualizer, localization.bear_detector)

    visualizer.on_tick = on_ui_tick
    visualizer.on_event = on_event

    visualizer.init()
    controller.start()
    controller.send_command(SubscribeCommand())
    try:
        visualizer.run(target_fps=max(1, TARGET_FPS))
    finally:
        controller.stop()


if __name__ == "__main__":
    main()
