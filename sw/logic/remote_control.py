from __future__ import annotations

from params import ROBOT_BODY_RADIUS
from util.init_common import create_default_bear
from util.launcher import TargetProgram


class RemoteControlTarget(TargetProgram):
    def __init__(self) -> None:
        self._resizing_bear = False

    @property
    def needs_keyboard(self) -> bool:
        return True

    def setup(self, controller, localization, keyboard, sim_server, visualizer):
        pass

    def on_measurements(self, measurements, controller, localization):
        localization.on_measurements(measurements)

    def on_ui_tick(self, dt_seconds, visualizer, localization, sim_server):
        from util.vis_common import (
            draw_bear,
            draw_bear_detection,
            draw_candidate_points,
            draw_estimated_pose,
            draw_lidar_history,
            draw_particles,
        )

        estimated = localization.localizer.get_estimate()
        bear_detection = localization.bear_detector.get_estimate()
        visualizer.draw(localization.world, color=(224, 228, 236))
        draw_bear(visualizer, create_default_bear())
        draw_estimated_pose(visualizer, estimated, ROBOT_BODY_RADIUS)
        draw_bear_detection(visualizer, bear_detection, estimated)
        draw_particles(visualizer, localization.localizer)
        draw_lidar_history(visualizer, localization.lidar_history, show_magnitude=False)
        draw_candidate_points(visualizer, localization.bear_detector)

    def on_ui_event(self, event, visualizer, keyboard, sim_server):
        from util.vis_common import handle_ui_control_event, handle_robot_control_event

        self._resizing_bear = handle_ui_control_event(
            event, visualizer, create_default_bear(), self._resizing_bear
        )
        if keyboard is not None:
            handle_robot_control_event(event, keyboard)
