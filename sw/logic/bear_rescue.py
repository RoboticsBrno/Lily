from __future__ import annotations

from control.game import GameStateMachine
from control.pure_pursuit import PurePursuitConfig, PurePursuitController
from params import PP_LOOKAHEAD_DISTANCE, PP_STEERING_GAIN, ROBOT_BODY_RADIUS
from util.init_common import create_default_bear
from util.launcher import TargetProgram
from util.vis_common import (
    draw_bear,
    draw_bear_detection,
    draw_candidate_points,
    draw_estimated_pose,
    draw_lidar_history,
    draw_particles,
    draw_path,
)


class BearRescueTarget(TargetProgram):
    def __init__(self) -> None:
        self.game: GameStateMachine | None = None

    @property
    def needs_keyboard(self) -> bool:
        return False

    def setup(self, controller, localization, keyboard, sim_server, visualizer):
        self.game = GameStateMachine(
            pursuit=PurePursuitController(
                PurePursuitConfig(
                    lookahead_distance=PP_LOOKAHEAD_DISTANCE,
                    steering_gain=PP_STEERING_GAIN,
                )
            ),
            controller=controller,
            localization=localization,
        )

    def on_measurements(self, measurements, controller, localization):
        assert self.game is not None

        localization.on_measurements(measurements)
        self.game.loop()
        estimated = localization.localizer.get_estimate()
        print(
            f"State: {self.game.state}, "
            f"Estimated: {estimated.x:.3f}, {estimated.y:.3f}, {estimated.yaw:.2f}"
        )

    def on_ui_tick(self, dt_seconds, visualizer, localization, sim_server):
        estimated = localization.localizer.get_estimate()
        bear_detection = localization.bear_detector.get_estimate()
        visualizer.draw(localization.world, color=(224, 228, 236))
        draw_bear(visualizer, create_default_bear())
        draw_path(visualizer, self.game.planned_path)
        draw_estimated_pose(visualizer, estimated, ROBOT_BODY_RADIUS)
        draw_bear_detection(visualizer, bear_detection, estimated)
        draw_particles(visualizer, localization.localizer)
        draw_lidar_history(visualizer, localization.lidar_history, show_magnitude=False)
        draw_candidate_points(visualizer, localization.bear_detector)

    def on_ui_event(self, event, visualizer, keyboard, sim_server):
        pass
