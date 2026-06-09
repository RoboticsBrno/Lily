from __future__ import annotations

import time

from control.game import GameStateMachine, PursuitState
from control.pure_pursuit import PurePursuitConfig, PurePursuitController
from params import GAME_START_LED_PIN, GAME_START_BUTTON_PIN, PP_LOOKAHEAD_DISTANCE, PP_STEERING_GAIN, PP_DERIVATIVE_GAIN, ROBOT_BODY_RADIUS
from util.init_common import create_default_bear
from util.launcher import TargetProgram

try:
    from gpiozero import LED, Button
except ImportError:
    LED = None
    Button = None


class BearRescueTarget(TargetProgram):
    def __init__(self) -> None:
        self.game: GameStateMachine | None = None
        self._start_led = None
        self._start_button = None
        self._led_blink_next = 0.0
        self._resizing_bear = False

    @property
    def needs_keyboard(self) -> bool:
        return False

    def setup(self, controller, localization, keyboard, sim_server, visualizer):
        self.game = GameStateMachine(
            pursuit=PurePursuitController(
                PurePursuitConfig(
                    lookahead_distance=PP_LOOKAHEAD_DISTANCE,
                    steering_gain=PP_STEERING_GAIN,
                    derivative_gain=PP_DERIVATIVE_GAIN,
                )
            ),
            controller=controller,
            localization=localization,
        )
        if LED is not None and Button is not None:
            try:
                self._start_led = LED(GAME_START_LED_PIN)
                self._start_button = Button(GAME_START_BUTTON_PIN, pull_up=True, bounce_time=0.1)
                self._start_button.when_pressed = self._trigger_start
            except KeyboardInterrupt:
                raise
            except Exception:
                import traceback
                traceback.print_exc()
                self._start_led = None
                self._start_button = None

    def _trigger_start(self) -> None:
        if self.game is None:
            return
        self.game.request_start()

    def on_measurements(self, measurements, controller, localization):
        assert self.game is not None

        localization.on_measurements(measurements)

        self.game.loop()

        if self._start_led is not None:
            now = time.time()
            if now >= self._led_blink_next:
                self._led_blink_next = now + 0.5
                if self._start_led.is_lit:
                    self._start_led.off()
                else:
                    self._start_led.on()

        estimated = localization.localizer.get_estimate()
        print(
            f"State: {self.game.state}, "
            f"Estimated: {estimated.x:.3f}, {estimated.y:.3f}, {estimated.yaw:.2f}"
        )

    def on_ui_tick(self, dt_seconds, visualizer, localization, sim_server):
        from util.vis_common import (
            draw_bear,
            draw_bear_detection,
            draw_candidate_points,
            draw_estimated_pose,
            draw_lidar_history,
            draw_particles,
            draw_path,
        )

        estimated = localization.localizer.get_estimate()
        bear_detection = localization.bear_detector.get_estimate()
        visualizer.draw(localization.world, color=(224, 228, 236))
        bear = sim_server.bear if sim_server is not None else create_default_bear()
        draw_bear(visualizer, bear)
        draw_path(visualizer, self.game.planned_path)
        draw_estimated_pose(visualizer, estimated, ROBOT_BODY_RADIUS)
        draw_bear_detection(visualizer, bear_detection, estimated)
        draw_particles(visualizer, localization.localizer)
        draw_lidar_history(visualizer, localization.lidar_history, show_magnitude=False)
        draw_candidate_points(visualizer, localization.bear_detector)

    def on_ui_event(self, event, visualizer, keyboard, sim_server):
        from util.vis_common import handle_ui_control_event

        bear = sim_server.bear if sim_server is not None else None
        if bear is not None:
            self._resizing_bear = handle_ui_control_event(
                event, visualizer, bear, self._resizing_bear
            )
        if self.game is not None and self.game.state == PursuitState.INIT:
            if self._start_button is None:
                try:
                    import pygame
                except ImportError:
                    return
                if event.type == pygame.KEYDOWN and event.key == pygame.K_SPACE:
                    self._trigger_start()
