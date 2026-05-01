from dataclasses import dataclass
from math import atan2, hypot, pi

from geometry.shapes import Point
from geometry.transforms import Pose
from geometry.util import wrap_angle


@dataclass(frozen=True)
class PurePursuitConfig:
    lookahead_distance: float
    steering_gain: float = 4


@dataclass(frozen=True)
class PurePursuitOutput:
    left_power: float
    right_power: float


class PurePursuitController:
    def __init__(self, config: PurePursuitConfig):
        self.config = config
        self.path: list[Point] = []
        self.allow_reverse = True
        self.current_segment = 0
        self._step_t = 0.0
        self._step_distance = 0.05

    def update_plan(self, path: list[Point], allow_reverse: bool) -> None:
        self.path = list(path)
        self.allow_reverse = allow_reverse
        self.current_segment = 0
        self._step_t = 0.0

    def compute(self, pose: Pose) -> PurePursuitOutput:
        if not self.path:
            return PurePursuitOutput(0.0, 0.0)

        lookahead_point = self._find_lookahead_point(pose)
        if lookahead_point is None:
            return PurePursuitOutput(0.0, 0.0)

        forward_angle = self._compute_steering_angle(pose, lookahead_point)
        reverse_angle = wrap_angle(forward_angle + pi)

        if abs(reverse_angle) < abs(forward_angle) and self.allow_reverse:
            return self._compute_wheel_powers(reverse_angle, drive_direction=-1.0)

        return self._compute_wheel_powers(forward_angle, drive_direction=1.0)

    def _find_lookahead_point(self, pose: Pose) -> Point | None:
        if not self.path:
            return None

        if len(self.path) == 1:
            return self.path[0]

        lookahead = self.config.lookahead_distance

        # Advance the active segment index once we are sufficiently close to the
        # next waypoint so the controller keeps moving forward on the path.
        previous_segment = self.current_segment
        while self.current_segment < len(self.path) - 2:
            next_waypoint = self.path[self.current_segment + 1]
            distance_to_next = hypot(next_waypoint.x - pose.x, next_waypoint.y - pose.y)
            if distance_to_next > lookahead:
                break
            self.current_segment += 1

        if previous_segment != self.current_segment:
            self._step_t = 0.0

        for segment_index in range(self.current_segment, len(self.path) - 1):
            start = self.path[segment_index]
            end = self.path[segment_index + 1]

            seg_dx = end.x - start.x
            seg_dy = end.y - start.y
            seg_len = hypot(seg_dx, seg_dy)
            if seg_len == 0.0:
                continue

            start_t = self._step_t if segment_index == self.current_segment else 0.0
            step_t = min(1.0, self._step_distance / seg_len)
            t = start_t

            while t <= 1.0:
                sample = Point(x=start.x + t * seg_dx, y=start.y + t * seg_dy)
                distance = hypot(sample.x - pose.x, sample.y - pose.y)
                if distance >= lookahead:
                    self._step_t = t
                    self.current_segment = segment_index
                    return sample
                t += step_t

            self.current_segment = segment_index + 1
            self._step_t = 0.0

        # If no waypoint is outside the lookahead radius, track the final target.
        self.current_segment = max(0, len(self.path) - 2)
        self._step_t = 1.0
        return self.path[-1]

    def _compute_steering_angle(self, pose: Pose, lookahead_point: Point) -> float:
        dx = lookahead_point.x - pose.x
        dy = lookahead_point.y - pose.y
        target_heading = atan2(dy, dx)
        return wrap_angle(target_heading - pose.yaw)

    def _compute_wheel_powers(
        self,
        steering_angle: float,
        drive_direction: float = 1.0,
    ) -> PurePursuitOutput:
        base_power = 1.0
        steering = (steering_angle / pi) * self.config.steering_gain
        turn = max(-1.0, min(1.0, steering))

        linear_speed = base_power * drive_direction
        angular_speed = base_power * turn

        left_speed = linear_speed - angular_speed
        right_speed = linear_speed + angular_speed

        scale = max(abs(left_speed), abs(right_speed))
        left_speed /= scale
        right_speed /= scale

        return PurePursuitOutput(left_power=left_speed, right_power=right_speed)
