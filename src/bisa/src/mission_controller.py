"""Mission FSM and normalized throttle/steering controller for D-Racer."""

from __future__ import annotations

import math
from dataclasses import dataclass

from .dracer_config import AutonomousConfig
from .lane_perception import LaneObs, clamp
from .object_detector import DetectionBuffer


def fresh_light_state(
    snapshot: tuple[str | None, int, float], now_sec: float, max_age_sec: float
) -> tuple[str | None, int]:
    """Drops a light verdict when it is absent, future-dated, or too old."""

    state, sequence, stamp = snapshot
    age = float(now_sec) - float(stamp)
    if state in ("green", "red") and 0.0 <= age <= max(float(max_age_sec), 0.0):
        return state, int(sequence)
    return None, int(sequence)


@dataclass
class ControlCmd:
    """Carries normalized control values published to /control."""

    throttle: float
    steering: float


class LaneController:
    """Turns LaneObs values into smooth steering and throttle commands."""

    def __init__(self, config: AutonomousConfig):
        """Initializes previous values used by rate limiting."""

        self.config = config
        self.prev_steer = 0.0
        self.prev_throttle = 0.0

    def rate_limit(self, target: float, previous: float, max_delta: float) -> float:
        """Limits one control update to prevent sudden steering jumps."""

        return previous + clamp(target - previous, -max_delta, max_delta)

    def steering_from_lane(
        self,
        lane: LaneObs,
        steer_limit: float = 0.80,
        curve_scale: float = 1.0,
    ) -> float:
        """Computes pure-pursuit lane steering.

        The aim point sits ``lookahead_m`` ahead, laterally offset by the lane
        center error (shifted along the road bend by ``curve_blend``). The
        bicycle-model steering angle toward that point is normalized into the
        [-1, 1] command range by ``max_steer_deg``.
        """

        cfg = self.config.steering
        if not lane.valid:
            raw = self.prev_steer * cfg.lost_decay
        else:
            curve_scale = clamp(curve_scale, 0.0, 1.0)
            target_error = clamp(
                lane.center_error
                + cfg.curve_blend * curve_scale * lane.signed_curvature,
                -1.0,
                1.0,
            )
            lateral = target_error * cfg.lateral_scale_m
            curvature = clamp(lane.curvature * curve_scale, 0.0, 1.0)
            curve_strength = curvature ** max(cfg.curve_response_power, 1e-3)
            minimum = min(cfg.lookahead_m, cfg.curve_lookahead_min_m)
            lookahead = max(
                cfg.lookahead_m + curve_strength * (minimum - cfg.lookahead_m),
                1e-3,
            )
            alpha = math.atan2(lateral, lookahead)
            delta = math.atan2(2.0 * cfg.wheelbase_m * math.sin(alpha), lookahead)
            curve_gain = 1.0 + max(cfg.curve_steer_boost, 0.0) * curve_strength
            raw = (
                cfg.pp_gain
                * curve_gain
                * delta
                / math.radians(max(cfg.max_steer_deg, 1.0))
            )

        raw = clamp(raw, -steer_limit, steer_limit)
        steer = self.rate_limit(raw, self.prev_steer, cfg.rate_limit_per_cmd)
        self.prev_steer = steer
        return float(cfg.steer_sign) * steer

    def throttle_scheduler(
        self,
        section_cap: float,
        steer: float,
        curvature: float,
        section_min: float = None,
    ) -> float:
        """Schedules throttle with immediate braking and ramped acceleration."""

        speed_min = self.config.throttle.speed_min
        speed_max = self.config.throttle.speed_max
        # Squeeze the per-state cap into the [speed_min, speed_max] band so the
        # commanded throttle always maps into that band while moving.
        cap = clamp(section_cap, speed_min, speed_max)
        floor = speed_min if section_min is None else clamp(section_min, speed_min, cap)
        floor = min(floor, cap)
        steer_demand = max(
            abs(steer) - self.config.throttle.straight_steer_deadband,
            0.0,
        )
        curvature_demand = max(
            abs(curvature) - self.config.throttle.straight_curvature_deadband,
            0.0,
        )
        target = cap
        target -= self.config.throttle.steer_slowdown * steer_demand
        target -= self.config.throttle.curvature_slowdown * curvature_demand
        target = clamp(target, floor, cap)
        if target > self.prev_throttle:
            if self.prev_throttle <= 0.0:
                target = floor
            else:
                target = min(
                    target,
                    self.prev_throttle + self.config.throttle.ramp_up_per_cmd,
                )
        target = clamp(target, floor, cap)
        self.prev_throttle = target
        return target

    def lane_follow(
        self,
        lane: LaneObs,
        cap: float,
        steer_limit: float,
        section_min: float = None,
        curve_scale: float = 1.0,
    ) -> ControlCmd:
        """Follows the latest lane observation with cautious lost-lane behavior."""

        if not lane.valid:
            steer = self.rate_limit(
                self.prev_steer * self.config.steering.lost_decay,
                self.prev_steer,
                self.config.steering.rate_limit_per_cmd,
            )
            self.prev_steer = steer
            throttle = min(self.prev_throttle, self.config.throttle.speed_min)
            self.prev_throttle = throttle
            return ControlCmd(throttle, steer)

        steer = self.steering_from_lane(
            lane,
            steer_limit=steer_limit,
            curve_scale=curve_scale,
        )
        throttle = self.throttle_scheduler(cap, steer, lane.curvature, section_min=section_min)
        return ControlCmd(throttle, steer)

    def follow_with_startup(
        self, lane: LaneObs, cap: float, steer_limit: float
    ) -> ControlCmd:
        """Follows curvature while allowing a straight crawl off the start grid."""

        if lane.valid:
            return self.lane_follow(lane, cap, steer_limit)
        steer = self.rate_limit(
            self.prev_steer * self.config.steering.lost_decay,
            self.prev_steer,
            self.config.steering.rate_limit_per_cmd,
        )
        self.prev_steer = steer
        self.prev_throttle = max(
            self.prev_throttle, self.config.throttle.speed_min
        )
        return ControlCmd(self.prev_throttle, steer)


class BaseCourseFSM:
    """Shared helpers for the course state machines."""

    def __init__(self, config: AutonomousConfig, controller: LaneController):
        """Initializes shared timing and debounced mission observations."""

        self.config = config
        self.controller = controller
        self.state = ""
        self.enter_t = 0.0
        # Consecutive control ticks the traffic-light verdict has held green/red.
        self._green_streak = 0
        self._red_streak = 0
        self._last_light_seq = None

    def update_light(self, light_state, light_seq=None) -> None:
        """Counts consecutive verdicts from distinct detector frames only.

        The control loop runs faster than inference, so repeated control ticks
        often carry the same result. ``light_seq`` prevents one detector frame
        from being counted several times. A missing/stale verdict always clears
        both streaks even when its sequence number is unchanged.
        """

        if light_state not in ("green", "red"):
            self._green_streak = 0
            self._red_streak = 0
            return
        if light_seq is not None and light_seq == self._last_light_seq:
            return
        self._last_light_seq = light_seq
        self._green_streak = self._green_streak + 1 if light_state == "green" else 0
        self._red_streak = self._red_streak + 1 if light_state == "red" else 0

    def green_confirmed(self) -> bool:
        """True once green has held for ``light_confirm_frames`` ticks."""

        return self._green_streak >= self.config.detector.light_confirm_frames

    def red_confirmed(self) -> bool:
        """True once red has held for ``light_confirm_frames`` ticks."""

        return self._red_streak >= self.config.detector.light_confirm_frames

    def transition(self, next_state: str, now: float) -> None:
        """Moves to a new state and stores entry time."""

        self.state = next_state
        self.enter_t = now

    def elapsed(self, now: float) -> float:
        """Returns seconds elapsed in the current state."""

        return now - self.enter_t

    def single_lane_reacquired(self, lane: LaneObs) -> bool:
        """Checks whether branch geometry has returned to one stable lane."""

        return lane.valid and not lane.fork_seen and abs(lane.center_error) < 0.45

    def consume_lane_reset_request(self) -> bool:
        """Non-OUT routes never request a fork-history reset."""

        return False

    def fork_decision_update(self, detector: DetectionBuffer) -> str | None:
        """Locks a LEFT/RIGHT fork decision using temporal sign votes."""

        left_score = detector.count("sign_left", self.config.detector.sign_vote_n)
        right_score = detector.count("sign_right", self.config.detector.sign_vote_n)
        if left_score >= self.config.detector.sign_vote_k and left_score > right_score:
            return "LEFT"
        if right_score >= self.config.detector.sign_vote_k and right_score > left_score:
            return "RIGHT"
        return None


class OutCourseFSM(BaseCourseFSM):
    """Production OUT FSM, isolated from every IN-course transition."""

    def __init__(self, config: AutonomousConfig, controller: LaneController):
        super().__init__(config, controller)
        self.state = "OUT_WAIT_GREEN"
        self.fork_decision: str | None = None
        self.lane_reset_requested = False

    def consume_lane_reset_request(self) -> bool:
        """Returns and clears the one-shot post-fork perception reset request."""

        requested = self.lane_reset_requested
        self.lane_reset_requested = False
        return requested

    def control_directional_fork(
        self, lane: LaneObs, cap: float, steer_limit: float
    ) -> ControlCmd:
        """Steers toward the selected screen side without waiting for the X."""

        if self.fork_decision == "LEFT":
            target_error = self.config.steering.fork_forced_error
        else:
            target_error = -self.config.steering.fork_forced_error
        virtual_lane = lane.with_center_error(target_error)
        steer = self.controller.steering_from_lane(
            virtual_lane,
            steer_limit=steer_limit,
            curve_scale=self.config.steering.fork_curve_scale,
        )
        throttle = self.controller.throttle_scheduler(
            cap, steer, lane.curvature
        )
        return ControlCmd(throttle, steer)

    def step(
        self,
        lane: LaneObs,
        detector: DetectionBuffer,
        now: float,
        light_state=None,
        light_seq=None,
        marker_visible: bool = False,
    ) -> ControlCmd:
        self.update_light(light_state, light_seq)

        if self.state == "OUT_WAIT_GREEN":
            if self.green_confirmed():
                self.transition("OUT_TO_FORK", now)
            return ControlCmd(0.0, 0.0)

        if self.state == "OUT_TO_FORK":
            cmd = self.controller.follow_with_startup(
                lane, self.config.throttle.speed_max, self.config.steering.s_curve_limit
            )
            decision = self.fork_decision_update(detector)
            if decision is not None:
                self.fork_decision = decision
                self.transition("OUT_SIGN_STOP_DELAY", now)
            return cmd

        if self.state == "OUT_SIGN_STOP_DELAY":
            if self.elapsed(now) >= self.config.mission.sign_stop_delay_sec:
                self.controller.prev_throttle = 0.0
                self.transition("OUT_SIGN_STOPPED", now)
                return ControlCmd(0.0, 0.0)
            return self.controller.follow_with_startup(
                lane, self.config.throttle.speed_max, self.config.steering.s_curve_limit
            )

        if self.state == "OUT_SIGN_STOPPED":
            self.controller.prev_throttle = 0.0
            return ControlCmd(0.0, 0.0)

        if self.state == "OUT_FORK_SIGN_ADVANCE":
            cmd = self.control_directional_fork(
                lane,
                self.config.throttle.fork_approach_cap,
                self.config.steering.fork_approach_limit,
            )
            if self.elapsed(now) >= self.config.mission.fork_sign_advance_sec:
                self.transition("OUT_FORK_COMMIT", now)
            return cmd

        if self.state == "OUT_FORK_COMMIT":
            cmd = self.control_directional_fork(
                lane,
                self.config.throttle.fork_commit_cap,
                self.config.steering.fork_limit,
            )
            if (
                self.single_lane_reacquired(lane)
                and self.elapsed(now) >= self.config.mission.fork_commit_min_sec
            ) or self.elapsed(now) >= self.config.mission.fork_commit_timeout_sec:
                self.lane_reset_requested = True
                self.transition("OUT_RESUME", now)
            return cmd

        if self.state == "OUT_RESUME":
            return self.controller.lane_follow(
                lane,
                self.config.throttle.post_fork_cap,
                self.config.steering.post_fork_limit,
                section_min=self.config.throttle.post_fork_min,
            )

        self.controller.prev_throttle = 0.0
        return ControlCmd(0.0, 0.0)


class InCourseFSM(BaseCourseFSM):
    """Isolated IN skeleton; dash/stop-line gates intentionally remain pending."""

    def __init__(self, config: AutonomousConfig, controller: LaneController):
        super().__init__(config, controller)
        self.state = "IN_WAIT_GREEN"

    def step(
        self,
        lane: LaneObs,
        detector: DetectionBuffer,
        now: float,
        light_state=None,
        light_seq=None,
        marker_visible: bool = False,
    ) -> ControlCmd:
        self.update_light(light_state, light_seq)
        if self.state == "IN_WAIT_GREEN":
            if self.green_confirmed():
                self.transition("IN_ENTRY", now)
            return ControlCmd(0.0, 0.0)
        if self.state in ("IN_ENTRY", "IN_LAP"):
            return self.controller.lane_follow(
                lane, self.config.throttle.s_curve_cap, self.config.steering.s_curve_limit
            )
        if self.state == "IN_EXIT":
            return self.controller.lane_follow(
                lane, self.config.throttle.post_fork_cap, self.config.steering.post_fork_limit
            )
        if self.state == "IN_RESUME":
            return self.controller.lane_follow(
                lane, self.config.throttle.post_fork_cap, self.config.steering.post_fork_limit
            )
        self.controller.prev_throttle = 0.0
        return ControlCmd(0.0, 0.0)


class LaneTestFSM(BaseCourseFSM):
    """Pure lane-following mode for verifying lane detection + steering.

    No green-light gate and no mission transitions: it just follows the lane at
    the configured speed band on every tick. Select with route_mode=LANE so the
    car starts driving immediately without needing a traffic light.
    """

    def __init__(self, config: AutonomousConfig, controller: LaneController):
        """Creates the lane-verification state machine."""

        super().__init__(config, controller)
        self.state = "LANE_TEST"

    def step(self, lane: LaneObs, detector: DetectionBuffer, now: float,
             light_state=None, light_seq=None) -> ControlCmd:
        """Follows the lane every tick, capped by the speed band (ignores lights)."""

        return self.controller.lane_follow(
            lane,
            self.config.throttle.speed_max,
            self.config.steering.s_curve_limit,
        )


def make_course_fsm(config: AutonomousConfig, controller: LaneController) -> BaseCourseFSM:
    """Creates the route-specific FSM selected by route_mode."""

    route = config.mission.route_mode.upper()
    if route in ("LANE", "LANE_TEST", "TEST"):
        return LaneTestFSM(config, controller)
    if route == "IN":
        return InCourseFSM(config, controller)
    return OutCourseFSM(config, controller)
