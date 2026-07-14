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
        target = cap
        target -= self.config.throttle.steer_slowdown * abs(steer)
        target -= self.config.throttle.curvature_slowdown * curvature
        target = clamp(target, floor, cap)
        if target > self.prev_throttle:
            target = min(target, self.prev_throttle + self.config.throttle.ramp_up_per_cmd)
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


class BaseCourseFSM:
    """Shared helpers for the course state machines."""

    def __init__(self, config: AutonomousConfig, controller: LaneController):
        """Initializes common timing and finish bookkeeping."""

        self.config = config
        self.controller = controller
        self.state = ""
        self.enter_t = 0.0
        self.start_t = 0.0
        self.finish_crossed = False
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

    def finish_window_open(self, now: float) -> bool:
        """Latches the red-listen finish window open after enough elapsed time.

        The detect model has no finish_line class, so the finish window is a
        pure mission timer: once ``finish_min_elapsed_sec`` has passed since
        launch (so the start light is out of view), a confirmed red verdict is
        allowed to stop the car. The latch never re-closes.
        """

        if not self.finish_crossed and now - self.start_t >= self.config.mission.finish_min_elapsed_sec:
            self.finish_crossed = True
        return self.finish_crossed

    def single_lane_reacquired(self, lane: LaneObs) -> bool:
        """Checks whether branch geometry has returned to one stable lane."""

        return lane.valid and not lane.fork_seen and abs(lane.center_error) < 0.45

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
    """FSM for the Out/Base route: green launch, S-curve, sign fork, red stop.

    Reachable path:
        OUT_WAIT_GREEN -> OUT_LAUNCH -> OUT_S_CURVE -> OUT_FORK_APPROACH
        -> OUT_FORK_COMMIT -> OUT_POST_FORK -(confirmed red)-> OUT_FINISH_STOP

    OUT_POST_FORK cruises to the finish; the red stop is handled by the global
    gate in :meth:`step`, not by a dedicated finish-approach state.
    """

    def __init__(self, config: AutonomousConfig, controller: LaneController):
        """Creates the OUT course state machine."""

        super().__init__(config, controller)
        self.state = "OUT_WAIT_GREEN"
        self.fork_decision: str | None = None

    def choose_branch_target(self, lane: LaneObs, decision: str | None) -> float:
        """Selects branch candidate error or applies a sign-based fallback bias."""

        if decision == "LEFT" and lane.left_branch is not None:
            return lane.left_branch.target_error
        if decision == "RIGHT" and lane.right_branch is not None:
            return lane.right_branch.target_error
        if decision == "LEFT":
            return lane.center_error + 0.18
        if decision == "RIGHT":
            return lane.center_error - 0.18
        return lane.center_error

    def control_fork_commit(self, lane: LaneObs) -> ControlCmd:
        """Controls the committed fork path without changing the locked decision."""

        target_error = self.choose_branch_target(lane, self.fork_decision)
        virtual_lane = lane.with_center_error(target_error)
        steer = self.controller.steering_from_lane(
            virtual_lane,
            steer_limit=self.config.steering.fork_limit,
            curve_scale=self.config.steering.fork_curve_scale,
        )
        throttle = self.controller.throttle_scheduler(
            self.config.throttle.fork_commit_cap,
            steer,
            lane.curvature,
        )
        return ControlCmd(throttle, steer)

    def step(self, lane: LaneObs, detector: DetectionBuffer, now: float,
             light_state=None, light_seq=None) -> ControlCmd:
        """Runs one OUT-course FSM tick and returns the desired control command."""

        if self.start_t <= 0.0:
            self.start_t = now
            self.enter_t = now
        self.update_light(light_state, light_seq)

        # Arrival red-stop, independent of the intermediate state chain: once the
        # finish window is open (enough time since launch, so the start light is
        # out of view), a confirmed red verdict stops the car from ANY driving
        # state. OUT_POST_FORK has no detection-driven exit, so this global gate
        # is what actually ends the run.
        if (
            self.state not in ("OUT_WAIT_GREEN", "OUT_FINISH_STOP")
            and self.finish_window_open(now)
            and self.red_confirmed()
        ):
            self.transition("OUT_FINISH_STOP", now)

        if self.state == "OUT_WAIT_GREEN":
            cmd = ControlCmd(0.0, 0.0)
            if self.green_confirmed():
                # Mission clock starts at launch, not at node startup, so the
                # finish timer is unaffected by how long the car waited at the
                # start light.
                self.start_t = now
                self.transition("OUT_LAUNCH", now)

        elif self.state == "OUT_LAUNCH":
            cmd = self.controller.lane_follow(
                lane,
                self.config.throttle.launch_cap,
                self.config.steering.straight_limit,
            )
            if self.elapsed(now) > self.config.mission.launch_min_sec:
                self.transition("OUT_S_CURVE", now)

        elif self.state == "OUT_S_CURVE":
            cmd = self.controller.lane_follow(
                lane,
                self.config.throttle.s_curve_cap,
                self.config.steering.s_curve_limit,
            )
            if self.fork_decision_update(detector) is not None or lane.fork_seen:
                self.transition("OUT_FORK_APPROACH", now)

        elif self.state == "OUT_FORK_APPROACH":
            cmd = self.controller.lane_follow(
                lane,
                self.config.throttle.fork_approach_cap,
                self.config.steering.fork_approach_limit,
                curve_scale=self.config.steering.fork_curve_scale,
            )
            decision = self.fork_decision_update(detector)
            if decision is not None:
                self.fork_decision = decision
                self.transition("OUT_FORK_COMMIT", now)

        elif self.state == "OUT_FORK_COMMIT":
            cmd = self.control_fork_commit(lane)
            commit_done = self.single_lane_reacquired(lane) and self.elapsed(now) > self.config.mission.fork_commit_min_sec
            if commit_done or self.elapsed(now) > self.config.mission.fork_commit_timeout_sec:
                self.transition("OUT_POST_FORK", now)

        elif self.state == "OUT_POST_FORK":
            # Terminal cruise: follow the lane to the finish. The confirmed-red
            # stop is handled by the global gate above.
            cmd = self.controller.lane_follow(
                lane,
                self.config.throttle.post_fork_cap,
                self.config.steering.post_fork_limit,
                section_min=self.config.throttle.post_fork_min,
            )

        else:
            cmd = ControlCmd(0.0, 0.0)

        return cmd


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

        if self.start_t <= 0.0:
            self.start_t = now
            self.enter_t = now
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
    return OutCourseFSM(config, controller)
