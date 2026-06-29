"""Mission FSM and normalized throttle/steering controller for D-Racer."""

from __future__ import annotations

from dataclasses import dataclass

from .dracer_config import AutonomousConfig
from .lane_perception import LaneObs, clamp
from .object_detector import DetectionBuffer


@dataclass
class ControlCmd:
    """Carries normalized control values published to /control."""

    throttle: float
    steering: float


class LaneController:
    """Turns LaneObs values into smooth steering and throttle commands."""

    def __init__(self, config: AutonomousConfig):
        """Initializes previous values used by PD and rate limiting."""

        self.config = config
        self.prev_error = 0.0
        self.prev_steer = 0.0
        self.prev_throttle = 0.0

    def rate_limit(self, target: float, previous: float, max_delta: float) -> float:
        """Limits one control update to prevent sudden steering jumps."""

        return previous + clamp(target - previous, -max_delta, max_delta)

    def steering_from_lane(self, lane: LaneObs, steer_ff: float = 0.0, steer_limit: float = 0.80) -> float:
        """Computes PD lane steering plus curvature/feedforward correction."""

        if not lane.valid:
            raw = self.prev_steer * self.config.steering.lost_decay
        else:
            d_error = lane.center_error - self.prev_error
            raw = (
                self.config.steering.kp * lane.center_error
                + self.config.steering.kd * d_error
                + self.config.steering.kcurv * lane.signed_curvature
                + steer_ff
            )
            self.prev_error = lane.center_error

        raw = clamp(raw, -steer_limit, steer_limit)
        steer = self.rate_limit(raw, self.prev_steer, self.config.steering.rate_limit_per_cmd)
        self.prev_steer = steer
        return float(self.config.steering.steer_sign) * steer

    def throttle_scheduler(
        self,
        section_cap: float,
        steer: float,
        curvature: float,
        force_stop: bool = False,
    ) -> float:
        """Schedules throttle with immediate braking and ramped acceleration."""

        if force_stop:
            self.prev_throttle = 0.0
            return 0.0

        cap = clamp(section_cap, 0.0, self.config.throttle.max)
        target = cap
        target -= self.config.throttle.steer_slowdown * abs(steer)
        target -= self.config.throttle.curvature_slowdown * curvature
        target = clamp(target, self.config.throttle.min_moving, cap)
        if target > self.prev_throttle:
            target = min(target, self.prev_throttle + self.config.throttle.ramp_up_per_cmd)
        self.prev_throttle = target
        return target

    def lane_follow(self, lane: LaneObs, cap: float, steer_limit: float) -> ControlCmd:
        """Follows the latest lane observation with cautious lost-lane behavior."""

        if not lane.valid:
            steer = self.rate_limit(
                self.prev_steer * self.config.steering.lost_decay,
                self.prev_steer,
                self.config.steering.rate_limit_per_cmd,
            )
            self.prev_steer = steer
            throttle = min(self.prev_throttle, self.config.throttle.min_moving)
            self.prev_throttle = throttle
            return ControlCmd(throttle, steer)

        steer = self.steering_from_lane(lane, steer_ff=0.0, steer_limit=steer_limit)
        throttle = self.throttle_scheduler(cap, steer, lane.curvature)
        return ControlCmd(throttle, steer)

    def hold_or_zero_steering(self) -> float:
        """Slowly releases the last steering command while stopped."""

        steer = self.rate_limit(0.0, self.prev_steer, self.config.steering.rate_limit_per_cmd)
        self.prev_steer = steer
        return steer


class BaseCourseFSM:
    """Shared helpers for OUT and IN course state machines."""

    def __init__(self, config: AutonomousConfig, controller: LaneController):
        """Initializes common timing and finish bookkeeping."""

        self.config = config
        self.controller = controller
        self.state = ""
        self.enter_t = 0.0
        self.start_t = 0.0
        self.finish_crossed = False

    def transition(self, next_state: str, now: float) -> None:
        """Moves to a new state and stores entry time."""

        self.state = next_state
        self.enter_t = now

    def elapsed(self, now: float) -> float:
        """Returns seconds elapsed in the current state."""

        return now - self.enter_t

    def finish_line_crossed(self, detector: DetectionBuffer, now: float) -> bool:
        """Latches finish after elapsed time or optional finish_line detection."""

        if self.finish_crossed:
            return True
        if detector.stable_consecutive("finish_line", 2):
            self.finish_crossed = True
        elif now - self.start_t >= self.config.mission.finish_min_elapsed_sec:
            self.finish_crossed = True
        return self.finish_crossed

    def likely_dynamic_zone(self, lane: LaneObs, now: float) -> bool:
        """Estimates dynamic zone approach from mission elapsed time and lane context."""

        return now - self.start_t >= self.config.mission.dynamic_zone_elapsed_sec or lane.curvature < 0.12

    def single_lane_reacquired(self, lane: LaneObs) -> bool:
        """Checks whether branch/rotary geometry has returned to one stable lane."""

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
    """FSM for the Out/Base route: S-curve, fork, dynamic obstacle, finish."""

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
            steer_ff=0.0,
            steer_limit=self.config.steering.fork_limit,
        )
        throttle = self.controller.throttle_scheduler(
            self.config.throttle.fork_commit_cap,
            steer,
            lane.curvature,
        )
        return ControlCmd(throttle, steer)

    def step(self, lane: LaneObs, detector: DetectionBuffer, now: float) -> ControlCmd:
        """Runs one OUT-course FSM tick and returns the desired control command."""

        if self.start_t <= 0.0:
            self.start_t = now
            self.enter_t = now

        if self.state == "OUT_WAIT_GREEN":
            cmd = ControlCmd(0.0, 0.0)
            if detector.stable_consecutive("traffic_green", self.config.detector.green_consecutive):
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
            cap = self.config.throttle.post_fork_cap
            limit = self.config.steering.post_fork_limit
            if self.likely_dynamic_zone(lane, now):
                cap = self.config.throttle.dynamic_approach_cap
                limit = self.config.steering.dynamic_limit
            cmd = self.controller.lane_follow(lane, cap, limit)
            if detector.stable_consecutive("dynamic_marker", self.config.detector.dynamic_detect_consecutive):
                self.transition("OUT_DYNAMIC_STOP", now)

        elif self.state == "OUT_DYNAMIC_STOP":
            cmd = ControlCmd(0.0, self.controller.hold_or_zero_steering())
            if self.elapsed(now) >= self.config.mission.dynamic_stop_hold_sec:
                self.transition("OUT_DYNAMIC_WAIT_CLEAR", now)

        elif self.state == "OUT_DYNAMIC_WAIT_CLEAR":
            cmd = ControlCmd(0.0, 0.0)
            if detector.not_seen_consecutive("dynamic_marker", self.config.detector.dynamic_clear_consecutive):
                self.transition("OUT_RESUME", now)

        elif self.state == "OUT_RESUME":
            cap = self.config.throttle.dynamic_approach_cap if not lane.valid else self.config.throttle.resume_cap
            cmd = self.controller.lane_follow(lane, cap, self.config.steering.resume_limit)
            if lane.valid and self.elapsed(now) > self.config.mission.resume_min_sec:
                self.transition("OUT_FINISH_APPROACH", now)

        elif self.state == "OUT_FINISH_APPROACH":
            cmd = self.controller.lane_follow(
                lane,
                self.config.throttle.finish_cap,
                self.config.steering.finish_limit,
            )
            if self.finish_line_crossed(detector, now) and detector.stable_consecutive(
                "traffic_red",
                self.config.detector.red_consecutive_after_finish,
            ):
                self.transition("OUT_FINISH_STOP", now)

        else:
            cmd = ControlCmd(0.0, 0.0)

        return cmd


class InCourseFSM(BaseCourseFSM):
    """FSM for the In/Shortcut route: rotary, dynamic obstacle, finish."""

    def __init__(self, config: AutonomousConfig, controller: LaneController):
        """Creates the IN course state machine."""

        super().__init__(config, controller)
        self.state = "IN_WAIT_GREEN"
        self.rotary_progress = 0.0
        self.last_progress_t: float | None = None
        self.exit_seen_count = 0

    def rotary_sign(self) -> float:
        """Returns feedforward steering sign for clockwise/counterclockwise rotary."""

        return 1.0 if self.config.rotary.direction.upper() == "CCW" else -1.0

    def reset_rotary_progress(self, now: float) -> None:
        """Clears rotary yaw proxy values at rotary entry."""

        self.rotary_progress = 0.0
        self.last_progress_t = now
        self.exit_seen_count = 0

    def update_rotary_progress(self, steer: float, throttle: float, now: float) -> float:
        """Integrates steering and throttle as a camera-only rotation proxy."""

        if self.last_progress_t is None:
            self.last_progress_t = now
            return self.rotary_progress
        dt = max(0.0, now - self.last_progress_t)
        self.last_progress_t = now
        self.rotary_progress += abs(steer) * max(throttle, 0.0) * dt
        return self.rotary_progress

    def rotation_ok(self, now: float) -> bool:
        """Checks minimum rotary time and progress before allowing exit."""

        time_ok = self.elapsed(now) >= self.config.rotary.min_rotation_time_sec
        progress_ok = self.rotary_progress >= self.config.rotary.progress_threshold
        return time_ok and progress_ok

    def rotary_entry_valid(self, lane: LaneObs) -> bool:
        """Verifies the front road is connected enough to enter the rotary."""

        return lane.valid and (lane.rotary_seen or lane.curvature > 0.18)

    def inside_rotary(self, lane: LaneObs) -> bool:
        """Detects sustained curved road shape inside the rotary."""

        return lane.valid and (lane.rotary_seen or lane.curvature > 0.22)

    def control_rotary_exit(self, lane: LaneObs) -> ControlCmd:
        """Biases toward the outside exit branch after one full rotary pass."""

        bias = self.config.rotary.exit_bias * self.rotary_sign()
        virtual_lane = lane.with_center_error(lane.center_error + bias)
        steer = self.controller.steering_from_lane(
            virtual_lane,
            steer_ff=0.0,
            steer_limit=self.config.steering.rotary_limit,
        )
        throttle = self.controller.throttle_scheduler(
            self.config.throttle.rotary_inside_cap,
            steer,
            lane.curvature,
        )
        return ControlCmd(throttle, steer)

    def step(self, lane: LaneObs, detector: DetectionBuffer, now: float) -> ControlCmd:
        """Runs one IN-course FSM tick and returns the desired control command."""

        if self.start_t <= 0.0:
            self.start_t = now
            self.enter_t = now

        if self.state == "IN_WAIT_GREEN":
            cmd = ControlCmd(0.0, 0.0)
            if detector.stable_consecutive("traffic_green", self.config.detector.green_consecutive):
                self.transition("IN_LAUNCH", now)

        elif self.state == "IN_LAUNCH":
            cmd = self.controller.lane_follow(
                lane,
                self.config.throttle.launch_cap,
                self.config.steering.straight_limit,
            )
            if lane.rotary_seen or self.elapsed(now) > self.config.mission.launch_min_sec:
                self.transition("IN_ROTARY_APPROACH", now)

        elif self.state == "IN_ROTARY_APPROACH":
            cmd = self.controller.lane_follow(
                lane,
                self.config.throttle.rotary_approach_cap,
                self.config.steering.fork_approach_limit,
            )
            if self.rotary_entry_valid(lane):
                self.transition("IN_ROTARY_ENTER", now)
                self.reset_rotary_progress(now)

        elif self.state == "IN_ROTARY_ENTER":
            steer = self.controller.steering_from_lane(
                lane,
                steer_ff=self.config.rotary.enter_ff * self.rotary_sign(),
                steer_limit=self.config.steering.rotary_limit,
            )
            throttle = self.controller.throttle_scheduler(self.config.throttle.rotary_inside_cap, steer, lane.curvature)
            cmd = ControlCmd(throttle, steer)
            if self.inside_rotary(lane):
                self.transition("IN_ROTARY_CIRCULATE", now)
                self.reset_rotary_progress(now)

        elif self.state == "IN_ROTARY_CIRCULATE":
            steer = self.controller.steering_from_lane(
                lane,
                steer_ff=self.config.rotary.circulate_ff * self.rotary_sign(),
                steer_limit=self.config.steering.rotary_limit,
            )
            throttle = self.controller.throttle_scheduler(self.config.throttle.rotary_inside_cap, steer, lane.curvature)
            cmd = ControlCmd(throttle, steer)
            self.update_rotary_progress(steer, throttle, now)
            if self.rotation_ok(now) and lane.rotary_exit_seen:
                self.exit_seen_count += 1
            else:
                self.exit_seen_count = 0
            if self.exit_seen_count >= self.config.rotary.exit_stable_frames:
                self.transition("IN_ROTARY_EXIT_READY", now)

        elif self.state == "IN_ROTARY_EXIT_READY":
            cmd = self.control_rotary_exit(lane)
            if lane.rotary_exit_seen:
                self.transition("IN_ROTARY_EXIT_COMMIT", now)

        elif self.state == "IN_ROTARY_EXIT_COMMIT":
            cmd = self.control_rotary_exit(lane)
            commit_done = self.single_lane_reacquired(lane) and self.elapsed(now) > self.config.mission.fork_commit_min_sec
            if commit_done or self.elapsed(now) > self.config.mission.fork_commit_timeout_sec:
                self.transition("IN_POST_ROTARY", now)

        elif self.state == "IN_POST_ROTARY":
            cap = self.config.throttle.post_fork_cap
            limit = self.config.steering.post_fork_limit
            if self.likely_dynamic_zone(lane, now):
                cap = self.config.throttle.dynamic_approach_cap
                limit = self.config.steering.dynamic_limit
            cmd = self.controller.lane_follow(lane, cap, limit)
            if detector.stable_consecutive("dynamic_marker", self.config.detector.dynamic_detect_consecutive):
                self.transition("IN_DYNAMIC_STOP", now)

        elif self.state == "IN_DYNAMIC_STOP":
            cmd = ControlCmd(0.0, self.controller.hold_or_zero_steering())
            if self.elapsed(now) >= self.config.mission.dynamic_stop_hold_sec:
                self.transition("IN_DYNAMIC_WAIT_CLEAR", now)

        elif self.state == "IN_DYNAMIC_WAIT_CLEAR":
            cmd = ControlCmd(0.0, 0.0)
            if detector.not_seen_consecutive("dynamic_marker", self.config.detector.dynamic_clear_consecutive):
                self.transition("IN_RESUME", now)

        elif self.state == "IN_RESUME":
            cap = self.config.throttle.dynamic_approach_cap if not lane.valid else self.config.throttle.resume_cap
            cmd = self.controller.lane_follow(lane, cap, self.config.steering.resume_limit)
            if lane.valid and self.elapsed(now) > self.config.mission.resume_min_sec:
                self.transition("IN_FINISH_APPROACH", now)

        elif self.state == "IN_FINISH_APPROACH":
            cmd = self.controller.lane_follow(
                lane,
                self.config.throttle.finish_cap,
                self.config.steering.finish_limit,
            )
            if self.finish_line_crossed(detector, now) and detector.stable_consecutive(
                "traffic_red",
                self.config.detector.red_consecutive_after_finish,
            ):
                self.transition("IN_FINISH_STOP", now)

        else:
            cmd = ControlCmd(0.0, 0.0)

        return cmd


def make_course_fsm(config: AutonomousConfig, controller: LaneController) -> BaseCourseFSM:
    """Creates the route-specific FSM selected by route_mode."""

    if config.mission.route_mode.upper() == "IN":
        return InCourseFSM(config, controller)
    return OutCourseFSM(config, controller)
