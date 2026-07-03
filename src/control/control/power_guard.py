"""Board-priority voltage guard: throttle the motor to protect the 5V rail.

The D-Racer's motor/ESC and the D3-G's 5V/5A regulator share one 2S 18650 pack.
When the pack sags toward the regulator's dropout, the board's 5V rail (and the
USB camera on it) browns out. Software cannot allocate power directly, but it
can enforce the priority the other way around: watch the pack voltage and scale
the throttle DOWN before the voltage reaches the danger zone, so the regulator
always keeps headroom. Motor gets less; board stays alive.

Pure Python (no ROS imports) so control_node can embed it and tests can run
anywhere. Behavior:

* Voltage is low-pass filtered asymmetrically: sags are tracked fast
  (``alpha_down``), recoveries slowly (``alpha_up``) — the pack bounces ±0.4 V
  under load, and reacting to every bounce would oscillate.
* Above ``low_v`` the scale is 1.0 (guard idle). Between ``low_v`` and
  ``critical_v`` it ramps linearly down to ``min_scale``. At/below
  ``critical_v`` it clamps at ``min_scale`` (still moving: a mid-mission full
  stop is its own failure).
* Scale cuts are applied instantly, recovery is rate-limited
  (``recover_per_tick``) so the guard doesn't chatter.
* If voltage data goes stale (battery_node died), the guard releases toward 1.0
  through the same slow-recovery path rather than pinning the car to a stale
  limit.
"""

from __future__ import annotations


class VoltageGuard:
    """Computes a [min_scale, 1.0] throttle multiplier from pack voltage."""

    def __init__(
        self,
        enabled: bool = True,
        low_v: float = 6.5,
        critical_v: float = 6.2,
        # Floor high enough that scaled AUTO commands still move the car: the
        # autonomy speed band is 0.20-0.25 and 0.20*0.75=0.15 clears the ESC's
        # forward deadband (the joystick default 0.12 is known to move it).
        min_scale: float = 0.75,
        alpha_down: float = 0.5,
        alpha_up: float = 0.05,
        recover_per_tick: float = 0.02,
        stale_sec: float = 3.0,
    ):
        """Validates thresholds and initializes the filter state."""

        if low_v <= critical_v:
            raise ValueError('low_v must be greater than critical_v')
        if not 0.0 < min_scale <= 1.0:
            raise ValueError('min_scale must be in (0, 1]')

        self.enabled = bool(enabled)
        self.low_v = float(low_v)
        self.critical_v = float(critical_v)
        self.min_scale = float(min_scale)
        self.alpha_down = float(alpha_down)
        self.alpha_up = float(alpha_up)
        self.recover_per_tick = float(recover_per_tick)
        self.stale_sec = float(stale_sec)

        self.filtered_v: float | None = None
        self.last_update: float | None = None
        self.scale = 1.0

    def update_voltage(self, voltage: float, now: float) -> None:
        """Feeds one pack-voltage sample into the asymmetric low-pass filter."""

        voltage = float(voltage)
        if self.filtered_v is None:
            self.filtered_v = voltage
        else:
            alpha = self.alpha_down if voltage < self.filtered_v else self.alpha_up
            self.filtered_v += alpha * (voltage - self.filtered_v)
        self.last_update = now

    def target_scale(self) -> float:
        """Maps the filtered voltage to the raw guard scale (no smoothing)."""

        if self.filtered_v is None:
            return 1.0
        v = self.filtered_v
        if v >= self.low_v:
            return 1.0
        if v <= self.critical_v:
            return self.min_scale
        span = self.low_v - self.critical_v
        return self.min_scale + (1.0 - self.min_scale) * (v - self.critical_v) / span

    def tick(self, now: float) -> float:
        """Advances one control tick and returns the throttle multiplier."""

        if not self.enabled:
            self.scale = 1.0
            return self.scale

        stale = (
            self.last_update is None
            or (now - self.last_update) > self.stale_sec
        )
        target = 1.0 if stale else self.target_scale()

        if target < self.scale:
            # Sag: cut immediately — headroom protection cannot wait.
            self.scale = target
        else:
            # Recovery: ramp back slowly so the guard doesn't oscillate.
            self.scale = min(target, self.scale + self.recover_per_tick)
        return self.scale

    @property
    def active(self) -> bool:
        """True while the guard is actually limiting the motor."""

        return self.scale < 0.995
