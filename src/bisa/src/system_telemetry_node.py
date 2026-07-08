"""On-car system vitals publisher (CPU / temperature / memory).

The onboard stack runs YOLO + the lane pipeline on the TOPST A72 CPU with no
GPU, so a stall is almost always compute or thermal saturation. The headless car
can't show a monitor, so this node publishes its own vitals as plain Float32
topics that the PC-side power GUI subscribes to and displays next to the power
panels.

Deliberately dependency-free: it reads the numbers the kernel already keeps in
memory (``/proc/stat``, ``/proc/meminfo``, ``/sys/class/thermal``) instead of
pulling in psutil, so it adds a negligible, predictable load on the constrained
chip — a few file reads and four small publishes at 2 Hz.

Published (all std_msgs/Float32):
  /system/cpu_percent      overall CPU busy %
  /system/cpu_percent_max  busiest single core % (a saturated core stalls the
                           control loop even while the average looks idle)
  /system/cpu_temp_c       hottest thermal zone in Celsius
  /system/mem_percent      used memory %
"""

from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


def _read_cpu_times():
    """Returns {name: (idle, total)} for the aggregate and every core.

    Parses ``/proc/stat`` cpu lines. idle counts idle+iowait; total is the sum
    of all fields. Returns an empty dict if the file is unreadable so the caller
    just skips this tick rather than crashing.
    """

    times = {}
    try:
        for line in Path('/proc/stat').read_text().splitlines():
            if not line.startswith('cpu'):
                continue
            parts = line.split()
            name = parts[0]
            values = [int(v) for v in parts[1:]]
            if len(values) < 5:
                continue
            idle = values[3] + values[4]  # idle + iowait
            total = sum(values)
            times[name] = (idle, total)
    except (OSError, ValueError):
        pass
    return times


def _busy_percent(prev, curr):
    """Computes busy % from two (idle, total) samples; None if no delta yet."""

    if prev is None or curr is None:
        return None
    idle_delta = curr[0] - prev[0]
    total_delta = curr[1] - prev[1]
    if total_delta <= 0:
        return None
    return max(0.0, min(100.0, 100.0 * (1.0 - idle_delta / total_delta)))


def _read_cpu_temp_c():
    """Returns the hottest thermal zone in Celsius, or None if none readable."""

    temps = []
    for zone in Path('/sys/class/thermal').glob('thermal_zone*/temp'):
        try:
            milli = int(zone.read_text().strip())
        except (OSError, ValueError):
            continue
        # Sysfs reports millidegrees; a few boards report raw degrees already.
        temps.append(milli / 1000.0 if abs(milli) > 1000 else float(milli))
    return max(temps) if temps else None


def _read_mem_percent():
    """Returns used memory % from /proc/meminfo (MemTotal vs MemAvailable)."""

    total = available = None
    try:
        for line in Path('/proc/meminfo').read_text().splitlines():
            if line.startswith('MemTotal:'):
                total = float(line.split()[1])
            elif line.startswith('MemAvailable:'):
                available = float(line.split()[1])
            if total is not None and available is not None:
                break
    except (OSError, ValueError, IndexError):
        return None
    if not total or available is None:
        return None
    return max(0.0, min(100.0, 100.0 * (1.0 - available / total)))


class SystemTelemetryNode(Node):
    """Publishes CPU / temperature / memory vitals for the PC monitor."""

    def __init__(self):
        super().__init__('system_telemetry_node')

        self.declare_parameter('publish_hz', 2.0)
        publish_hz = float(self.get_parameter('publish_hz').value)
        if publish_hz <= 0.0:
            raise ValueError('publish_hz must be greater than 0')

        self.cpu_pub = self.create_publisher(Float32, '/system/cpu_percent', 10)
        self.cpu_max_pub = self.create_publisher(Float32, '/system/cpu_percent_max', 10)
        self.temp_pub = self.create_publisher(Float32, '/system/cpu_temp_c', 10)
        self.mem_pub = self.create_publisher(Float32, '/system/mem_percent', 10)

        # CPU % needs two samples to compute a delta; seed the baseline now so
        # the first timer tick already produces a real number.
        self.prev_times = _read_cpu_times()

        self.timer = self.create_timer(1.0 / publish_hz, self.timer_callback)
        self.get_logger().info(
            f'[System Telemetry] publishing CPU/temp/mem at {publish_hz} Hz'
        )

    def timer_callback(self):
        curr_times = _read_cpu_times()

        overall = _busy_percent(self.prev_times.get('cpu'), curr_times.get('cpu'))
        core_busy = [
            b for name, curr in curr_times.items() if name != 'cpu'
            for b in (_busy_percent(self.prev_times.get(name), curr),)
            if b is not None
        ]
        self.prev_times = curr_times

        if overall is not None:
            self.cpu_pub.publish(Float32(data=float(overall)))
        if core_busy:
            self.cpu_max_pub.publish(Float32(data=float(max(core_busy))))

        temp = _read_cpu_temp_c()
        if temp is not None:
            self.temp_pub.publish(Float32(data=float(temp)))

        mem = _read_mem_percent()
        if mem is not None:
            self.mem_pub.publish(Float32(data=float(mem)))


def main(args=None):
    rclpy.init(args=args)
    node = SystemTelemetryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
