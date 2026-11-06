"""Nonblocking, architecture-independent Linux CPU utilization sampling."""

from pathlib import Path


class CpuUsage:
    def __init__(self, stat_path='/proc/stat'):
        self.stat_path = Path(stat_path)
        self._previous = None
        self.sample()  # Establish a baseline before the first display refresh.

    def sample(self) -> float | None:
        """Return total busy CPU percent since the last sample, or None if unavailable."""
        try:
            with self.stat_path.open() as stat:
                fields = stat.readline().split()
            if not fields or fields[0] != 'cpu' or len(fields) < 5:
                raise ValueError('Missing aggregate CPU counters')
            # user, nice, system, idle, iowait, irq, softirq, steal.
            # guest counters are already included in user/nice; do not count twice.
            counters = [int(value) for value in fields[1:9]]
            total = sum(counters)
            idle = counters[3] + (counters[4] if len(counters) > 4 else 0)
        except (OSError, ValueError):
            self._previous = None
            return None

        previous = self._previous
        self._previous = (total, idle)
        if previous is None:
            return None
        elapsed = total - previous[0]
        idle_elapsed = idle - previous[1]
        if elapsed <= 0 or idle_elapsed < 0:
            return None
        return max(0.0, min(100.0, 100.0 * (elapsed - idle_elapsed) / elapsed))
