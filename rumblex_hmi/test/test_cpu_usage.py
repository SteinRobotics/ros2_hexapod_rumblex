"""CPU percentages use interval deltas and remain independent of core count."""

import pytest

from rumblex_hmi.cpu_usage import CpuUsage


@pytest.mark.parametrize('scale', [1, 4, 20])
def test_aggregate_usage_excludes_iowait_and_duplicate_guest_time(tmp_path, scale):
    stat = tmp_path / 'stat'

    def write(values):
        stat.write_text('cpu ' + ' '.join(str(value * scale) for value in values)
                        + '\ncpu0 999 0 999 0\n')

    write([100, 0, 50, 800, 50, 0, 0, 0, 20, 0])
    usage = CpuUsage(stat)
    write([120, 0, 55, 865, 60, 0, 0, 0, 30, 0])
    assert usage.sample() == pytest.approx(25.0)
    assert usage.sample() is None  # No elapsed ticks.
    write([120, 0, 55, 965, 60, 0, 0, 0, 30, 0])
    assert usage.sample() == 0.0
    write([220, 0, 55, 965, 60, 0, 0, 0, 30, 0])
    assert usage.sample() == 100.0


@pytest.mark.parametrize('invalid', ['', 'cpu0 1 2 3 4', 'cpu 1 2', 'cpu a b c d'])
def test_missing_or_malformed_reading_recovers(tmp_path, invalid):
    stat = tmp_path / 'stat'
    usage = CpuUsage(stat)
    assert usage.sample() is None
    stat.write_text(invalid)
    assert usage.sample() is None
    stat.write_text('cpu 10 0 10 80\n')
    assert usage.sample() is None
    stat.write_text('cpu 20 0 20 160\n')
    assert usage.sample() == pytest.approx(20.0)
    stat.unlink()
    assert usage.sample() is None


def test_counter_reset_restarts_measurement(tmp_path):
    stat = tmp_path / 'stat'
    stat.write_text('cpu 100 0 100 800\n')
    usage = CpuUsage(stat)
    stat.write_text('cpu 10 0 10 80\n')
    assert usage.sample() is None
    stat.write_text('cpu 20 0 20 160\n')
    assert usage.sample() == pytest.approx(20.0)
