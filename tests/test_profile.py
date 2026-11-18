"""Trapezoidal profile: reaches the goal, never overshoots, respects speed and accel."""
import pytest

from shield.controller import profile_step


@pytest.mark.parametrize("start,goal,v0", [(0, 5000, 0), (5000, 100, 0), (0, 10, 0),
                                           (1000, 2000, -1500), (0, 0, 0)])
def test_profile(start, goal, v0):
    vmax, accel, dt = 2500.0, 10000.0, 0.005
    p, v = float(start), float(v0)
    lo, hi = min(start, goal), max(start, goal)
    for _ in range(4000):
        p_new, v_new = profile_step(p, v, goal, vmax, accel, dt)
        assert abs(v_new) <= max(vmax, abs(v0)) + 1e-6
        assert abs(v_new - v) <= accel * dt + 1e-6 or (p_new, v_new) == (goal, 0.0)
        p, v = p_new, v_new
        if v0 == 0:
            assert lo - 1e-6 <= p <= hi + 1e-6  # never past the goal
        if (p, v) == (goal, 0.0):
            break
    assert (p, v) == (goal, 0.0)
