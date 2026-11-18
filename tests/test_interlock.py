"""Never-both-PWM invariant: the duty -> (RPWM, LPWM) mapping the driver relies on."""
import math

import pytest

from shield.motor import split_duty


@pytest.mark.parametrize("duty", [-2.0, -1.0, -0.5, -1e-9, -0.0, 0.0, 1e-9, 0.3, 1.0, 5.0,
                                  math.nan, math.inf, -math.inf])
def test_never_both_channels(duty):
    r, l = split_duty(duty)
    assert r == 0.0 or l == 0.0
    assert 0.0 <= r <= 1.0 and 0.0 <= l <= 1.0


def test_direction_and_clamp():
    assert split_duty(0.4) == (0.4, 0.0)
    assert split_duty(-0.4) == (0.0, 0.4)
    assert split_duty(0.9, max_duty=0.3) == (0.3, 0.0)
    assert split_duty(-0.9, max_duty=0.3) == (0.0, 0.3)


def test_zero_and_non_finite_brake():
    for duty in (0.0, -0.0, math.nan, math.inf):
        assert split_duty(duty) == (0.0, 0.0)
