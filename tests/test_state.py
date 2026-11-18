"""Command priority: physical override > hazard close > requests > rest open."""
from types import SimpleNamespace

from shield.override import OverrideStatus
from shield.state import (CLOSED, CLOSING, FAULT, MANUAL, OPEN, Arbiter, Hazard, state_of)

AUTO = OverrideStatus()
MANUAL_IDLE = OverrideStatus(manual=True)
MANUAL_OPEN = OverrideStatus(manual=True, open_pressed=True)
MANUAL_CLOSE = OverrideStatus(manual=True, close_pressed=True)
CRITICAL = Hazard(1.0, emergency=True, reason="falling object")


def test_rests_open_by_default():
    cmd = Arbiter().decide(AUTO)
    assert (cmd.coverage, cmd.source) == (0.0, "default")


def test_request_used_without_hazard():
    assert Arbiter().decide(AUTO, request=0.4).coverage == 0.4


def test_hazard_beats_dashboard_open():
    cmd = Arbiter().decide(AUTO, hazard=CRITICAL, request=0.0)
    assert cmd.coverage == 1.0 and cmd.emergency and cmd.source == "hazard"


def test_request_can_close_further_than_a_warning():
    assert Arbiter().decide(AUTO, hazard=Hazard(0.5), request=0.8).coverage == 0.8


def test_manual_beats_hazard_and_latches():
    a = Arbiter()
    assert a.decide(MANUAL_IDLE, hazard=CRITICAL).coverage is None  # holds, ignores hazard
    cmd = a.decide(MANUAL_OPEN, hazard=CRITICAL)
    assert (cmd.coverage, cmd.source, cmd.emergency) == (0.0, "manual", False)
    assert a.decide(MANUAL_IDLE, hazard=CRITICAL).coverage == 0.0     # latched after release
    assert a.decide(MANUAL_CLOSE).coverage == 1.0


def test_both_buttons_ignored():
    a = Arbiter()
    assert a.decide(OverrideStatus(True, True, True)).coverage is None


def test_back_to_auto_drops_manual_target():
    a = Arbiter()
    a.decide(MANUAL_CLOSE)
    assert a.decide(AUTO).coverage == 0.0


def test_fault_blocks_auto_but_manual_press_clears_it():
    a = Arbiter()
    assert a.decide(AUTO, hazard=CRITICAL, faulted=True).coverage is None
    cmd = a.decide(MANUAL_CLOSE, faulted=True)
    assert cmd.coverage == 1.0 and cmd.clear_fault


def _ctrl(**kw):
    base = dict(fault="", mode="holding", zero=0, goal=100, position=100, coverage=0.0, direction=0)
    return SimpleNamespace(**{**base, **kw})


def test_state_names():
    assert state_of(_ctrl(fault="stalled"), "manual") == (FAULT, "stalled")
    assert state_of(_ctrl(), "manual")[0] == MANUAL
    assert state_of(_ctrl(mode="moving", goal=900, position=100, direction=1), "hazard")[0] == CLOSING
    assert state_of(_ctrl(coverage=0.0), "default")[0] == OPEN
    assert state_of(_ctrl(coverage=1.0), "default")[0] == CLOSED


def test_hazard_close_latches_until_manual_open():
    a = Arbiter()
    assert a.decide(AUTO, hazard=CRITICAL).coverage == 1.0
    # Hazard gone (the camera is blind under the shell): stays closed, even if asked to open.
    cmd = a.decide(AUTO, hazard=None, request=0.0)
    assert (cmd.coverage, cmd.source, cmd.emergency) == (1.0, "hazard", True)
    # MANUAL alone doesn't release it; back to AUTO is still closed.
    a.decide(MANUAL_IDLE)
    assert a.decide(AUTO).coverage == 1.0
    # MANUAL + CLOSE doesn't release it either.
    a.decide(MANUAL_CLOSE)
    assert a.decide(AUTO).coverage == 1.0
    # MANUAL + OPEN retracts and re-arms.
    assert a.decide(MANUAL_OPEN).coverage == 0.0
    assert a.decide(AUTO).coverage == 0.0


def test_latch_escalates_but_never_relaxes():
    a = Arbiter()
    assert a.decide(AUTO, hazard=Hazard(0.5)).coverage == 0.5
    assert a.decide(AUTO, hazard=Hazard(0.3)).coverage == 0.5
    cmd = a.decide(AUTO, hazard=CRITICAL)
    assert cmd.coverage == 1.0 and cmd.emergency
    assert a.decide(AUTO, hazard=Hazard(0.2)).emergency  # stays the same command: no re-profile


def test_manual_ignores_new_hazards():
    a = Arbiter()
    a.decide(MANUAL_OPEN, hazard=CRITICAL)
    assert a.latched is None
    assert a.decide(AUTO).coverage == 0.0
