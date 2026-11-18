"""Who gets to move the shield, and what state it is in.

Priority: physical override > hazard close > requests (CLI / dashboard) > rest open.

A hazard close LATCHES. The camera sits under the shell, so once the shield closes it
can't see whether the hazard has passed. Only a person can reopen it: MANUAL, press OPEN.
"""
from dataclasses import dataclass

UNHOMED, HOMING, OPEN, OPENING, CLOSING, CLOSED, PARTIAL, MANUAL, FAULT = (
    "UNHOMED", "HOMING", "OPEN", "OPENING", "CLOSING", "CLOSED", "PARTIAL", "MANUAL", "FAULT")


@dataclass(frozen=True)
class Hazard:
    coverage: float           # 0..1 the hazard demands
    emergency: bool = False   # CRITICAL: max speed, profile bypassed
    reason: str = ""
    ttc_s: float | None = None      # time to contact, if known
    t_capture: float | None = None  # monotonic time the triggering frame was exposed
    t_detect: float | None = None   # monotonic time detection finished


@dataclass(frozen=True)
class Command:
    coverage: float | None    # None = stop and hold where it is
    emergency: bool = False
    source: str = "default"   # manual | fault | hazard | request | default
    clear_fault: bool = False


class Arbiter:
    """Decides each tick what the controller should do."""

    def __init__(self):
        self.manual_target = None  # set by the last button press
        self.latched = None        # the hazard close in force until a manual OPEN

    def decide(self, override, hazard: Hazard | None = None, request: float | None = None,
               faulted: bool = False) -> Command:
        if override.manual:
            pressed = None
            if override.close_pressed != override.open_pressed:  # both at once = ignore
                pressed = 1.0 if override.close_pressed else 0.0
                self.manual_target = pressed
                if pressed == 0.0:
                    self.latched = None  # a person retracted the shell: re-arm
            # A press is a human taking charge, so it also clears a fault.
            return Command(self.manual_target, source="manual", clear_fault=pressed is not None)
        self.manual_target = None
        if faulted:
            return Command(None, source="fault")
        if hazard is not None and hazard.coverage > 0:
            old = self.latched
            if old is None:
                self.latched = hazard
            elif hazard.coverage > old.coverage or (hazard.emergency and not old.emergency):
                self.latched = Hazard(max(hazard.coverage, old.coverage),
                                      hazard.emergency or old.emergency, hazard.reason)
        if self.latched is not None:
            # Hazard close beats a dashboard OPEN; a request may still close further.
            h = self.latched
            want = h.coverage if request is None else max(h.coverage, request)
            return Command(want, h.emergency, "hazard")
        if request is not None:
            return Command(request, source="request")
        return Command(0.0)  # the testbed rests open


def state_of(ctrl, source: str) -> tuple[str, str]:
    """(state, reason) from the controller and the command source."""
    if ctrl.fault:
        return FAULT, ctrl.fault
    if source == "manual":
        return MANUAL, ""
    if ctrl.mode == "homing":
        return HOMING, ""
    if ctrl.zero is None:
        return UNHOMED, ""
    if ctrl.mode == "moving":
        return (CLOSING if ctrl.direction > 0 else OPENING), source
    cov = ctrl.coverage
    if cov is None:
        return PARTIAL, "travel not calibrated"
    return (OPEN if cov <= 0.02 else CLOSED if cov >= 0.98 else PARTIAL), ""
