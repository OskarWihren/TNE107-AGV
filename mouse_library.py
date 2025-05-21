#!/usr/bin/env python3
"""
relmouse.py – Non‑blocking helpers for reading relative mouse motion.

Public API:
-----------
open_mouse(dev_path="/dev/input/event2", grab=True)   -> evdev.InputDevice
read_motion_delta(device)                             -> MouseDelta
"""

from dataclasses import dataclass
import evdev
import evdev.ecodes as ev


# ────────────────────────────  data container  ────────────────────────────
@dataclass(frozen=True, slots=True)
class MouseDelta:
    """Immutable X/Y displacement returned by read_motion_delta."""
    dx: int
    dy: int


# ────────────────────────────  public API  ────────────────────────────
def open_mouse(dev_path="/dev/input/event2", grab=True):
    """
    Open *dev_path* (default /dev/input/event2) and return an InputDevice
    set to non‑blocking mode.
    """
    dev = evdev.InputDevice(dev_path)
    dev.read()  # flush any stale events
    if grab:
        dev.grab()
    return dev


def _get_mousemove_events(device):
    """Yield all EV_REL / (REL_X, REL_Y) events since the previous read()."""
    try:
        for e in device.read():
            if e.type == ev.EV_REL and e.code in (ev.REL_X, ev.REL_Y):
                yield e
    except IOError:
        return


def _sum_motion(events):
    """Aggregate deltas in *events* and return a MouseDelta instance."""
    dx = dy = 0
    for e in events:
        if e.code == ev.REL_X:
            dx += e.value
        elif e.code == ev.REL_Y:
            dy += e.value
    return MouseDelta(dx, dy)


def read_motion_delta(device):
    """Return total MouseDelta since the last call (non‑blocking)."""
    return _sum_motion(_get_mousemove_events(device))


# ────────────────────────────  demo / self‑test  ────────────────────────────
if __name__ == "__main__":
    mouse = open_mouse()           # defaults to /dev/input/event2

    mouse_x = 40
    mouse_y = 40
    scale_factor = 0.002395209581
    import sys
    import time
    
    mouse_x = 0
    mouse_y = 0
    delta = read_motion_delta(mouse)
    if delta.dx != 0 or delta.dy != 0:
        print(f"Mouse moved: Δx={delta.dx}, Δy={delta.dy}")
    delta_x = delta.dx
    delta_y = delta.dy
    delta_x = delta_x*scale_factor
    delta_y = delta_y*scale_factor
    mouse_x += delta_x
    mouse_y += delta_y

    data = mouse_x,mouse_y
    print(data)
