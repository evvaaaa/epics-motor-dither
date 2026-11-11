#!/usr/bin/env python3
"""Show the live XY motor position, dither ranges, and soft limits.

Shamelessly vibe-coded.
"""

import argparse
import collections
import math
import sys
import threading
import time

import epics
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from matplotlib.colors import LinearSegmentedColormap, PowerNorm
from matplotlib.patches import Circle


# A black reference circle drawn on the plot, in user units
REFERENCE_CIRCLE = {"center": (5.0, 5.0), "radius": 3.0}
# Positions closer together than this are counted as the same position
DWELL_BIN = 0.05
# Longest single time step credited to a position (e.g. if the window stalls)
MAX_DWELL_STEP = 1.0
MAX_TRAIL_POINTS = 20000
# Seconds at one position that gives the strongest red, at minimum
MIN_DWELL_SCALE = 1.0
DWELL_CMAP = LinearSegmentedColormap.from_list(
    "dwell", ["#ffd6d6", "#ff0000", "#7a0000"]
)
# A small oval marker, as unit-scale polygon vertices
OVAL = [
    (math.cos(2 * math.pi * i / 32), 0.55 * math.sin(2 * math.pi * i / 32))
    for i in range(32)
]


class DwellTrail:
    """Accumulates the path taken and how long was spent at each position."""

    def __init__(self, bin_size=DWELL_BIN, max_points=MAX_TRAIL_POINTS):
        self.bin_size = bin_size
        self.dwell = collections.defaultdict(float)
        self.path = collections.deque(maxlen=max_points)

    def add(self, x, y, seconds):
        seconds = min(max(seconds, 0.0), MAX_DWELL_STEP)
        key = (round(x / self.bin_size), round(y / self.bin_size))
        self.dwell[key] += seconds
        self.path.append((x, y))

    def bins(self):
        """Return x, y and dwell seconds arrays, one entry per visited bin."""
        if not self.dwell:
            return np.empty(0), np.empty(0), np.empty(0)
        keys = np.array(list(self.dwell.keys()), dtype=float) * self.bin_size
        return keys[:, 0], keys[:, 1], np.array(list(self.dwell.values()))


FIELDS = {
    "position": ".RBV",
    "target": ".VAL",
    "low": ".LLM",
    "high": ".HLM",
    "dial_low": ".DLLM",
    "dial_high": ".DHLM",
    "units": ".EGU",
    "positive": ":DITHER.PLUS",
    "negative": ":DITHER.MINUS",
    "center": ":DITHER.CEN",
    "state": ":DITHER.STATE",
}


class AxisMonitor:
    def __init__(self, motor):
        self.motor = motor
        self.lock = threading.Lock()
        self.values = {}
        self.connected = set()
        self.pvs = {}
        for key, suffix in FIELDS.items():
            pvname = motor + suffix
            self.pvs[key] = epics.PV(
                pvname,
                auto_monitor=True,
                callback=self.on_value,
                connection_callback=self.on_connection,
            )

    def on_value(self, pvname=None, value=None, **_kwargs):
        with self.lock:
            self.values[pvname] = value

    def on_connection(self, pvname=None, conn=None, **_kwargs):
        with self.lock:
            if conn:
                self.connected.add(pvname)
            else:
                self.connected.discard(pvname)
                self.values.pop(pvname, None)

    def snapshot(self):
        with self.lock:
            return {
                key: self.values.get(pv.pvname) if pv.pvname in self.connected else None
                for key, pv in self.pvs.items()
            }

    def missing(self):
        with self.lock:
            return [
                pv.pvname
                for pv in self.pvs.values()
                if pv.pvname not in self.connected or self.values.get(pv.pvname) is None
            ]


def positive_float(value):
    try:
        number = float(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("must be a finite positive number") from exc
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError("must be a finite positive number")
    return number


def axis_geometry(values):
    """Return readback, configured endpoints, and enabled user soft limits."""
    position = values["position"]
    limits_enabled = values["dial_low"] != 0 or values["dial_high"] != 0
    low, high = (values["low"], values["high"]) if limits_enabled else (None, None)
    if not math.isfinite(position):
        raise ValueError("Non-finite motor readback")
    if limits_enabled and (
        not math.isfinite(low) or not math.isfinite(high) or low >= high
    ):
        raise ValueError("Invalid motor soft limits")
    center = values["center"]
    negative, positive = values["negative"], values["positive"]
    endpoints = None
    if (
        all(math.isfinite(v) for v in (center, negative, positive))
        and negative >= 0
        and positive >= 0
    ):
        start, end = center - negative, center + positive
        if limits_enabled:
            start, end = max(start, low), min(end, high)
        if math.isfinite(start) and math.isfinite(end) and start <= end:
            endpoints = (start, end)
    return position, endpoints, (low, high)


class TrajectoryPlot:
    def __init__(self, motors, interval):
        self.monitors = [AxisMonitor(motor) for motor in motors]
        self.figure, self.axis = plt.subplots(figsize=(12, 8))
        self.axis.set_title("Manipulator XY position and configured dither range")
        self.axis.grid(True, alpha=0.3)
        self.trail = DwellTrail()
        self.last_sample = None
        (self.path_line,) = self.axis.plot(
            [], [], color="red", alpha=0.25, linewidth=1, zorder=2
        )
        self.dwell_points = self.axis.scatter(
            [],
            [],
            c=[],
            cmap=DWELL_CMAP,
            norm=PowerNorm(0.5, vmin=0, vmax=1),
            marker="s",
            s=18,
            linewidths=0,
            zorder=3,
            label="Time spent (redder = longer)",
        )
        self.reference = Circle(
            REFERENCE_CIRCLE["center"],
            REFERENCE_CIRCLE["radius"],
            fill=False,
            edgecolor="black",
            linewidth=2,
            zorder=4,
        )
        self.axis.add_patch(self.reference)
        self.axis.set_aspect("equal", adjustable="box")
        (self.point,) = self.axis.plot(
            [],
            [],
            linestyle="none",
            marker=OVAL,
            color="red",
            markeredgecolor="darkred",
            markersize=14,
            label="Position (.RBV)",
            zorder=6,
        )
        (self.xbar,) = self.axis.plot(
            [],
            [],
            color="tab:green",
            linewidth=3,
            marker="|",
            markersize=12,
            label="Dither endpoints",
        )
        (self.ybar,) = self.axis.plot(
            [], [], color="tab:green", linewidth=3, marker="_", markersize=12
        )
        self.xlimits = [
            self.axis.axvline(
                0,
                color="tab:red",
                linestyle="--",
                label="Soft limits" if i == 0 else None,
            )
            for i in range(2)
        ]
        self.ylimits = [
            self.axis.axhline(0, color="tab:red", linestyle="--") for _ in range(2)
        ]
        self.axis.legend(loc="upper left")
        self.problem = None
        self.figure.subplots_adjust(left=0.09, right=0.97, bottom=0.10)
        self.animation = FuncAnimation(
            self.figure,
            self.update,
            interval=interval * 1000,
            cache_frame_data=False,
        )

    def wait_connected(self, timeout=5.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            missing = [pv for monitor in self.monitors for pv in monitor.missing()]
            if not missing:
                return
            epics.ca.poll(evt=0.01)
            time.sleep(0.05)
        missing = [pv for monitor in self.monitors for pv in monitor.missing()]
        raise RuntimeError("Cannot connect to or read PVs: " + ", ".join(missing))

    def update(self, _frame):
        snapshots = [monitor.snapshot() for monitor in self.monitors]
        missing = [
            monitor.motor + FIELDS[key]
            for monitor, values in zip(self.monitors, snapshots)
            for key, value in values.items()
            if value is None
        ]
        error = None
        if not missing:
            try:
                geometries = [axis_geometry(values) for values in snapshots]
            except ValueError as exc:
                error = str(exc)
        artists = [self.point, self.xbar, self.ybar, *self.xlimits, *self.ylimits]
        trail_artists = [self.dwell_points, self.path_line]
        if missing or error:
            for artist in artists:
                artist.set_visible(False)
            problem = (
                "Disconnected / waiting for: " + ", ".join(missing)
                if missing
                else error
            )
            if problem != self.problem:
                print(problem, file=sys.stderr)
                self.problem = problem
            self.axis.set_title("Manipulator XY position - data unavailable")
            return tuple(artists)

        (x, xends, xlimits), (y, yends, ylimits) = geometries
        invalid = [
            monitor.motor
            for monitor, geom in zip(self.monitors, geometries)
            if geom[1] is None
        ]
        problem = "Invalid dither range: " + ", ".join(invalid) if invalid else None
        if problem != self.problem:
            if problem:
                print(problem, file=sys.stderr)
            self.problem = problem
        self.axis.set_title(
            "Manipulator XY position and configured dither range"
            + (" - invalid dither range" if invalid else "")
        )
        now = time.monotonic()
        if self.last_sample is not None:
            self.trail.add(x, y, now - self.last_sample)
        self.last_sample = now
        bx, by, seconds = self.trail.bins()
        self.dwell_points.set_offsets(np.column_stack([bx, by]))
        self.dwell_points.set_array(seconds)
        self.dwell_points.norm.vmax = max(
            float(seconds.max()) if seconds.size else 0.0, MIN_DWELL_SCALE
        )
        self.path_line.set_data(
            [p[0] for p in self.trail.path], [p[1] for p in self.trail.path]
        )
        self.point.set_data([x], [y])
        self.point.set_visible(True)
        self.xbar.set_visible(xends is not None)
        self.ybar.set_visible(yends is not None)
        if xends:
            self.xbar.set_data([xends[0], x, xends[1]], [y, y, y])
        if yends:
            self.ybar.set_data([x, x, x], [yends[0], y, yends[1]])
        for line, value in zip(self.xlimits, xlimits):
            line.set_visible(value is not None)
            if value is not None:
                line.set_xdata([value, value])
        for line, value in zip(self.ylimits, ylimits):
            line.set_visible(value is not None)
            if value is not None:
                line.set_ydata([value, value])

        for geom, setter in zip(geometries, (self.axis.set_xlim, self.axis.set_ylim)):
            position, endpoints, limits = geom
            bounds = [position]
            if endpoints:
                bounds.extend(endpoints)
            bounds.extend(value for value in limits if value is not None)
            index = 0 if setter == self.axis.set_xlim else 1
            centre, radius = (
                REFERENCE_CIRCLE["center"][index],
                REFERENCE_CIRCLE["radius"],
            )
            bounds.extend((centre - radius, centre + radius))
            lower, upper = min(bounds), max(bounds)
            padding = max((upper - lower) * 0.08, 0.1)
            setter(lower - padding, upper + padding)
        self.axis.set_xlabel(
            f"{self.monitors[0].motor} ({snapshots[0]['units'] or 'user units'})"
        )
        self.axis.set_ylabel(
            f"{self.monitors[1].motor} ({snapshots[1]['units'] or 'user units'})"
        )
        return tuple(artists + trail_artists)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--motors",
        nargs=2,
        default=["DEMO:m1", "DEMO:m2"],
        metavar=("MOTOR1", "MOTOR2"),
        help="two motor record names",
    )
    parser.add_argument(
        "--interval",
        type=positive_float,
        default=0.1,
        help="plot sampling interval in seconds",
    )
    args = parser.parse_args()
    if matplotlib.get_backend().lower() in {
        "agg",
        "pdf",
        "ps",
        "svg",
        "cairo",
        "template",
    }:
        print(
            "A graphical Matplotlib backend is required; check DISPLAY and GUI support.",
            file=sys.stderr,
        )
        return 1
    plot = TrajectoryPlot(args.motors, args.interval)
    try:
        plot.wait_connected()
    except RuntimeError as exc:
        plt.close(plot.figure)
        print(exc, file=sys.stderr)
        return 1
    plt.show()
    return 0


if __name__ == "__main__":
    sys.exit(main())
