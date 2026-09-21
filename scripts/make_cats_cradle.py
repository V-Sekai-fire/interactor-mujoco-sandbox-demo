#!/usr/bin/env python3
"""Generate the cat's cradle MJCF from the figure's path.

The cradle is one closed loop of string whose two diagonals cross between the
hands. Writing 24 woven capsules by hand invites a silent mistake in the weave,
so the path is stated once as waypoints and the segments are resampled from it.

    python scripts/make_cats_cradle.py            # write the model
    python scripts/make_cats_cradle.py --self-test
"""

import argparse
import math
import pathlib
import sys

OUT = pathlib.Path(__file__).resolve().parent.parent / "project" / "plans" / "cats_cradle.xml"

SEGMENTS = 24
RADIUS = 0.003          # 3 mm, about two stacked credit cards
HAND_X = 0.15
FINGER_Y = 0.05
HEIGHT = 0.30
CROSS = 0.012           # how far the two diagonals sit apart where they cross

# The loop, as the figure is actually held: each diagonal runs from one hand's
# thumb to the other hand's index finger, so the two cross between the hands.
# Separating them in z is what makes one rest on the other instead of sharing a
# point.
WAYPOINTS = [
    ("left_thumb",  (-HAND_X, -FINGER_Y, HEIGHT)),
    (None,          (0.0, 0.0, HEIGHT - CROSS)),
    ("right_index", (+HAND_X, +FINGER_Y, HEIGHT)),
    ("right_thumb", (+HAND_X, -FINGER_Y, HEIGHT)),
    (None,          (0.0, 0.0, HEIGHT + CROSS)),
    ("left_index",  (-HAND_X, +FINGER_Y, HEIGHT)),
]


def sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def dist(a, b):
    return math.sqrt(sum(c * c for c in sub(a, b)))


def resample(points, n):
    """n points evenly spaced along the closed polyline through points."""
    closed = list(points) + [points[0]]
    edges = [dist(closed[i], closed[i + 1]) for i in range(len(points))]
    total = sum(edges)
    if total == 0.0:
        return [tuple(points[0])] * n
    step = total / n

    out = []
    for k in range(n):
        target = k * step
        acc = 0.0
        for i, length in enumerate(edges):
            if acc + length >= target or i == len(edges) - 1:
                t = (target - acc) / length if length else 0.0
                a, b = closed[i], closed[i + 1]
                out.append((a[0] + t * (b[0] - a[0]),
                            a[1] + t * (b[1] - a[1]),
                            a[2] + t * (b[2] - a[2])))
                break
            acc += length
    return out


def nearest_index(pts, target):
    return min(range(len(pts)), key=lambda i: dist(pts[i], target))


def build():
    path = [p for _, p in WAYPOINTS]
    pts = resample(path, SEGMENTS)

    bodies = []
    for i in range(SEGMENTS):
        a = pts[i]
        b = pts[(i + 1) % SEGMENTS]
        mid = ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2, (a[2] + b[2]) / 2)
        # The capsule is placed by fromto in the body's own frame, so each body
        # keeps an identity rotation and the initial qpos needs no quaternions.
        bodies.append((i, mid, sub(a, mid), sub(b, mid)))

    q = chr(34)
    lines = []
    lines.append('<mujoco model="cats_cradle">')
    lines.append('  <option timestep="0.002" gravity="0 0 -9.81" integrator="implicitfast"/>')
    lines.append('  <default>')
    lines.append('    <geom type="capsule" size="%s" density="1200" friction="0.6 0.005 0.0001" condim="3"/>' % RADIUS)
    lines.append('  </default>')
    lines.append('  <worldbody>')
    for name, p in WAYPOINTS:
        if name:
            lines.append('    <site name="%s" pos="%.5f %.5f %.5f" size="0.006"/>' % (name, p[0], p[1], p[2]))
    for i, mid, la, lb in bodies:
        lines.append('    <body name="seg%d" pos="%.5f %.5f %.5f">' % (i, mid[0], mid[1], mid[2]))
        lines.append('      <freejoint/>')
        lines.append('      <geom name="g%d" fromto="%.5f %.5f %.5f %.5f %.5f %.5f"/>'
                     % (i, la[0], la[1], la[2], lb[0], lb[1], lb[2]))
        lines.append('    </body>')
    lines.append('  </worldbody>')

    lines.append('  <equality>')
    for i, mid, la, lb in bodies:
        j = (i + 1) % SEGMENTS
        lines.append('    <connect name="link%d" body1="seg%d" body2="seg%d" anchor="%.5f %.5f %.5f"/>'
                     % (i, i, j, lb[0], lb[1], lb[2]))
    # Four fingers hold the figure up. Without these the loop is a free rope.
    for name, p in WAYPOINTS:
        if not name:
            continue
        k = nearest_index([b[1] for b in bodies], p)
        anchor = sub(p, bodies[k][1])
        lines.append('    <connect name="hold_%s" body1="seg%d" anchor="%.5f %.5f %.5f"/>'
                     % (name, k, anchor[0], anchor[1], anchor[2]))
    lines.append('  </equality>')

    # Neighbours always touch; leaving them in makes every step report contacts
    # that say nothing about whether the weave is holding.
    lines.append('  <contact>')
    for i in range(SEGMENTS):
        lines.append('    <exclude body1="seg%d" body2="seg%d"/>' % (i, (i + 1) % SEGMENTS))
        lines.append('    <exclude body1="seg%d" body2="seg%d"/>' % (i, (i + 2) % SEGMENTS))
    lines.append('  </contact>')
    lines.append('</mujoco>')
    return "\n".join(lines) + "\n"


def self_test():
    controls = []

    def control(name, ok, detail=""):
        controls.append((name, ok, detail))

    path = [p for _, p in WAYPOINTS]
    pts = resample(path, SEGMENTS)
    control("resample returns the requested count", len(pts) == SEGMENTS, str(len(pts)))

    # Segment lengths must be near-uniform, or the capsules overlap in places
    # and gap in others while the model still loads.
    lens = [dist(pts[i], pts[(i + 1) % SEGMENTS]) for i in range(SEGMENTS)]
    spread = (max(lens) - min(lens)) / (sum(lens) / len(lens))
    control("segment lengths are within 25% of uniform", spread < 0.25, "%.3f" % spread)

    # The whole point of the figure: the two diagonals must pass each other
    # without sharing a point, or there is no crossing to rest on.
    control("the diagonals are separated where they cross", CROSS > RADIUS * 2,
            "gap %.0f mm vs diameter %.0f mm" % (CROSS * 1000, RADIUS * 2000))

    control("the loop closes", dist(pts[-1], pts[0]) <= max(lens) * 1.5,
            "%.1f mm" % (dist(pts[-1], pts[0]) * 1000))

    # Negative control: a path whose waypoints all coincide must not come back
    # looking like a figure, or the uniformity check above proves nothing.
    degenerate = resample([(0.0, 0.0, 0.0)] * 6, SEGMENTS)
    dl = [dist(degenerate[i], degenerate[(i + 1) % SEGMENTS]) for i in range(SEGMENTS)]
    control("a degenerate path is not mistaken for a figure", max(dl) == 0.0, str(max(dl)))

    xml = build()
    control("every segment is anchored into the loop", xml.count('<connect name="link') == SEGMENTS)
    control("four fingers hold the figure", xml.count('<connect name="hold_') == 4)

    for name, ok, detail in controls:
        print(("PASS" if ok else "FAIL") + "  " + name + ("  [" + detail + "]" if detail else ""))
    passed = sum(1 for _, ok, _ in controls if ok)
    print("%d/%d controls" % (passed, len(controls)))
    return 0 if passed == len(controls) else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--self-test", action="store_true")
    args = ap.parse_args()
    if args.self_test:
        return self_test()
    OUT.write_text(build(), encoding="utf-8", newline="\n")
    print("wrote %s (%d segments, %.0f mm radius)" % (OUT, SEGMENTS, RADIUS * 1000))
    return 0


if __name__ == "__main__":
    sys.exit(main())
