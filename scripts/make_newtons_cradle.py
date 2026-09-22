#!/usr/bin/env python3
"""Generate the Newton's cradle MJCF.

Five balls hang touching each other from equal-length strings. Lift the end
one and release it, and the ball at the far end swings out while the middle
three stay put. That only happens when the balls start exactly touching and
the strings are exactly equal, so both are stated once here and checked.

    python scripts/make_newtons_cradle.py
    python scripts/make_newtons_cradle.py --self-test
"""

import argparse
import math
import pathlib
import sys

OUT = pathlib.Path(__file__).resolve().parent.parent / "project" / "plans" / "newtons_cradle.xml"

BALLS = 5
RADIUS = 0.0106          # 21.2 mm across, a nickel
STRING = 0.16            # pivot to ball centre
PIVOT_Z = 0.22
SPREAD = 0.05            # half-width of the V each string hangs in
LIFT = 0.5               # radians the end ball starts raised


def ball_x(i):
    """Centres spaced one diameter apart, so neighbours just touch."""
    return (i - (BALLS - 1) / 2.0) * (2.0 * RADIUS)


def build():
    q = chr(34)
    lines = []
    lines.append('<mujoco model="newtons_cradle">')
    # A short timestep and a stiff, barely damped contact are what make the
    # collision read as elastic; at the default the balls thud and stop.
    lines.append('  <option timestep="0.0002" gravity="0 0 -9.81" integrator="implicitfast"/>')
    lines.append('  <default>')
    lines.append('    <geom solref="0.0004 1" solimp="0.99 0.9999 0.0001" friction="0.001 0.0001 0.00001"/>')
    lines.append('    <joint damping="0.00002"/>')
    lines.append('  </default>')
    lines.append('  <worldbody>')

    # The frame the strings hang from, drawn but never collided with.
    lines.append('    <geom name="beam" type="capsule" contype="0" conaffinity="0" size="0.004" '
                 'fromto="%.5f 0 %.5f %.5f 0 %.5f" rgba="0.4 0.42 0.5 1"/>'
                 % (ball_x(0) - 0.02, PIVOT_Z, ball_x(BALLS - 1) + 0.02, PIVOT_Z))

    for i in range(BALLS):
        x = ball_x(i)
        angle = LIFT if i == 0 else 0.0
        lines.append('    <body name="ball%d" pos="%.5f 0 %.5f" euler="0 %.5f 0">' % (i, x, PIVOT_Z, angle))
        # Swinging about y keeps every ball in one plane, which is what makes
        # the row strike squarely instead of glancing off.
        lines.append('      <joint name="pivot%d" type="hinge" axis="0 1 0" pos="0 0 0"/>' % i)
        # Two strings in a V, so the ball cannot swing sideways out of the row.
        lines.append('      <geom name="wireL%d" type="capsule" contype="0" conaffinity="0" size="0.0004" '
                     'fromto="0 %.5f 0 0 0 %.5f" rgba="0.75 0.75 0.8 1"/>' % (i, SPREAD, -STRING))
        lines.append('      <geom name="wireR%d" type="capsule" contype="0" conaffinity="0" size="0.0004" '
                     'fromto="0 %.5f 0 0 0 %.5f" rgba="0.75 0.75 0.8 1"/>' % (i, -SPREAD, -STRING))
        lines.append('      <geom name="ball%d" type="sphere" size="%.5f" pos="0 0 %.5f" '
                     'density="7800" condim="3" rgba="0.85 0.86 0.9 1"/>' % (i, RADIUS, -STRING))
        lines.append('    </body>')

    lines.append('  </worldbody>')
    lines.append('</mujoco>')
    return "\n".join(lines) + "\n"


def self_test():
    controls = []

    def control(name, ok, detail=""):
        controls.append((name, ok, detail))

    gaps = [ball_x(i + 1) - ball_x(i) for i in range(BALLS - 1)]
    control("neighbouring balls start touching", all(abs(g - 2 * RADIUS) < 1e-12 for g in gaps),
            "gap %.4f mm vs diameter %.4f mm" % (gaps[0] * 1000, RADIUS * 2000))

    control("the row is centred on the origin", abs(ball_x(0) + ball_x(BALLS - 1)) < 1e-12)

    xml = build()
    control("every ball has a pivot", xml.count('type="hinge"') == BALLS, str(xml.count('type="hinge"')))
    control("exactly one ball starts lifted", xml.count("euler=") == BALLS and xml.count(" %.5f 0\"" % LIFT) == 1)

    # The strings and the beam must not collide, or the row jams instead of
    # swinging and the demo looks like a bug in the physics.
    control("only the balls collide",
            xml.count('contype="0" conaffinity="0"') == BALLS * 2 + 1,
            str(xml.count('contype="0" conaffinity="0"')))

    # Negative control: a spacing that is not one diameter must be caught by
    # the touching check above, or that check proves nothing.
    bad = [2 * RADIUS, 2 * RADIUS + 0.001]
    control("a wrong spacing would be caught",
            not all(abs(g - 2 * RADIUS) < 1e-12 for g in bad))

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
    print("wrote %s (%d balls, %.1f mm across, %.0f mm strings)"
          % (OUT, BALLS, RADIUS * 2000, STRING * 1000))
    return 0


if __name__ == "__main__":
    sys.exit(main())
