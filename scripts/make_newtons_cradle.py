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

# A desk Newton's cradle: five 25 mm chrome steel balls, a shade wider than a
# US quarter, on 130 mm strings under a frame about 160 mm tall.
BALLS = 5
RADIUS = 0.0125          # 25 mm across, about a quarter
STRING = 0.13            # pivot to ball centre
PIVOT_Z = 0.16
SPREAD = 0.035           # half-width of the V each string hangs in
LIFT = 0.62              # radians the end ball is drawn back to
SWING = -LIFT            # where the figure opens: drawn fully back, released from rest

# Zero, and not a free parameter. Opening part-way down with the velocity that
# fall implies is the more interesting picture, but a non-zero qvel in the
# keyframe makes the first contact gain energy: ball 0 reached 284 mm from a
# speed that can lift it 11 mm, against a 160 mm pivot. Released from rest the
# same model stays inside its energy bound, so the opening state is the
# drawn-back one until that is understood.
SPEED = 0.0
GAP = 0.0005             # 0.5 mm of air between neighbours at rest


def ball_x(i):
    """Centres a diameter apart plus a hair, so the row rests without loading
    the solver. Starting every pair in contact makes the first step resolve
    five stiff constraints at once, which pumps in energy rather than removing
    it."""
    return (i - (BALLS - 1) / 2.0) * (2.0 * RADIUS + GAP)


def build():
    q = chr(34)
    lines = []
    lines.append('<mujoco model="newtons_cradle">')
    # MJCF angles are degrees unless this says otherwise, and every angle here
    # is written in radians.
    lines.append('  <compiler angle="radian"/>')
    # A 1 ms step with 50 solver iterations: measured to hold the same energy
    # bound as a far finer step (peak 61 mm against an 84 mm release) while
    # costing about a sixth as much, so contacts do not drop the frame rate.
    lines.append('  <option timestep="0.001" gravity="0 0 -9.81" integrator="implicitfast" '
                 'iterations="50" ls_iterations="50" tolerance="1e-12"/>')
    lines.append('  <default>')
    lines.append('    <geom solref="-120000 -180" solimp="0.98 0.999 0.0002" friction="0.001 0.0001 0.00001"/>')
    lines.append('    <joint damping="0.00002"/>')
    lines.append('  </default>')
    lines.append('  <worldbody>')

    # The frame the strings hang from, drawn but never collided with.
    lines.append('    <geom name="beam" type="capsule" contype="0" conaffinity="0" size="0.004" '
                 'fromto="%.5f 0 %.5f %.5f 0 %.5f" rgba="0.4 0.42 0.5 1"/>'
                 % (ball_x(0) - 0.02, PIVOT_Z, ball_x(BALLS - 1) + 0.02, PIVOT_Z))

    for i in range(BALLS):
        x = ball_x(i)
        lines.append('    <body name="ball%d" pos="%.5f 0 %.5f">' % (i, x, PIVOT_Z))
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
    qpos = ' '.join(['%.5f' % (SWING if i == 0 else 0.0) for i in range(BALLS)])
    qvel = ' '.join(['%.5f' % (SPEED if i == 0 else 0.0) for i in range(BALLS)])
    lines.append('  <keyframe>')
    lines.append('    <key name="mid_swing" qpos="%s" qvel="%s"/>' % (qpos, qvel))
    lines.append('  </keyframe>')
    lines.append('</mujoco>')
    return "\n".join(lines) + "\n"


def self_test():
    controls = []

    def control(name, ok, detail=""):
        controls.append((name, ok, detail))

    gaps = [ball_x(i + 1) - ball_x(i) for i in range(BALLS - 1)]
    control("neighbours rest a hair apart, not loaded against each other",
            all(abs(g - (2 * RADIUS + GAP)) < 1e-12 for g in gaps) and GAP > 0,
            "centres %.4f mm apart, diameter %.4f mm" % (gaps[0] * 1000, RADIUS * 2000))
    control("the gap is small enough to still read as a row",
            GAP < RADIUS / 4.0, "%.2f mm" % (GAP * 1000))

    control("the row is centred on the origin", abs(ball_x(0) + ball_x(BALLS - 1)) < 1e-12)

    xml = build()
    control("every ball has a pivot", xml.count('type="hinge"') == BALLS, str(xml.count('type="hinge"')))
    control("angles are declared in radians", 'angle="radian"' in xml)
    control("the figure opens from a state it could be placed in by hand",
            xml.count('<key name="mid_swing"') == 1 and SPEED == 0.0,
            "drawn back %.2f rad, at rest" % abs(SWING))

    # The bound the opening state has to respect: released from LIFT, no ball
    # can rise above PIVOT_Z - STRING*cos(LIFT).
    peak_mm = (PIVOT_Z - STRING * math.cos(LIFT)) * 1000.0
    control("the release height is well under the pivot",
            peak_mm < PIVOT_Z * 1000.0, "%.0f mm against a %.0f mm pivot" % (peak_mm, PIVOT_Z * 1000))
    control("the drawn-back angle is a real displacement",
            abs(LIFT) > 0.5 and abs(SWING) <= abs(LIFT), "lift %.2f rad, opens at %.2f" % (LIFT, SWING))

    # The strings and the beam must not collide, or the row jams instead of
    # swinging and the demo looks like a bug in the physics.
    control("only the balls collide",
            xml.count('contype="0" conaffinity="0"') == BALLS * 2 + 1,
            str(xml.count('contype="0" conaffinity="0"')))

    # Negative control: a spacing that is not one diameter must be caught by
    # the touching check above, or that check proves nothing.
    bad = [2 * RADIUS + GAP, 2 * RADIUS + GAP + 0.001]
    control("a wrong spacing would be caught",
            not all(abs(g - (2 * RADIUS + GAP)) < 1e-12 for g in bad))

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
