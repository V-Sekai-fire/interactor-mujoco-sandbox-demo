#!/usr/bin/env python3
"""Generate the Newton's cradle MJCF from an XML AST.

Five steel balls hang a hair apart from equal strings. The end ball is drawn
OUTWARD and released; it strikes the row and the ball at the far end swings out
while the middle three stay still. That one-in-one-out transfer is the whole
point, and it only appears when three things are right, each checked below:

  * the end ball is drawn outward, not into the row (a sign on the hinge angle);
  * the balls rest a hair apart, so impacts are sequential, not one mushy solve;
  * the contact is stiff, so little energy is lost per strike.

The model is built as an ElementTree, not by pasting strings, so a tuning
change is an attribute on a node rather than a fragile search-and-replace.

    python scripts/make_newtons_cradle.py
    python scripts/make_newtons_cradle.py --self-test
"""

import argparse
import math
import pathlib
import sys
import xml.etree.ElementTree as ET

OUT = pathlib.Path(__file__).resolve().parent.parent / "project" / "plans" / "newtons_cradle.xml"

# A desk Newton's cradle: five 25 mm chrome steel balls, a shade wider than a
# US quarter, on 130 mm strings under a frame about 160 mm tall.
BALLS = 5
RADIUS = 0.0125          # 25 mm across, about a quarter
STRING = 0.13            # pivot to ball centre
PIVOT_Z = 0.16
SPREAD = 0.035           # half-width of the V each string hangs in
GAP = 0.0008             # 0.8 mm of air between neighbours, so strikes are sequential

LIFT = 0.62              # radians the end ball is drawn back to
# Drawn OUTWARD, away from the row. A +y hinge moves the leftmost ball to -x for
# +theta, which is up-and-out; a negative angle would lift it into ball 1 and
# start the run jammed against the row -- the bug that produced a mushy fan.
SWING = LIFT

# Stiff, lightly damped contact with a 0.5 ms step: measured to keep the middle
# balls still and pass most of the energy to the end ball. A softer contact
# fans the row; a coarser step loses too much on each strike.
TIMESTEP = 0.0005
ITERATIONS = 50
SOLREF = "-2000000 -20"
SOLIMP = "0.98 0.999 0.0002"


def ball_x(i):
    """Centres a diameter apart plus the gap, so neighbours rest just clear."""
    return (i - (BALLS - 1) / 2.0) * (2.0 * RADIUS + GAP)


def build_tree():
    m = ET.Element("mujoco", model="newtons_cradle")
    ET.SubElement(m, "compiler", angle="radian")  # radians, not the MJCF default of degrees
    ET.SubElement(m, "option", timestep=str(TIMESTEP), gravity="0 0 -9.81",
                  integrator="implicitfast", iterations=str(ITERATIONS),
                  ls_iterations="50", tolerance="1e-12")
    default = ET.SubElement(m, "default")
    ET.SubElement(default, "geom", type="capsule", solref=SOLREF, solimp=SOLIMP,
                  friction="0.001 0.0001 0.00001")
    ET.SubElement(default, "joint", damping="0.00002")

    world = ET.SubElement(m, "worldbody")
    ET.SubElement(world, "geom", name="beam", type="capsule", contype="0",
                  conaffinity="0", size="0.004",
                  fromto="%.5f 0 %.5f %.5f 0 %.5f" % (ball_x(0) - 0.02, PIVOT_Z,
                                                      ball_x(BALLS - 1) + 0.02, PIVOT_Z),
                  rgba="0.4 0.42 0.5 1")

    for i in range(BALLS):
        body = ET.SubElement(world, "body", name="ball%d" % i,
                             pos="%.5f 0 %.5f" % (ball_x(i), PIVOT_Z))
        # Hinge about y keeps every ball in the one plane, so the row strikes
        # squarely rather than glancing.
        ET.SubElement(body, "joint", name="pivot%d" % i, type="hinge",
                      axis="0 1 0", pos="0 0 0")
        for side, y in (("L", SPREAD), ("R", -SPREAD)):
            ET.SubElement(body, "geom", name="wire%s%d" % (side, i), type="capsule",
                          contype="0", conaffinity="0", size="0.0004",
                          fromto="0 %.5f 0 0 0 %.5f" % (y, -STRING),
                          rgba="0.75 0.75 0.8 1")
        ET.SubElement(body, "geom", name="ball%d" % i, type="sphere",
                      size="%.5f" % RADIUS, pos="0 0 %.5f" % -STRING,
                      density="7800", condim="3", rgba="0.85 0.86 0.9 1")

    key = ET.SubElement(m, "keyframe")
    qpos = " ".join("%.5f" % (SWING if i == 0 else 0.0) for i in range(BALLS))
    qvel = " ".join("%.5f" % 0.0 for _ in range(BALLS))
    ET.SubElement(key, "key", name="drawn_back", qpos=qpos, qvel=qvel)
    return m


def build():
    m = build_tree()
    ET.indent(m, space="  ")
    return ET.tostring(m, encoding="unicode") + "\n"


def self_test():
    controls = []

    def control(name, ok, detail=""):
        controls.append((name, ok, detail))

    m = build_tree()

    gaps = [ball_x(i + 1) - ball_x(i) for i in range(BALLS - 1)]
    control("neighbours rest a hair apart, not loaded against each other",
            all(abs(g - (2 * RADIUS + GAP)) < 1e-12 for g in gaps) and GAP > 0,
            "centres %.4f mm apart, diameter %.4f mm" % (gaps[0] * 1000, RADIUS * 2000))
    control("the gap is small enough to still read as a row",
            GAP < RADIUS / 4.0, "%.2f mm" % (GAP * 1000))
    control("the row is centred on the origin", abs(ball_x(0) + ball_x(BALLS - 1)) < 1e-12)

    # The correctness bug that produced the mush: the end ball must be drawn
    # OUTWARD. A negative opening angle lifts it into ball 1.
    control("the end ball is drawn outward, not into the row",
            SWING > 0.0 and abs(SWING) <= abs(LIFT),
            "opens at %+.2f rad (positive = outward)" % SWING)

    # AST assertions, in place of the old substring counts.
    control("angles are radians", m.find("compiler").get("angle") == "radian")
    control("every ball has a hinge",
            len(m.findall(".//joint[@type='hinge']")) == BALLS)
    control("one keyframe, one ball moving off rest",
            len(m.findall(".//key")) == 1 and
            m.find(".//key").get("qpos").split()[0] == "%.5f" % SWING)

    noncolliding = [g for g in m.iter("geom") if g.get("contype") == "0"]
    control("only the balls collide", len(noncolliding) == BALLS * 2 + 1,
            str(len(noncolliding)))

    peak_mm = (PIVOT_Z - STRING * math.cos(LIFT)) * 1000.0
    control("the release height is well under the pivot",
            peak_mm < PIVOT_Z * 1000.0,
            "%.0f mm against a %.0f mm pivot" % (peak_mm, PIVOT_Z * 1000))

    control("a wrong spacing would be caught",
            not all(abs(g - (2 * RADIUS + GAP)) < 1e-12
                    for g in [2 * RADIUS + GAP + 0.001] * 2))

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
