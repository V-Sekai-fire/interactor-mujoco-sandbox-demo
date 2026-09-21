#!/usr/bin/env python3
"""Check the riscv64 MuJoCo package links, not merely that files exist.

An archive that is present but cannot satisfy a link is the failure worth
catching, so this compiles and links a program calling mj_defaultOption and
mj_version and inspects what came out.

    python verify.py
    python verify.py --self-test
"""

import argparse
import pathlib
import subprocess
import sys
import tempfile

ROOT = pathlib.Path(__file__).resolve().parent
SYSROOT = ROOT.parent / "riscv64-sysroot" / "sysroot"
GCC_LIB = SYSROOT / "lib" / "gcc-cross" / "14"
LIB = ROOT / "lib"
INC = ROOT / "include"

ARCHIVES = ["libmujoco.a", "libccd.a", "libtinyxml2.a", "libqhullstatic_r.a",
            "libtinyobjloader.a", "libminiz.a"]

SOURCE = """
#include <stdio.h>
#include <mujoco/mujoco.h>
int main(void) {
    mjOption opt;
    mj_defaultOption(&opt);
    printf("timestep=%g version=%d\\n", opt.timestep, mj_version());
    return 0;
}
"""


def link(out_dir, libs):
    src = out_dir / "probe.c"
    src.write_text(SOURCE, encoding="utf-8")
    out = out_dir / "probe.elf"
    cmd = ["clang", "--target=riscv64-unknown-linux-gnu", "--sysroot=" + str(SYSROOT),
           "-march=rv64g", "-mabi=lp64d", "-I" + str(INC), "-fuse-ld=lld",
           "-B" + str(GCC_LIB), "-L" + str(GCC_LIB), "-static", str(src), "-o", str(out)]
    cmd += [str(LIB / a) for a in libs]
    cmd += ["-lm", "-lstdc++"]
    return subprocess.run(cmd, capture_output=True, text=True), out


def verify():
    failures = []
    for a in ARCHIVES:
        if not (LIB / a).is_file():
            failures.append("missing archive: " + a)
    if not (INC / "mujoco" / "mujoco.h").is_file():
        failures.append("missing header: include/mujoco/mujoco.h")
    if not SYSROOT.is_dir():
        failures.append("the riscv64 sysroot is not placed beside this package")
    if failures:
        return failures

    with tempfile.TemporaryDirectory() as tmp:
        proc, elf = link(pathlib.Path(tmp), ARCHIVES)
        if proc.returncode != 0 or not elf.exists():
            first = next((l for l in proc.stderr.splitlines() if "error" in l), "(no error line)")
            return ["link failed: " + first.strip()]
        data = elf.read_bytes()
        if data[:4] != b"\x7fELF" or int.from_bytes(data[18:20], "little") != 243:
            return ["linked output is not a RISC-V ELF"]
        print("linked a riscv64 executable against MuJoCo (" + str(len(data)) + " bytes)")
    return []


def self_test():
    controls = []

    def control(name, failures, want_fail):
        controls.append((name, bool(failures) == want_fail, failures[:1]))

    control("the package links", verify(), False)

    with tempfile.TemporaryDirectory() as tmp:
        # Dropping libmujoco leaves the mj_* symbols unresolved, so a link that
        # still succeeds would mean the probe was not calling MuJoCo at all.
        proc, _ = link(pathlib.Path(tmp), [a for a in ARCHIVES if a != "libmujoco.a"])
        control("linking without libmujoco is rejected", ["failed"] if proc.returncode else [], True)

        # Not a control: linking with libmujoco alone succeeds, because this
        # probe never reaches the collision code that needs ccd and qhull. The
        # dependencies are shipped because a real guest does reach it.

        # A truncated archive is the shape a corrupted checkout takes, and it
        # must fail rather than link against whatever survived.
        broken = pathlib.Path(tmp) / "libmujoco.a"
        broken.write_bytes((LIB / "libmujoco.a").read_bytes()[: 1 << 16])
        proc = subprocess.run(
            ["clang", "--target=riscv64-unknown-linux-gnu", "--sysroot=" + str(SYSROOT),
             "-march=rv64g", "-mabi=lp64d", "-I" + str(INC), "-fuse-ld=lld",
             "-B" + str(GCC_LIB), "-L" + str(GCC_LIB), "-static",
             str(pathlib.Path(tmp) / "probe.c"), "-o", str(pathlib.Path(tmp) / "b.elf"),
             str(broken), "-lm", "-lstdc++"],
            capture_output=True, text=True)
        control("a truncated archive is rejected", ["failed"] if proc.returncode else [], True)

    for name, ok, detail in controls:
        print(("PASS" if ok else "FAIL") + "  " + name + (f"  [{detail[0]}]" if detail else ""))
    passed = sum(1 for _, ok, _ in controls if ok)
    print(f"{passed}/{len(controls)} controls")
    return 0 if passed == len(controls) else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--self-test", action="store_true")
    args = ap.parse_args()
    if args.self_test:
        return self_test()
    failures = verify()
    for f in failures:
        print("FAIL  " + f)
    if failures:
        return 1
    print("ok: " + str(len(ARCHIVES)) + " archives, headers present, link succeeds")
    return 0


if __name__ == "__main__":
    sys.exit(main())
