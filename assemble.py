#!/usr/bin/env python3
"""Assemble or verify the riscv64-linux-gnu sysroot from pinned Debian packages.

The sysroot is committed, so this is not needed to use it. It exists so the
committed tree can be reproduced and checked against its sources rather than
trusted.

    python assemble.py --verify      # the committed tree is complete
    python assemble.py --assemble    # rebuild it from pins.txt
    python assemble.py --self-test   # controls
"""

import argparse
import hashlib
import io
import pathlib
import shutil
import sys
import tarfile
import urllib.request

ROOT = pathlib.Path(__file__).resolve().parent
PINS = ROOT / "pins.txt"
SYSROOT = ROOT / "sysroot"
DEBS = ROOT / "_debs"
MIRROR = "https://deb.debian.org/debian/"

# Only these two prefixes hold sysroot content; the rest of a deb is docs.
PREFIXES = ("usr/riscv64-linux-gnu/", "usr/lib/gcc-cross/riscv64-linux-gnu/")

# A sysroot missing any of these cannot build or link a C/C++ program, so their
# absence is the cheapest honest check that the tree is usable.
REQUIRED = [
    "include/stdlib.h",
    "include/math.h",
    "include/stdio.h",
    "lib/crt1.o",
    "lib/crti.o",
    "lib/libc.so",
    "lib/libc.so.6",
    "lib/libm.a",
    "lib/gcc-cross/14/libgcc.a",
    "lib/gcc-cross/14/crtbegin.o",
]

# Debian ships libc.so as an ld script holding absolute paths that only resolve
# on a Debian host. Rewritten sysroot-relative, or every link fails.
BAD_PREFIX = "/usr/riscv64-linux-gnu/lib/"
GOOD_PREFIX = "/lib/"


def read_pins():
    rows = []
    for line in PINS.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        pkg, ver, sha, size, path = line.split()
        rows.append((pkg, ver, sha, int(size), path))
    return rows


def ar_members(data):
    if data[:8] != b"!<arch>\n":
        raise ValueError("not an ar archive")
    off = 8
    while off + 60 <= len(data):
        hdr = data[off : off + 60]
        name = hdr[0:16].decode("ascii", "replace").strip().rstrip("/")
        size = int(hdr[48:58].decode("ascii").strip())
        yield name, data[off + 60 : off + 60 + size]
        off += 60 + size + (size & 1)


def fetch(rows):
    DEBS.mkdir(exist_ok=True)
    for pkg, _ver, sha, _size, path in rows:
        out = DEBS / (pkg + ".deb")
        if not out.exists():
            urllib.request.urlretrieve(MIRROR + path, out)
        got = hashlib.sha256(out.read_bytes()).hexdigest()
        if got != sha:
            raise SystemExit(f"{pkg}: sha256 {got} does not match the pin {sha}")
    return len(rows)


def extract(rows):
    if SYSROOT.exists():
        shutil.rmtree(SYSROOT)
    count = 0
    for pkg, *_ in rows:
        data = (DEBS / (pkg + ".deb")).read_bytes()
        for name, body in ar_members(data):
            if not name.startswith("data.tar"):
                continue
            mode = "r:xz" if name.endswith(".xz") else "r:gz" if name.endswith(".gz") else "r:*"
            with tarfile.open(fileobj=io.BytesIO(body), mode=mode) as tf:
                for m in tf.getmembers():
                    p = m.name.lstrip("./")
                    pref = next((x for x in PREFIXES if p.startswith(x)), None)
                    if pref is None:
                        continue
                    rel = ("lib/gcc-cross/" + p[len(pref):]) if "gcc-cross" in pref else p[len(pref):]
                    if not rel:
                        continue
                    dest = SYSROOT / rel
                    if m.isdir():
                        dest.mkdir(parents=True, exist_ok=True)
                    elif m.isfile():
                        dest.parent.mkdir(parents=True, exist_ok=True)
                        f = tf.extractfile(m)
                        if f:
                            dest.write_bytes(f.read())
                            count += 1
    return count


def relocate_scripts(root=SYSROOT):
    fixed = []
    for p in root.rglob("*"):
        if not p.is_file() or p.stat().st_size > 8192:
            continue
        try:
            t = p.read_text(encoding="utf-8")
        except (UnicodeDecodeError, OSError):
            continue
        if BAD_PREFIX not in t or ("GROUP" not in t and "INPUT" not in t):
            continue
        p.write_text(t.replace(BAD_PREFIX, GOOD_PREFIX), encoding="utf-8", newline="\n")
        fixed.append(str(p.relative_to(root)))
    return fixed


def verify(root=SYSROOT):
    failures = []
    if not root.is_dir():
        return [f"{root} does not exist"]
    for rel in REQUIRED:
        if not (root / rel).is_file():
            failures.append(f"missing: {rel}")
    # An unrelocated ld script links on a Debian host and nowhere else, so it
    # has to be a failure rather than a note.
    for p in root.rglob("*"):
        if not p.is_file() or p.stat().st_size > 8192:
            continue
        try:
            t = p.read_text(encoding="utf-8")
        except (UnicodeDecodeError, OSError):
            continue
        if BAD_PREFIX in t:
            failures.append(f"unrelocated absolute path in {p.relative_to(root)}")
    return failures


def self_test():
    import tempfile

    controls = []

    def control(name, failures, want_fail):
        ok = bool(failures) == want_fail
        controls.append((name, ok, failures[:1]))

    control("the committed sysroot verifies", verify(), False)

    with tempfile.TemporaryDirectory() as tmp:
        work = pathlib.Path(tmp) / "sysroot"
        work.mkdir()
        for rel in REQUIRED:
            f = work / rel
            f.parent.mkdir(parents=True, exist_ok=True)
            f.write_bytes(b"x")
        control("a tree holding every required file verifies", verify(work), False)

        (work / "lib/crt1.o").unlink()
        control("a missing startup file is rejected", verify(work), True)
        (work / "lib/crt1.o").write_bytes(b"x")

        (work / "lib/libc.so").write_text(
            "GROUP ( " + BAD_PREFIX + "libc.so.6 )", encoding="utf-8"
        )
        control("an unrelocated ld script is rejected", verify(work), True)

        relocate_scripts(work)
        control("relocating it makes the tree verify", verify(work), False)

        control("a tree that does not exist is rejected", verify(work / "nope"), True)

    for name, ok, detail in controls:
        print(("PASS" if ok else "FAIL") + "  " + name + (f"  [{detail[0]}]" if detail else ""))
    passed = sum(1 for _, ok, _ in controls if ok)
    print(f"{passed}/{len(controls)} controls")
    return 0 if passed == len(controls) else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--assemble", action="store_true")
    ap.add_argument("--verify", action="store_true")
    ap.add_argument("--self-test", action="store_true")
    args = ap.parse_args()

    if args.self_test:
        return self_test()

    if args.assemble:
        rows = read_pins()
        print(f"{fetch(rows)} packages verified against pins.txt")
        print(f"{extract(rows)} files extracted")
        print(f"relocated: {', '.join(relocate_scripts()) or 'nothing'}")

    failures = verify()
    for f in failures:
        print("FAIL  " + f)
    if failures:
        print(f"{len(failures)} failure(s)")
        return 1
    print(f"sysroot ok: {len(REQUIRED)} required paths present, no unrelocated scripts")
    return 0


if __name__ == "__main__":
    sys.exit(main())
