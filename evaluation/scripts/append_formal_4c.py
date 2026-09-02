#!/usr/bin/env python3
"""Append a Phase-4C object's VALID closed-loop trials from real_manip_results.csv
to the clean real_manip_4c_formal.csv, relabeling placement_id / repeat in order.

A trial is VALID iff its note contains a measured-servo token `<N>meas` with N>0.
That single rule:
  * keeps the real closed-loop grasp attempts (e.g. "servo 8meas 26->3.9mm conv"),
  * drops UNMEASURED-openloop rows (0meas == equipment-down, marker not seen --
    NOT a grasp attempt), and
  * drops the older non-servo pilot rows (they have no `meas` token at all).

Dry-run by default (prints what it WOULD append + the k/N); pass --append to write.
Refuses to double-append an object already present in the formal file (use --force).

  python3 append_formal_4c.py --object banana                 # preview
  python3 append_formal_4c.py --object banana --append        # write
  python3 append_formal_4c.py --object coke_can --placements 2 # 2x3 = 6 trials
"""
import argparse
import csv
import os
import re

HERE = os.path.dirname(os.path.abspath(__file__))
RES = os.path.normpath(os.path.join(HERE, "..", "results"))
MEAS = re.compile(r"(\d+)\s*meas")


def valid_rows(src_rows, obj):
    out = []
    for r in src_rows:
        if r.get("object") != obj:
            continue
        m = MEAS.search(r.get("note") or "")
        if m and int(m.group(1)) > 0:
            out.append(r)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--object", required=True)
    ap.add_argument("--src", default=os.path.join(RES, "real_manip_results.csv"))
    ap.add_argument("--formal", default=os.path.join(RES, "real_manip_4c_formal.csv"))
    ap.add_argument("--placements", type=int, default=5)
    ap.add_argument("--repeats", type=int, default=3)
    ap.add_argument("--append", action="store_true")
    ap.add_argument("--force", action="store_true",
                    help="append even if the object is already in the formal file")
    args = ap.parse_args()

    with open(args.src) as f:
        reader = csv.DictReader(f)
        fields = reader.fieldnames
        src_rows = list(reader)

    rows = valid_rows(src_rows, args.object)
    if not rows:
        print(f"no valid (meas>0) {args.object} rows in {os.path.basename(args.src)}")
        return

    existing = []
    if os.path.exists(args.formal):
        with open(args.formal) as f:
            existing = list(csv.DictReader(f))
    already = [r for r in existing if r.get("object") == args.object]
    if already and not args.force:
        print(f"REFUSING: {len(already)} {args.object} rows already in the formal file. "
              f"Re-run with --force only if you really mean to add more.")
        return

    cap = args.placements * args.repeats
    if len(rows) != cap:
        print(f"WARNING: {len(rows)} valid rows != {args.placements}x{args.repeats}={cap} "
              f"expected -- relabeling sequentially anyway; check the run is complete.")

    out, k = [], 0
    for i, r in enumerate(rows):
        nr = dict(r)
        nr["placement_id"] = i // args.repeats + 1
        nr["repeat"] = i % args.repeats + 1
        out.append(nr)
        k += int(r.get("lift_success") or 0)

    print(f"{args.object}: {len(out)} valid trials, lift {k}/{len(out)} = "
          f"{100*k/len(out):.0f}%")
    for nr in out:
        print(f"  p{nr['placement_id']} r{nr['repeat']}  lift={nr['lift_success']}  {nr['note']}")

    if not args.append:
        print("\n(dry-run; pass --append to write to the formal file)")
        return

    write_header = (not os.path.exists(args.formal)) or os.path.getsize(args.formal) == 0
    with open(args.formal, "a", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        if write_header:
            w.writeheader()
        for nr in out:
            w.writerow(nr)
    print(f"\nappended {len(out)} rows to {os.path.basename(args.formal)}")


if __name__ == "__main__":
    main()
