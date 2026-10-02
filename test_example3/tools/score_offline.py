"""Score saved measurements without FreeCAD, for tuning scoring.py.

Usage (from FreeCAD/4_sling_lift):
    python ../../tools/score_offline.py measure_out            # table
    python ../../tools/score_offline.py measure_out -v         # + reasons

Each measure_out/<name>/measure.json is scored against
measure_out/solution/measure.json as the reference.
"""
import glob
import json
import os
import sys

sys.path.insert(0, os.path.join("tests", "task", "harness"))
import scoring                                            # noqa: E402

SHORT = {
    "executes and builds geometry": "gate",
    "geometry matches ground truth": "geom",
    "pantograph joints mated": "mated",
    "mechanism articulates": "artic",
    "carries dimensional constraints": "cons",
    "dimensions drive the geometry": "drive",
    "signature dimensions present": "sig",
}

folder = sys.argv[1] if len(sys.argv) > 1 and not sys.argv[1].startswith("-") else "measure_out"
verbose = "-v" in sys.argv
ref = json.load(open(os.path.join(folder, "solution", "measure.json"), encoding="utf-8"))

rows = []
for f in sorted(glob.glob(os.path.join(folder, "*", "measure.json"))):
    name = os.path.basename(os.path.dirname(f)).replace("adversarial_", "")
    res = scoring.score_all(json.load(open(f, encoding="utf-8")), ref)
    total = sum(s for n, (s, _) in res.items() if n != scoring.GATE) \
        if res[scoring.GATE][0] else 0.0
    rows.append((name, res, total, f))

w = max(len(r[0]) for r in rows)
rows = [r for r in rows]
print("%-*s  %s  total" % (w, "model", "  ".join("%5s" % s for s in SHORT.values())))
for name, res, total, _ in rows:
    print("%-*s  %s  %4.2f" % (w, name, "  ".join("%5.2f" % res[n][0] for n in SHORT), total))
if verbose:
    for name, res, total, path in rows:
        print("\n== %s  (%.2f / 6)" % (name, total))
        for n in SHORT:
            print("  [%4.2f] %s: %s" % (res[n][0], n, res[n][1]))
        m = scoring.unwrap(json.load(open(path, encoding="utf-8")))
        rep = scoring.mismatch_report(m, scoring.unwrap(ref))
        if rep:
            print("  -- parts that did not match exactly:")
            for line in rep:
                print("     " + line)