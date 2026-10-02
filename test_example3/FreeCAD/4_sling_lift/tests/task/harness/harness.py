#!/usr/bin/env python3
"""Grading harness for 4_sling_lift.

    python3 tests/task/harness/harness.py solution/solution.FCStd

The split is the one the stub set up: `measure_stage.py` runs inside
freecadcmd and writes what it measured; this file never imports FreeCAD.
The scoring itself lives in `scoring.py` (pure Python) so that it can be
run and tuned against saved measurements -- see tools/score_offline.py.

The candidate is compared with the reference document in
tests/task/reference/. Its measurement is cached next to this file,
keyed by the hashes of the reference and of measure_stage.py, so the
reference is only measured once per stage version.
"""
import hashlib
import json
import os
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent


def _repo_root(start):
    d = start
    for _ in range(8):
        if (d / "common" / "__init__.py").is_file():
            return d
        if d.parent == d:
            break
        d = d.parent
    return start


REPO = _repo_root(HERE)
sys.path[:0] = [str(HERE), str(REPO), "/opt"]

from common.harness_base import Harness                   # noqa: E402
import scoring                                            # noqa: E402

REFERENCE = HERE.parent / "reference" / "solution.FCStd"
CACHE_DIR = HERE / ".reference_cache"

#: Weight per component; 0 makes it a gate when it is also in MUST_PASS.
#: The sum must equal task.toml's max_score (6).
ALL_CRITERIA = {
    "executes and builds geometry": 0,
    "geometry matches ground truth": 1,
    "pantograph joints mated": 1,
    "mechanism articulates": 1,
    "carries dimensional constraints": 1,
    "dimensions drive the geometry": 1,
    "signature dimensions present": 1,
}


def _sha(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


class SlingLiftHarness(Harness):
    MUST_PASS = ("executes and builds geometry",)
    WEIGHTS = ALL_CRITERIA
    BUILD_TIMEOUT_S = 1200

    def _measure(self, path):
        try:
            measurement, error = self.run_freecad_stage(str(path))
        except SystemExit as exc:
            measurement, error = None, str(exc)
        except Exception as exc:                            # noqa: BLE001
            measurement, error = None, f"{type(exc).__name__}: {exc}"
        return measurement or {}, error

    def reference_measurement(self):
        override = os.environ.get("SLING_REF_MEASURE")
        if override:
            return json.loads(Path(override).read_text(encoding="utf-8")), None
        if not REFERENCE.is_file():
            return {}, f"reference document not found at {REFERENCE}"
        key = (_sha(REFERENCE)[:16] + "-" + _sha(HERE / "measure_stage.py")[:16])
        cached = CACHE_DIR / f"{key}.json"
        if cached.is_file():
            return json.loads(cached.read_text(encoding="utf-8")), None
        meas, err = self._measure(REFERENCE)
        if err is None and meas:
            try:                       # a read-only checkout just skips the cache
                CACHE_DIR.mkdir(exist_ok=True)
                cached.write_text(json.dumps(meas), encoding="utf-8")
            except OSError:
                pass
        return meas, err

    def build_state(self, candidate_path):
        measurement, error = self._measure(candidate_path)
        ref, ref_error = self.reference_measurement()
        return {"candidate": candidate_path, "measurement": measurement,
                "error": error, "reference": ref, "reference_error": ref_error}

    def checks(self, state):
        if state.get("reference_error"):
            msg = "reference could not be measured: %s" % state["reference_error"]
            res = {n: (0.0, msg) for n in ALL_CRITERIA}
        else:
            res = scoring.score_all(state["measurement"], state["reference"],
                                    state["error"])
        return {name: (name, float(res[name][0]), res[name][1])
                for name in ALL_CRITERIA}


main = SlingLiftHarness.as_main()

if __name__ == "__main__":
    SlingLiftHarness.cli()