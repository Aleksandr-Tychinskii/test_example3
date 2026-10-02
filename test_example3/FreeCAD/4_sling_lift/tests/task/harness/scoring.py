"""Scoring for 4_sling_lift: two measurement dicts in, component scores out.

Pure Python, no FreeCAD -- `harness.py` feeds it what `measure_stage.py`
wrote for the candidate and for the reference, and `tools/score_offline.py`
feeds it saved measure.json files so thresholds can be tuned without
re-running FreeCAD.

What is compared against the reference, and what is not
------------------------------------------------------
Compared: the *shapes* of the parts (volume and principal moments of each
solid -- invariant to pose, placement and reflection) and the handedness of
the assembled lift. A few parts detailed differently from the reference are
tolerated (the reference is one rendering of the drawing); missing parts are
not.
Not compared: names, labels, tree shape, part count per sub-assembly, the
pose the lift was saved in, the position of the lift in space, or how a
candidate chose to split a part into features. Joints, articulation and
parametric structure are judged on what they do, not against the
reference.
"""
import math

#: The mirror question. True: a lift that is a reflection of the drawing
#: (clutch A and B, arm 1 and arm 2 swapped side for side) is a different
#: physical product and loses the arrangement half of the geometry score.
#: False: accept either hand. See NOTES.md for the argument.
MIRROR_IS_DEFECT = True

#: Drawing dimensions the model must carry. Diameters also match as radii.
SIGNATURES = (
    ("pivot pitch 120", 120.0, None),
    ("pin Ø16", 16.0, 8.0),
    ("hole Ø16.2", 16.2, 8.1),
    ("Ø20", 20.0, 10.0),
    ("Ø20.2", 20.2, 10.1),
)
PITCH = 120.0
PITCH_TOL = 0.3            # drawing: 120 +-0.3 on the arms

# tolerances for "this is the same solid"
VOL_TOL = 0.003            # relative
MOM_TOL = 0.01             # relative, each principal moment
# a reference solid with no exact twin may still be "the same part, detailed
# differently" -- a missing chamfer, a different bore -- if its volume is
# within this of an unmatched candidate solid
DETAIL_VOL_TOL = 0.20
# part-level scoring (fractions of the reference's solids / volume)
MISSING_ZERO = 0.10        # 10 % of parts missing -> 0; every missing part costs
DETAIL_FREE = 0.05         # up to 5 % of parts detailed differently is free
DETAIL_ZERO = 0.30
VOL_FREE = 0.01            # up to 1 % of volume misplaced is free
VOL_ZERO = 0.10

# joint mating tolerances
AXIS_TOL_DEG = 0.5
RADIAL_TOL = 0.1
AXIAL_TOL = 0.05
RADIUS_MISMATCH = 1.0      # mm; washer 6.5 on a 7.1 hole is fine, a 8.85 head on a 6.6 hole is not

# a reached drive pose
POSE_D_TOL = 0.5
POSE_SIDE_TOL = 0.5
MIN_SPAN_TRAVEL = 5.0      # mm the arms must move across the band


# ---------------------------------------------------------------- helpers
def clamp01(x):
    return max(0.0, min(1.0, float(x)))


def lin(value, perfect, zero):
    """1 at `perfect`, 0 at `zero`, linear in between (either direction)."""
    if perfect == zero:
        return 1.0 if value == perfect else 0.0
    return clamp01((value - zero) / (perfect - zero))


def unwrap(m):
    if isinstance(m, dict) and isinstance(m.get("measurement"), dict):
        return m["measurement"]
    return m or {}


class UF:
    def __init__(self):
        self.p = {}

    def find(self, x):
        self.p.setdefault(x, x)
        while self.p[x] != x:
            self.p[x] = self.p[self.p[x]]
            x = self.p[x]
        return x

    def union(self, a, b):
        self.p[self.find(a)] = self.find(b)


def rigid_clusters(m):
    uf = UF()
    for c in m.get("components", []):
        uf.find(c.get("path"))
    for j in m.get("joints", []):
        if j.get("kind") == "joint" and j.get("type") == "Fixed" \
                and not j.get("Suppressed"):
            a, b = (x.get("component") for x in j.get("refs", [{}, {}]))
            if a and b:
                uf.union(a, b)
    return uf


def handedness(m):
    mech = m.get("mechanism") or {}
    ch = mech.get("chirality") or {}
    if "sign_upright" in ch:
        return ch["sign_upright"]
    if "sign" in ch and mech.get("upright") is not None:
        return ch["sign"] if mech["upright"] else -ch["sign"]
    return None


# ------------------------------------------------------------- the gate
def check_builds(m, err):
    if err:
        return 0.0, "the measurement stage failed: %s" % str(err)[:300]
    if m.get("ok") is False:
        return 0.0, "the measurement stage raised: %s" % str(m.get("error"))[:300]
    bad = m.get("recompute_errors") or []
    if bad:
        return 0.0, "%d objects fail to recompute (e.g. %s)" % (len(bad), ", ".join(bad[:5]))
    if m.get("error_no_assembly"):
        return 0.0, "no assembly in the document"
    n = sum(len(c.get("solids", [])) for c in m.get("components", []))
    if n == 0:
        return 0.0, "the assembly contains no solids"
    return 1.0, "recomputes cleanly; the assembly holds %d solids" % n


# ------------------------------------------------------------- geometry
def solids_of(m):
    out = []
    for c in m.get("components", []):
        for s in c.get("solids", []):
            out.append({"vol": float(s.get("volume", 0.0)),
                        "mom": sorted(s.get("moments") or []),
                        "label": c.get("label")})
    return out


def same_solid(a, b):
    if abs(a["vol"] - b["vol"]) > VOL_TOL * max(a["vol"], b["vol"], 1e-9):
        return False
    if len(a["mom"]) != 3 or len(b["mom"]) != 3:
        return True
    return all(abs(x - y) <= MOM_TOL * max(abs(x), abs(y), 1.0)
               for x, y in zip(a["mom"], b["mom"]))


def match_solids(cand, ref):
    used = [False] * len(cand)
    matched, missing = [], []
    for rs in sorted(ref, key=lambda s: -s["vol"]):
        best = None
        for i, cs in enumerate(cand):
            if used[i] or not same_solid(cs, rs):
                continue
            d = abs(cs["vol"] - rs["vol"])
            if best is None or d < best[0]:
                best = (d, i)
        if best is None:
            missing.append(rs)
        else:
            used[best[1]] = True
            matched.append((rs, cand[best[1]]))
    extra = [c for i, c in enumerate(cand) if not used[i]]
    return matched, missing, extra


def mismatch_report(m, ref):
    """What did not match exactly, for tuning and for the notes."""
    exact, missing, extra = match_solids(solids_of(m), solids_of(ref))
    detail, gone, surplus = pair_leftovers(missing, extra)
    lines = []
    for r, c in detail:
        dm = [round((a - b) / b, 3) for a, b in zip(c["mom"], r["mom"])] \
            if len(c["mom"]) == len(r["mom"]) == 3 else None
        lines.append("detail  ref %-24s %9.0f  cand %-24s %9.0f  dvol %+6.1f%%  dmom %s" % (
            r["label"], r["vol"], c["label"], c["vol"],
            100 * (c["vol"] - r["vol"]) / r["vol"], dm))
    for r in gone:
        lines.append("missing ref %-24s %9.0f" % (r["label"], r["vol"]))
    for c in surplus:
        lines.append("extra   cand %-23s %9.0f" % (c["label"], c["vol"]))
    return lines


def pair_leftovers(missing, extra):
    """Pair unmatched reference solids with unmatched candidate solids that
    are plausibly the same part with different detailing (volume within
    DETAIL_VOL_TOL). Globally closest pairs first, so a big missing part
    never swallows a small one."""
    cands = []
    for i, rs in enumerate(missing):
        for k, cs in enumerate(extra):
            dv = abs(cs["vol"] - rs["vol"]) / max(rs["vol"], 1e-9)
            if dv <= DETAIL_VOL_TOL:
                cands.append((dv, i, k))
    cands.sort()
    used_r, used_c, pairs = set(), set(), []
    for dv, i, k in cands:
        if i in used_r or k in used_c:
            continue
        used_r.add(i)
        used_c.add(k)
        pairs.append((missing[i], extra[k]))
    gone = [rs for i, rs in enumerate(missing) if i not in used_r]
    surplus = [cs for k, cs in enumerate(extra) if k not in used_c]
    return pairs, gone, surplus


def check_geometry(m, ref):
    cand_s, ref_s = solids_of(m), solids_of(ref)
    if not ref_s:
        return 0.0, "reference has no solids to compare with"
    exact, missing, extra = match_solids(cand_s, ref_s)
    detail, gone, surplus = pair_leftovers(missing, extra)
    ref_vol = sum(s["vol"] for s in ref_s)

    # Three readings, the worst one governs:
    #  - missing parts always cost (a drawing part that is simply not there);
    #  - parts detailed differently from the reference are tolerated while
    #    they are few -- the reference is one rendering of the drawing, and
    #    the corpus itself shows it disagreeing with the drawing's BOM on a
    #    tube bore -- and cost once they show a pattern of unfinished parts;
    #  - material that is not where the reference has it, by volume.
    n_ref = float(len(ref_s))
    missing_score = lin(len(gone) / n_ref, 0.0, MISSING_ZERO)
    detail_score = lin(len(detail) / n_ref, DETAIL_FREE, DETAIL_ZERO)
    vol_err = (sum(abs(c["vol"] - r["vol"]) for r, c in detail)
               + sum(s["vol"] for s in gone) + sum(s["vol"] for s in surplus)) / ref_vol
    vol_score = lin(vol_err, VOL_FREE, VOL_ZERO)
    parts = min(missing_score, detail_score, vol_score)

    # arrangement: handedness of the assembled lift. The pivot pitch is not
    # re-read here from the saved pose -- a broken pose is a joint fault and
    # is graded there; the 120 mm hole spacing itself is already in the part
    # fingerprints above.
    mech = m.get("mechanism") or {}
    notes = []
    if mech.get("error") or not mech.get("side_lengths"):
        arr = 0.0
        notes.append("pantograph not found (%s)" % (mech.get("error") or "no pivots"))
    else:
        h, rh = handedness(m), handedness(ref)
        if h is None or rh is None:
            arr = 1.0
            notes.append("handedness not determinable")
        elif h == rh:
            arr = 1.0
            notes.append("same hand as the drawing")
        else:
            arr = 0.0 if MIRROR_IS_DEFECT else 1.0
            notes.append("MIRRORED relative to the drawing (clutch A/B and arm 1/2 swap sides)")
    score = 0.5 * parts + 0.5 * arr

    desc = ("parts %.2f: of %d reference solids %d match exactly, %d are present with "
            "different detailing, %d are missing; %d extra solids; %.1f%% of the "
            "reference volume differs (missing %.2f, detailing %.2f, volume %.2f). "
            "Arrangement %.2f: %s" % (
                parts, len(ref_s), len(exact), len(detail), len(gone), len(surplus),
                100 * vol_err, missing_score, detail_score, vol_score,
                arr, "; ".join(notes)))
    if detail:
        desc += ". Detailing differs: " + ", ".join(
            "%s (%+.1f%% vol)" % (r["label"], 100 * (c["vol"] - r["vol"]) / r["vol"])
            for r, c in detail[:6])
    if gone:
        desc += ". Missing: " + ", ".join(sorted({s["label"] for s in gone})[:8])
    if surplus:
        desc += ". Extra: " + ", ".join(sorted({s["label"] for s in surplus})[:6])
    return score, desc


# ---------------------------------------------------------------- joints
def joint_problems(j, uf):
    probs = []
    refs = j.get("refs") or [{}, {}]
    a, b = (x.get("component") for x in refs)
    if not a or not b:
        return ["a reference does not resolve to a part"]
    if uf.find(a) == uf.find(b):
        probs.append("joins parts that are already rigidly fixed together")
    if j.get("type") in ("Revolute", "Cylindrical"):
        e1, e2 = (x.get("elem") or {} for x in refs)
        if "axis_deg" not in j:
            probs.append("no circular feature to pivot on")
        else:
            if j["axis_deg"] > AXIS_TOL_DEG:
                probs.append("axes %.1f deg apart" % j["axis_deg"])
            if j["radial_off"] > RADIAL_TOL:
                probs.append("axes %.2f mm apart" % j["radial_off"])
            if j["axial_gap"] > AXIAL_TOL:
                probs.append("stretched %.2f mm along the axis" % j["axial_gap"])
        r1, r2 = e1.get("radius"), e2.get("radius")
        if r1 is not None and r2 is not None and abs(r1 - r2) > RADIUS_MISMATCH:
            probs.append("radius %.2f on radius %.2f" % (r1, r2))
        if e1.get("kind") == "shaft" and e2.get("kind") == "shaft":
            probs.append("shaft on shaft")
    return probs


def check_mated(m):
    uf = rigid_clusters(m)
    joints = [j for j in m.get("joints", [])
              if j.get("kind") == "joint" and j.get("type") != "Fixed"
              and not j.get("Suppressed") and j.get("Activated", True) is not False]
    if not joints:
        return 0.0, "no moving joints at all"
    bad = []
    for j in joints:
        p = joint_problems(j, uf)
        if p:
            bad.append("%s %s: %s" % (j.get("type"), j.get("label"), ", ".join(p)))
    ok = len(joints) - len(bad)
    mech = m.get("mechanism") or {}
    piv = mech.get("pivots") or []
    pf = min(1.0, len(piv) / 4.0)
    score = ok / float(len(joints)) * pf
    desc = "%d of %d moving joints genuinely mated; %d of 4 pantograph pivots found" % (
        ok, len(joints), len(piv))
    if bad:
        desc += ". Not mated: " + "; ".join(bad[:6])
        if len(bad) > 6:
            desc += "; ... %d more" % (len(bad) - 6)
    return score, desc


# ------------------------------------------------------------ articulation
def pose_ok(p, base_sides):
    if p.get("rc") != 0 or not p.get("path_ok", True):
        return False
    if p.get("opening_d") is None or abs(p["opening_d"] - p["target_d"]) > POSE_D_TOL:
        return False
    if p.get("upright") is not True:
        return False
    if (p.get("worst_axis_deg") or 0) > AXIS_TOL_DEG or (p.get("worst_off_axis") or 0) > RADIAL_TOL:
        return False
    sides = p.get("side_lengths") or []
    if base_sides and (len(sides) != len(base_sides) or
                       max(abs(x - y) for x, y in zip(sides, base_sides)) > POSE_SIDE_TOL):
        return False
    return True


def check_articulates(m):
    mech = m.get("mechanism") or {}
    drv = m.get("drive") or {}
    if mech.get("error"):
        return 0.0, "no pantograph identified, nothing to articulate (%s)" % mech["error"]
    if not drv.get("driver"):
        return 0.5, ("pantograph found but there is no slider to drive it with, so "
                     "articulation could not be exercised; half credit")
    if drv.get("error"):
        return 0.0, "drive experiment failed: %s" % str(drv["error"])[:200]
    st = drv.get("static") or {}
    adm = float(st.get("admissible_fraction", 0.0))
    poses = drv.get("poses") or []
    base_sides = mech.get("side_lengths")
    good = [p for p in poses if pose_ok(p, base_sides)]
    reach = len(good) / float(len(poses)) if poses else 0.0
    spans = [p["arm_span"] for p in good if p.get("arm_span") is not None]
    travel = (max(spans) - min(spans)) if len(spans) >= 2 else 0.0
    moves = 1.0 if travel >= MIN_SPAN_TRAVEL else (0.5 if good else 0.0)
    score = adm * reach * moves
    lim = st.get("own_limits") or {}
    if lim.get("EnableLengthMin") or lim.get("EnableLengthMax"):
        ltxt = "its limits [%s, %s] admit %.0f%% of the working band %s" % (
            lim.get("LengthMin") if lim.get("EnableLengthMin") else "-inf",
            lim.get("LengthMax") if lim.get("EnableLengthMax") else "inf",
            100 * adm, st.get("working_s_range"))
    else:
        ltxt = "no limits set, the whole working band is admitted"
    desc = ("saved %s; %s; driven to %s mm opening: %d/%d poses reached mated and upright; "
            "arms travel %.0f mm" % (
                "the right way out" if st.get("upright0") else "folded the wrong way",
                ltxt, "/".join("%g" % p["target_d"] for p in poses),
                len(good), len(poses), travel))
    return score, desc


# ------------------------------------------------------------- parametric
def used_sketches(m):
    return [s for s in m.get("sketches", []) if s.get("body")]


def check_constraints(m):
    sk = used_sketches(m)
    if not sk:
        return 0.0, "no sketches feed any body -- dead geometry"
    with_dims = [s for s in sk if any(d.get("driving") for d in s.get("dims", []))]
    frac = len(with_dims) / float(len(sk))
    fc = [s["fully_constrained"] for s in sk if s.get("fully_constrained") is not None]
    if fc:
        fcf = sum(1 for x in fc if x) / float(len(fc))
        score = frac * (0.8 + 0.2 * fcf)
        tail = "; %d of %d fully constrained" % (sum(1 for x in fc if x), len(fc))
    else:
        score, tail = frac, ""
    n_dims = sum(1 for s in sk for d in s.get("dims", []) if d.get("driving"))
    return score, ("%d of %d sketches carry driving dimensions (%d in all)%s" % (
        len(with_dims), len(sk), n_dims, tail))


def check_drives(m):
    probes = [p for p in m.get("param", []) if "sketch" in p]
    errs = [p for p in m.get("param", []) if "sketch" not in p]
    if not probes:
        if errs:
            return 0.0, "parametric probe failed: %s" % str(errs[0].get("error"))[-200:]
        return 0.0, "no sketch dimension to probe -- nothing drives the geometry"
    good, notes = 0, []
    for p in probes:
        restored = p.get("body_volume_restored") == p.get("body_volume_before")
        changed = (p.get("components_changed") or 0) > 0
        if changed and restored and not p.get("error") and not p.get("recompute_errors"):
            good += 1
        follows = any(abs(x - p["to"]) < PITCH_TOL for x in (p.get("side_lengths_after") or []))
        notes.append("%s %g->%g: %d assembly parts changed%s" % (
            p.get("sketch"), p["from"], p["to"], p.get("components_changed") or 0,
            ", pivots followed" if follows else ""))
    score = good / float(len(probes))
    return score, "%d of %d probed dimensions reshape the assembly (%s)" % (
        good, len(probes), "; ".join(notes))


def check_signatures(m):
    vals = []
    for s in used_sketches(m):
        for d in s.get("dims", []):
            if d.get("driving"):
                vals.append((d.get("type"), float(d.get("value", 0.0))))
    got, missing, total = [], [], 0.0
    for name, dia, rad in SIGNATURES:
        hit = any(abs(v - dia) < 1e-3 for t, v in vals if t != "Radius") or \
            (rad is not None and any(abs(v - rad) < 1e-3 for t, v in vals if t == "Radius"))
        part = 1.0 if hit else 0.0
        total += part
        (got if part == 1.0 else missing).append(name)
    score = total / len(SIGNATURES)
    desc = "%d of %d drawing dimensions carried as driving sketch dimensions" % (
        len(got), len(SIGNATURES))
    if missing:
        desc += "; missing: " + ", ".join(missing)
    return score, desc


# ----------------------------------------------------------------- driver
GATE = "executes and builds geometry"
CHECKS = (
    ("geometry matches ground truth", lambda m, ref: check_geometry(m, ref)),
    ("pantograph joints mated", lambda m, ref: check_mated(m)),
    ("mechanism articulates", lambda m, ref: check_articulates(m)),
    ("carries dimensional constraints", lambda m, ref: check_constraints(m)),
    ("dimensions drive the geometry", lambda m, ref: check_drives(m)),
    ("signature dimensions present", lambda m, ref: check_signatures(m)),
)


def score_all(meas, ref, err=None):
    """{name: (score, description)} for the gate and every component."""
    m, ref = unwrap(meas), unwrap(ref)
    out = {}
    g, gdesc = check_builds(m, err)
    out[GATE] = (g, gdesc)
    for name, fn in CHECKS:
        if g == 0.0:
            out[name] = (0.0, "not scored: the gate failed")
            continue
        try:
            s, d = fn(m, ref)
            out[name] = (round(clamp01(s), 4), d)
        except Exception as exc:                          # noqa: BLE001
            out[name] = (0.0, "scoring error: %s: %s" % (type(exc).__name__, exc))
    return out