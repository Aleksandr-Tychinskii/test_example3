"""Measurement stage for 4_sling_lift (v4).

Runs inside `freecadcmd`. Launched by `Harness.run_freecad_stage` with:

    FC_STAGE_INPUT    the document to measure
    FC_STAGE_OUT      a directory to write measure.json into
    FC_STAGE_COMMON   the directory holding common/

Stdlib and FreeCAD only. Everything here is *measurement*; no scoring.

What is measured, and why
-------------------------
Nothing here depends on object names, labels or the shape of the tree.

* Top-level assembly only (the one no AssemblyLink points at), so that
  sub-assembly originals are not counted twice.
* Components in assembly coordinates; per solid: volume, area, centre of
  mass, principal moments. Volume and moments do not change with pose,
  rigid motion or reflection, so parts can be compared with a reference
  without caring how the lift was placed, folded or mirrored.
* Joints, deduplicated. For each side: the component it lands on and the
  referenced element as geometry (centre, axis, radius, shaft/hole), plus
  the axis angle, radial offset and axial gap between the two sides. A
  joint is mated when its two sides really coincide; one that is
  "stretched" across a missing part shows an axial gap or a joint offset.
* Mechanism, from geometry alone: rigid clusters (by Fixed joints), pivot
  axes (by revolute locations), the top pivot T and centre pivot C of the
  rhombus, opening d = |T-C|, whether the lift is the right way out, the
  span between the arms, and a handedness sign that flips under reflection.
* Drive experiment: whether the candidate's own limits admit the working
  band (arithmetic, no solver), and whether the solver can carry the lift
  through it while staying mated (slider clamped, stepped 2 mm at a time).
* Parametric experiment: each 120 mm sketch dimension (fallback: the
  largest dimension of a few sketches) is nudged by +5 mm; recorded is
  whether any *assembly component* changed and whether the mechanism's
  pivot geometry followed. Then everything is restored.
* Sketch dimensional constraints by value, and whether each sketch is
  fully constrained.
"""
import math
import os
import sys
import traceback

sys.path.insert(0, os.environ["FC_STAGE_COMMON"])

from common.freecad_stage import Stage                    # noqa: E402

import FreeCAD as App                                     # noqa: E402
import Part                                               # noqa: E402

DIM_TYPES = ("Distance", "DistanceX", "DistanceY", "Radius", "Diameter", "Angle")
SKIP_TYPES = ("App::Origin", "App::Line", "App::Plane", "App::Point")
LIMIT_KEYS = ("EnableLengthMin", "LengthMin", "EnableLengthMax", "LengthMax",
              "EnableAngleMin", "AngleMin", "EnableAngleMax", "AngleMax")
MOVING = ("Revolute", "Slider", "Cylindrical", "Ball")
PARAM_PROBE = (120.0,)
#: Openings (top pivot to centre pivot, mm) the lift is driven through: a
#: band around the reference pose (158.7) that any working lift must reach.
DRIVE_OPENINGS = (145.0, 158.7, 172.0)
STEP_MM = 2.0
MAX_FALLBACK_PROBES = 4


# ---------------------------------------------------------------- helpers
def r(x, n=4):
    return round(float(x), n)


def vec(v):
    return [r(v.x), r(v.y), r(v.z)]


def plc(p):
    return {"base": vec(p.Base), "rot_q": [r(q, 6) for q in p.Rotation.Q]}


def mat3(m):
    return [[r(m.A11, 2), r(m.A12, 2), r(m.A13, 2)],
            [r(m.A21, 2), r(m.A22, 2), r(m.A23, 2)],
            [r(m.A31, 2), r(m.A32, 2), r(m.A33, 2)]]


def safe(fn, default=None):
    try:
        return fn()
    except Exception:                                     # noqa: BLE001
        return default


def is_joint(o):
    return hasattr(o, "JointType") and hasattr(o, "Reference1")


def is_grounded(o):
    return hasattr(o, "ObjectToGround")


def val(v):
    return v.Value if hasattr(v, "Value") else v


# list-vector maths (positions come back as lists)
def vsub(a, b):
    return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]


def vadd(a, b):
    return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]


def vmul(a, k):
    return [a[0] * k, a[1] * k, a[2] * k]


def vdot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def vnorm(a):
    return math.sqrt(vdot(a, a))


def vunit(a):
    n = vnorm(a)
    return vmul(a, 1.0 / n) if n > 1e-12 else a


def vcross(a, b):
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0]]


def qrot(q, v):
    x, y, z, w = q
    u = [x, y, z]

    def cr(a, b):
        return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
                a[0] * b[1] - a[1] * b[0]]
    t = vmul(cr(u, v), 2)
    return vadd(vadd(v, vmul(t, w)), cr(u, t))


def axis_angle(a, b):
    c = abs(vdot(vunit(a), vunit(b)))
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


def off_axis(p, origin, axis):
    d = vsub(p, origin)
    a = vunit(axis)
    return vnorm(vsub(d, vmul(a, vdot(d, a))))


# ------------------------------------------------------- assembly discovery
def find_assemblies(doc):
    asms = [o for o in doc.Objects if o.TypeId == "Assembly::AssemblyObject"]
    linked = set()
    for o in doc.Objects:
        if o.TypeId == "Assembly::AssemblyLink":
            t = getattr(o, "LinkedObject", None)
            if t is not None:
                linked.add(t.Name)
    tops = [a for a in asms if a.Name not in linked] or asms
    tops.sort(key=lambda a: safe(lambda: len(Part.getShape(a).Solids), 0),
              reverse=True)
    return asms, tops, linked


def children(container, doc):
    grp = getattr(container, "Group", None)
    if grp is not None:
        return list(grp)
    out = []
    for sub in safe(container.getSubObjects, []) or []:
        o = doc.getObject(sub.split(".")[0])
        if o is not None:
            out.append(o)
    return out


def walk(asm, doc):
    comps, joints, paths, skipped, seen = [], [], {asm.Name: ""}, {}, set()

    def add_joint(prefix, j):
        if j.Name not in seen:
            seen.add(j.Name)
            joints.append((prefix, j))

    def rec(container, prefix, depth):
        if depth > 6:
            return
        for ch in children(container, doc):
            path = prefix + ch.Name + "."
            paths.setdefault(ch.Name, path)
            t = ch.TypeId
            if t == "Assembly::JointGroup":
                for j in getattr(ch, "Group", []):
                    if is_joint(j) or is_grounded(j):
                        add_joint(prefix, j)
                continue
            if is_joint(ch) or is_grounded(ch):
                add_joint(prefix, ch)
                continue
            if t in ("Assembly::AssemblyLink", "App::DocumentObjectGroup"):
                rec(ch, path, depth + 1)
                continue
            if t in SKIP_TYPES:
                continue
            shape = safe(lambda: Part.getShape(asm, path, transform=True))
            if shape is not None and not shape.isNull() and shape.Solids:
                comps.append((path, ch))
                for sub in safe(ch.getSubObjects, []) or []:
                    paths.setdefault(sub.split(".")[0], path)
            else:
                skipped[t] = skipped.get(t, 0) + 1

    rec(asm, "", 0)
    return comps, joints, paths, skipped


# ---------------------------------------------------------- measurements
def solid_stats(s):
    d = {"volume": r(abs(s.Volume), 3), "area": r(s.Area, 3),
         "com": vec(s.CenterOfMass)}
    d["inertia"] = safe(lambda: mat3(s.MatrixOfInertia))
    pp = safe(lambda: s.PrincipalProperties)
    if pp:
        d["moments"] = [r(m, 1) for m in pp["Moments"]]
        d["axes"] = [vec(pp["FirstAxisOfInertia"]),
                     vec(pp["SecondAxisOfInertia"]),
                     vec(pp["ThirdAxisOfInertia"])]
    return d


def comp_shape(asm, path):
    return Part.getShape(asm, path, transform=True)


def comp_mass(asm, path):
    """(volume, centre of mass) of one component, in assembly coordinates."""
    sh = comp_shape(asm, path)
    vol, acc = 0.0, [0.0, 0.0, 0.0]
    for s in sh.Solids:
        v = abs(s.Volume)
        vol += v
        acc = vadd(acc, vmul(vec(s.CenterOfMass), v))
    return vol, (vmul(acc, 1.0 / vol) if vol > 0 else vec(sh.BoundBox.Center))


def split_element(full):
    if full.endswith("."):
        return full, None
    i = full.rfind(".")
    return full[:i + 1], full[i + 1:]


def cyl_kind(asm, full, radius):
    """'shaft' if the cylinder at this element is convex, 'hole' if concave."""
    objpath, el = split_element(full)
    if not el:
        return None
    sh = Part.getShape(asm, objpath, transform=True)
    e = sh.getElement(el)
    faces = [e] if e.ShapeType == "Face" else sh.ancestorsOfType(e, Part.Face)
    for f in faces:
        s = f.Surface
        if type(s).__name__ != "Cylinder":
            continue
        if radius is not None and abs(s.Radius - radius) > 0.05:
            continue
        u0, u1, v0, v1 = f.ParameterRange
        u, v = (u0 + u1) / 2, (v0 + v1) / 2
        p, n = vec(f.valueAt(u, v)), vec(f.normalAt(u, v))
        a, c = vec(s.Axis), vec(s.Center)
        d = vsub(p, c)
        radial = vsub(d, vmul(vunit(a), vdot(d, vunit(a))))
        return "shaft" if vdot(n, radial) > 0 else "hole"
    return None


def element_info(asm, full, with_kind=True):
    try:
        sh = Part.getShape(asm, full, needSubElement=True, transform=True)
    except Exception as exc:                              # noqa: BLE001
        return {"error": str(exc)[:200]}
    if sh is None or sh.isNull():
        m = safe(lambda: asm.getSubObject(full, retType=2))
        if m is not None:
            p = App.Placement(m)
            return {"stype": "placement", "point": vec(p.Base),
                    "axis": vec(p.Rotation.multVec(App.Vector(0, 0, 1)))}
        return {"null": True}
    info = {"stype": sh.ShapeType}
    try:
        if sh.ShapeType == "Edge":
            c = sh.Curve
            info["geom"] = type(c).__name__
            if hasattr(c, "Radius") and hasattr(c, "Axis"):
                info.update(center=vec(c.Center), axis=vec(c.Axis),
                            radius=r(c.Radius))
            elif hasattr(c, "Direction"):
                info.update(point=vec(sh.Vertexes[0].Point),
                            axis=vec(c.Direction))
        elif sh.ShapeType == "Face":
            s = sh.Surface
            info["geom"] = type(s).__name__
            if hasattr(s, "Radius") and hasattr(s, "Axis"):
                info.update(center=vec(s.Center), axis=vec(s.Axis),
                            radius=r(s.Radius))
            elif hasattr(s, "Axis"):
                info.update(point=vec(s.Position), axis=vec(s.Axis))
        elif sh.ShapeType == "Vertex":
            info["point"] = vec(sh.Point)
        else:
            if sh.Edges and hasattr(sh.Edges[0].Curve, "Direction"):
                e = sh.Edges[0]
                info.update(geom="Line", point=vec(e.Vertexes[0].Point),
                            axis=vec(e.Curve.Direction))
            elif sh.Faces and hasattr(sh.Faces[0].Surface, "Axis"):
                f = sh.Faces[0]
                info.update(geom="Plane", point=vec(f.Surface.Position),
                            axis=vec(f.Surface.Axis))
        if with_kind and "radius" in info:
            info["kind"] = safe(lambda: cyl_kind(asm, full, info["radius"]))
    except Exception as exc:                              # noqa: BLE001
        info["error"] = str(exc)[:200]
    return info


def resolve_ref(asm, ref, paths, comp_paths, with_kind=True):
    if not ref:
        return {"empty": True}
    obj, subs = ref[0], ref[1] if len(ref) > 1 else []
    sub = subs[0] if subs else ""
    prefix = paths.get(obj.Name)
    out = {"obj_label": obj.Label, "sub": sub}
    if prefix is None:
        out["unresolved"] = True
        return out
    full = prefix + sub
    owner = None
    for cp in comp_paths:
        if full.startswith(cp) and (owner is None or len(cp) > len(owner)):
            owner = cp
    out["component"] = owner
    out["elem"] = element_info(asm, full, with_kind)
    return out


def jcs_global(j, which):
    try:
        import UtilsAssembly as UA                        # noqa: N813
        ref = getattr(j, "Reference%d" % which)
        p = getattr(j, "Placement%d" % which)
        return plc(UA.getJcsGlobalPlc(p, ref))
    except Exception as exc:                              # noqa: BLE001
        return {"error": str(exc)[:200]}


def slider_coord(rec):
    j1, j2 = rec.get("jcs", [{}, {}])
    if "base" not in j1 or "base" not in j2:
        return None
    return vdot(vsub(j2["base"], j1["base"]), qrot(j1["rot_q"], [0, 0, 1]))


def joint_record(asm, j, paths, comp_paths, with_kind=True):
    rec = {"name": j.Name, "label": j.Label}
    if is_grounded(j) and not is_joint(j):
        g = j.ObjectToGround
        rec["kind"] = "grounded"
        rec["component"] = paths.get(g.Name) if g is not None else None
        return rec
    rec["kind"] = "joint"
    rec["type"] = str(j.JointType)
    for k in ("Activated", "Suppressed", "Distance", "Distance2") + LIMIT_KEYS:
        if hasattr(j, k):
            rec[k] = val(getattr(j, k))
    rec["refs"] = [resolve_ref(asm, j.Reference1, paths, comp_paths, with_kind),
                   resolve_ref(asm, j.Reference2, paths, comp_paths, with_kind)]
    rec["jcs"] = [jcs_global(j, 1), jcs_global(j, 2)]
    for k in ("Offset1", "Offset2"):
        if hasattr(j, k):
            rec[k] = plc(getattr(j, k))
    e1, e2 = (x.get("elem", {}) for x in rec["refs"])
    if all(k in e for e in (e1, e2) for k in ("center", "axis")):
        a = vunit(e1["axis"])
        d = vsub(e2["center"], e1["center"])
        rec["axial_gap"] = r(abs(vdot(d, a)), 4)
        rec["radial_off"] = r(off_axis(e2["center"], e1["center"], a), 4)
        rec["axis_deg"] = r(axis_angle(e1["axis"], e2["axis"]), 3)
    if rec["type"] == "Slider":
        s = slider_coord(rec)
        rec["coord"] = r(s) if s is not None else None
    return rec


# ------------------------------------------------------------- mechanism
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


def clusters_of(comp_paths, jrecs, ignore=()):
    uf = UF()
    for p in comp_paths:
        uf.find(p)
    for j in jrecs:
        if j.get("kind") != "joint" or j.get("type") != "Fixed" or j.get("Suppressed"):
            continue
        if j.get("name") in ignore:
            continue
        a, b = (x.get("component") for x in j["refs"])
        if a and b:
            uf.union(a, b)
    return uf


def analyse_mechanism(asm, comp_paths, jrecs, ignore=()):
    """Pivots, T/C, opening, orientation, arm span -- from geometry alone.

    `ignore` names joints to leave out of the rigid clustering -- the drive
    experiment temporarily turns the slider into a Fixed joint, which must
    not weld the top and centre pivots into one body.
    """
    uf = clusters_of(comp_paths, jrecs, ignore)
    mass = {}
    for p in comp_paths:
        v, c = safe(lambda p=p: comp_mass(asm, p), (0.0, [0, 0, 0]))
        k = uf.find(p)
        mv, mc = mass.get(k, (0.0, [0.0, 0.0, 0.0]))
        mass[k] = (mv + v, vadd(mc, vmul(c, v)))
    cl = {k: (v, vmul(c, 1.0 / v) if v > 0 else c) for k, (v, c) in mass.items()}

    revs = []
    for j in jrecs:
        if j.get("kind") != "joint" or j.get("type") != "Revolute" or j.get("Suppressed"):
            continue
        a, b = (x.get("component") for x in j["refs"])
        e = j["refs"][0].get("elem", {})
        if not (a and b) or uf.find(a) == uf.find(b):
            continue
        if "center" not in e or "axis" not in e:
            continue
        revs.append((uf.find(a), uf.find(b), e["center"], e["axis"]))
    out = {"n_clusters": len(cl), "n_moving_revolutes": len(revs)}
    if len(revs) < 3:
        out["error"] = "fewer than three moving revolutes"
        return out

    n = vunit(revs[0][3])
    pivots = []
    for ca, cb, c, _ in revs:
        flat = vsub(c, vmul(n, vdot(c, n)))
        for pv in pivots:
            if vnorm(vsub(pv["flat"], flat)) < 2.0:
                pv["clusters"].update((ca, cb))
                break
        else:
            pivots.append({"flat": flat, "point": c, "clusters": {ca, cb}})
    out["pivots"] = [{"point": [r(x, 2) for x in pv["point"]],
                      "n_bodies": len(pv["clusters"])} for pv in pivots]
    out["axis_spread_deg"] = r(max(axis_angle(n, x[3]) for x in revs), 2)
    if len(pivots) < 4:
        out["error"] = "found %d pivots, expected 4" % len(pivots)
        return out

    count = {}
    for pv in pivots:
        for k in pv["clusters"]:
            count[k] = count.get(k, 0) + 1
    links = {k for k, c in count.items() if c >= 2}

    best = None
    for i in range(len(pivots)):
        for k in range(i + 1, len(pivots)):
            rest = [pv for m, pv in enumerate(pivots) if m not in (i, k)]
            if len(rest) < 2:
                continue
            P, Q = pivots[i]["flat"], pivots[k]["flat"]
            e0, e1 = rest[0]["flat"], rest[1]["flat"]
            asym = abs(vnorm(vsub(e0, P)) - vnorm(vsub(e1, P))) + \
                abs(vnorm(vsub(e0, Q)) - vnorm(vsub(e1, Q)))
            if best is None or asym < best[0]:
                best = (asym, i, k, rest)
    asym, i, k, rest = best

    def beyond(Tp, Cp):
        T, C = Tp["flat"], Cp["flat"]
        tc = vsub(C, T)
        return sum(1 for b in Cp["clusters"] if b in links and
                   vdot(vsub(cl[b][1], C), tc) > 0)

    Tp, Cp = pivots[i], pivots[k]
    if beyond(Cp, Tp) > beyond(Tp, Cp):
        Tp, Cp = Cp, Tp
    T, C = Tp["flat"], Cp["flat"]
    tc = vsub(C, T)

    ring = [b for b in Tp["clusters"] if b not in links]
    ring.sort(key=lambda b: cl[b][0], reverse=True)
    upright = None
    if ring:
        upright = vdot(vsub(cl[ring[0]][1], T), tc) < 0
    arms = [b for b in Cp["clusters"] if b in links]
    span = vnorm(vsub(cl[arms[0]][1], cl[arms[1]][1])) if len(arms) >= 2 else None
    elbows = [pv["flat"] for pv in rest]

    # Handedness of the arrangement: T->C, the in-plane side the heavier arm
    # cluster sits on, and which arm is in front along the pivot axis. Pose-
    # and rotation-invariant; flips sign under a reflection.
    chir = {}
    if len(arms) >= 2:
        a_hi, a_lo = sorted(arms, key=lambda b: cl[b][0], reverse=True)[:2]
        c_hi, c_lo = cl[a_hi][1], cl[a_lo][1]
        u = vunit(tc)
        ax_sep = vdot(vsub(c_lo, c_hi), n)
        flat_hi = vsub(c_hi, vmul(n, vdot(c_hi, n)))
        w = vsub(flat_hi, T)
        w = vsub(w, vmul(u, vdot(w, u)))
        if abs(ax_sep) > 0.5 and vnorm(w) > 0.5:
            no = vmul(n, 1.0 if ax_sep > 0 else -1.0)
            chir["sign"] = 1 if vdot(vcross(u, vunit(w)), no) > 0 else -1
        if "sign" in chir and upright is not None:
            # folding to the other branch flips this measure; normalise it to
            # the working (upright) branch so it reflects only handedness
            chir["sign_upright"] = chir["sign"] if upright else -chir["sign"]
        chir.update(arm_axial_sep=r(ax_sep, 3), arm_lateral=r(vnorm(w), 3),
                    heavier_arm_volume=r(cl[a_hi][0], 0),
                    lighter_arm_volume=r(cl[a_lo][0], 0))

    out.update({
        "rhombus_asymmetry": r(asym, 3),
        "opening_d": r(vnorm(tc), 3),
        "upright": upright,
        "ring_volume": r(cl[ring[0]][0], 0) if ring else None,
        "arm_span": r(span, 3) if span is not None else None,
        "arm_volumes": [r(cl[a][0], 0) for a in arms],
        "side_lengths": sorted(r(vnorm(vsub(e, x)), 3)
                               for e in elbows for x in (T, C)),
        "n_links": len(links),
        "chirality": chir,
    })
    return out


def mated_summary(jrecs, uf):
    """Worst misalignment over moving revolutes between different clusters."""
    worst_ang, worst_off = 0.0, 0.0
    for j in jrecs:
        if j.get("kind") != "joint" or j.get("type") != "Revolute" or j.get("Suppressed"):
            continue
        a, b = (x.get("component") for x in j["refs"])
        if not (a and b) or uf.find(a) == uf.find(b):
            continue
        e1, e2 = (x.get("elem", {}) for x in j["refs"])
        if "axis" in e1 and "axis" in e2 and "center" in e1 and "center" in e2:
            worst_ang = max(worst_ang, axis_angle(e1["axis"], e2["axis"]))
            worst_off = max(worst_off, off_axis(e2["center"], e1["center"], e1["axis"]))
    return r(worst_ang, 3), r(worst_off, 3)


# ------------------------------------------------------------ experiments
def snapshot_placements(comps):
    return [(o, App.Placement(o.Placement)) for _, o in comps
            if hasattr(o, "Placement")]


def restore_placements(snap):
    for o, p in snap:
        safe(lambda o=o, p=p: setattr(o, "Placement", p))


def comp_fingerprints(asm, comps):
    """Per component: total volume and sorted principal moments of its solids."""
    out = {}
    for p, _ in comps:
        sh = safe(lambda p=p: comp_shape(asm, p))
        if sh is None or sh.isNull():
            continue
        vol, mom = 0.0, []
        for s in sh.Solids:
            vol += abs(s.Volume)
            pp = safe(lambda s=s: s.PrincipalProperties)
            if pp:
                mom.extend(sorted(pp["Moments"]))
        out[p] = (vol, mom)
    return out


def fingerprints_differ(a, b, rel=1e-6):
    if abs(a[0] - b[0]) > rel * max(1.0, abs(a[0])):
        return True
    if len(a[1]) != len(b[1]):
        return True
    return any(abs(x - y) > rel * max(1.0, abs(x)) for x, y in zip(a[1], b[1]))


def find_driver(base_jrecs, comp_paths):
    uf = clusters_of(comp_paths, base_jrecs)
    for rec in base_jrecs:
        if rec.get("type") == "Slider" and not rec.get("Suppressed") \
                and rec.get("coord") is not None:
            a, b = (x.get("component") for x in rec["refs"])
            if a and b and uf.find(a) != uf.find(b):
                return rec
    return None


def drive_experiment(doc, asm, comps, joints, paths, comp_paths, base_mech, base_jrecs):
    """Drive the lift through DRIVE_OPENINGS and record each pose.

    Two separate questions:
      static  -- do the candidate's own limits admit the working band at all?
                 (pure arithmetic on the slider coordinate, no solver)
      poses   -- with limits lifted, can the solver actually carry the lift
                 through the band, staying mated and the right way out?
    The slider is clamped by temporarily turning it into a Fixed joint with
    an offset, stepping STEP_MM at a time from the saved pose so the solver
    never has to jump.
    """
    out = {}
    jobj = {j.Name: j for _, j in joints if is_joint(j)}
    driver = find_driver(base_jrecs, comp_paths)
    if driver is None:
        out["driver"] = None
        return out
    out["driver"] = driver["name"]
    if base_mech.get("opening_d") is None or base_mech.get("upright") is None:
        out["error"] = "mechanism not identified, cannot drive"
        return out
    j = jobj[driver["name"]]
    s0, d0, up0 = driver["coord"], base_mech["opening_d"], base_mech["upright"]
    sgn0 = 1.0 if s0 >= 0 else -1.0
    sign_up = sgn0 if up0 else -sgn0          # slider sign on the working branch
    c = abs(s0) - d0                          # slider datum offset vs pivot distance
    orig_limits = {name: {k: val(getattr(o, k)) for k in LIMIT_KEYS if hasattr(o, k)}
                   for name, o in jobj.items()}
    lim = orig_limits[driver["name"]]

    lo_d, hi_d = min(DRIVE_OPENINGS), max(DRIVE_OPENINGS)
    s_a, s_b = sorted((sign_up * (lo_d + c), sign_up * (hi_d + c)))
    lmin = lim["LengthMin"] if lim.get("EnableLengthMin") else -1e9
    lmax = lim["LengthMax"] if lim.get("EnableLengthMax") else 1e9
    overlap = max(0.0, min(s_b, lmax) - max(s_a, lmin))
    out["static"] = {"s0": r(s0, 3), "d0": r(d0, 3), "upright0": up0,
                     "sign_up": sign_up, "offset_c": r(c, 3),
                     "working_s_range": [r(s_a, 2), r(s_b, 2)],
                     "own_limits": lim,
                     "admissible_fraction": r(overlap / (s_b - s_a), 4)}

    jc1, jc2 = driver["jcs"]
    r1 = App.Rotation(*jc1["rot_q"])
    r2 = App.Rotation(*jc2["rot_q"])
    z1 = qrot(jc1["rot_q"], [0, 0, 1])
    rot_off = r2.inverted().multiply(r1)
    orig_type = str(j.JointType)
    orig_off = {k: App.Placement(getattr(j, k)) for k in ("Offset1", "Offset2")
                if hasattr(j, k)}
    ign = {driver["name"]}
    snap = snapshot_placements(comps)

    def clamp(d):
        j.JointType = "Fixed"
        t = r2.inverted().multVec(App.Vector(*vmul(z1, -sign_up * (d + c))))
        j.Offset2 = App.Placement(t, rot_off)
        return asm.solve()

    def pose(target, rc, how, reached_path):
        jr = [joint_record(asm, o, paths, comp_paths, with_kind=False)
              for _, o in joints]
        m = analyse_mechanism(asm, comp_paths, jr, ign)
        rec = {"target_d": r(target, 2), "rc": rc, "how": how,
               "path_ok": reached_path}
        rec.update({k: m.get(k) for k in ("opening_d", "upright", "arm_span",
                                          "side_lengths", "error")})
        rec["worst_axis_deg"], rec["worst_off_axis"] = mated_summary(
            jr, clusters_of(comp_paths, jr, ign))
        return rec

    poses = []
    try:
        for o in jobj.values():
            for k in ("EnableLengthMin", "EnableLengthMax",
                      "EnableAngleMin", "EnableAngleMax"):
                if hasattr(o, k):
                    setattr(o, k, False)
        targets = sorted(DRIVE_OPENINGS)
        if up0:
            down = sorted((t for t in targets if t <= d0), reverse=True)
            up = [t for t in targets if t > d0]
            for seq in (down, up):
                restore_placements(snap)
                cur, alive, rc = d0, True, 0
                for t in seq:
                    if alive:
                        n = max(1, int(math.ceil(abs(t - cur) / STEP_MM)))
                        for k in range(1, n + 1):
                            try:
                                rc = clamp(cur + (t - cur) * k / n)
                            except Exception as exc:      # noqa: BLE001
                                rc = "exception: %s" % str(exc)[:120]
                            if rc != 0:
                                alive = False
                                break
                        cur = t
                    poses.append(pose(t, rc, "stepped", alive))
        else:
            # saved folded the wrong way: no continuous path, try a direct jump
            for t in targets:
                restore_placements(snap)
                try:
                    rc = clamp(t)
                except Exception as exc:                  # noqa: BLE001
                    rc = "exception: %s" % str(exc)[:120]
                poses.append(pose(t, rc, "jump", rc == 0))
    finally:
        safe(lambda: setattr(j, "JointType", orig_type))
        for k, v in orig_off.items():
            safe(lambda k=k, v=v: setattr(j, k, v))
        for name, o in jobj.items():
            for k, v in orig_limits[name].items():
                safe(lambda o=o, k=k, v=v: setattr(o, k, v))
        restore_placements(snap)
        safe(asm.solve)
    out["poses"] = sorted(poses, key=lambda p: p["target_d"])
    return out


def param_targets(doc):
    """Sketch dimensions to nudge: every 120 mm one (the pivot pitch the
    drawing is built around); failing that, the largest driving distance
    in each of up to MAX_FALLBACK_PROBES sketches."""
    sk = [s for s in doc.Objects if s.TypeId == "Sketcher::SketchObject"
          and any(p.TypeId == "PartDesign::Body" for p in s.InList)]
    out = []
    for s in sk:
        for i, c in enumerate(s.Constraints):
            if c.Type in ("Distance", "DistanceX", "DistanceY") and \
                    getattr(c, "Driving", True) and \
                    any(abs(c.Value - p) < 1e-6 for p in PARAM_PROBE):
                out.append((s, i, c.Value, "signature"))
    if out:
        return out
    best = []
    for s in sk:
        cands = [(c.Value, i) for i, c in enumerate(s.Constraints)
                 if c.Type in ("Distance", "DistanceX", "DistanceY")
                 and getattr(c, "Driving", True) and c.Value > 1.0]
        if cands:
            v, i = max(cands)
            best.append((v, s, i))
    best.sort(key=lambda x: -x[0])
    return [(s, i, v, "fallback") for v, s, i in best[:MAX_FALLBACK_PROBES]]


def param_experiment(doc, asm, comps, joints, paths, comp_paths, base_mech):
    """Nudge each probe dimension by +5 mm and see what follows.

    Recorded per probe: whether the owning body changed, how many *assembly
    components* changed shape (a dimension that only moves an orphan body
    drives nothing the candidate actually built), and whether the pivot
    geometry of the mechanism followed after a re-solve.
    """
    out = []
    targets = param_targets(doc)
    snap = snapshot_placements(comps)
    base_fp = comp_fingerprints(asm, comps)

    def moments(b):
        pp = safe(lambda: b.Shape.PrincipalProperties)
        if pp is None:
            pp = safe(lambda: b.Shape.Solids[0].PrincipalProperties)
        return sorted(r(m, 0) for m in pp["Moments"]) if pp else None

    for s, i, v, why in targets:
        body = next((p for p in s.InList if p.TypeId == "PartDesign::Body"), None)
        rec = {"sketch": s.Name, "index": i, "from": v, "to": v + 5, "why": why}
        try:
            if body is not None:
                rec["body_volume_before"] = r(body.Shape.Volume, 1)
                rec["body_moments_before"] = moments(body)
            s.setDatum(i, App.Units.Quantity(v + 5, App.Units.Length))
            doc.recompute()
            rec["recompute_errors"] = [o.Name for o in doc.Objects
                                       if "Invalid" in o.State or "Error" in o.State][:10]
            if body is not None:
                rec["body_volume_after"] = r(body.Shape.Volume, 1)
                rec["body_moments_after"] = moments(body)
            after_fp = comp_fingerprints(asm, comps)
            rec["components_changed"] = sum(
                1 for p, fp in base_fp.items()
                if p in after_fp and fingerprints_differ(fp, after_fp[p]))
            rec["rc"] = safe(asm.solve)
            jr = [joint_record(asm, o, paths, comp_paths, with_kind=False)
                  for _, o in joints]
            m = analyse_mechanism(asm, comp_paths, jr)
            rec["side_lengths_after"] = m.get("side_lengths")
            rec["mech_error"] = m.get("error")
            rec["worst_axis_deg"], rec["worst_off_axis"] = mated_summary(
                jr, clusters_of(comp_paths, jr))
        except Exception:                                 # noqa: BLE001
            rec["error"] = traceback.format_exc()[-400:]
        finally:
            safe(lambda: s.setDatum(i, App.Units.Quantity(v, App.Units.Length)))
            doc.recompute()
            restore_placements(snap)
            safe(asm.solve)
            if body is not None:
                rec["body_volume_restored"] = safe(lambda: r(body.Shape.Volume, 1))
        out.append(rec)
    return out


def sketches(doc):
    out = []
    for s in doc.Objects:
        if s.TypeId != "Sketcher::SketchObject":
            continue
        bound = {p for p, _ in s.ExpressionEngine}
        dims = []
        for i, c in enumerate(s.Constraints):
            if c.Type not in DIM_TYPES:
                continue
            v = math.degrees(c.Value) if c.Type == "Angle" else c.Value
            keys = {".Constraints[%d]" % i}
            if c.Name:
                keys.add(".Constraints." + c.Name)
            dims.append({"i": i, "type": c.Type, "value": r(v, 4),
                         "driving": bool(getattr(c, "Driving", True)),
                         "expr": bool(keys & bound)})
        body = next((p.Name for p in s.InList
                     if p.TypeId == "PartDesign::Body"), None)
        fc = getattr(s, "FullyConstrained", None)
        out.append({"name": s.Name, "body": body,
                    "n_constraints": len(s.Constraints), "dims": dims,
                    "fully_constrained": bool(fc) if fc is not None else None})
    return out


# --------------------------------------------------------------- the stage
class SlingLiftStage(Stage):
    def measure(self, doc):
        out = {"version": 4}
        doc.recompute()
        out["recompute_errors"] = [o.Name for o in doc.Objects
                                   if "Invalid" in o.State or "Error" in o.State]
        out["sketches"] = sketches(doc)

        asms, tops, linked = find_assemblies(doc)
        out["assemblies"] = {"all": [a.Name for a in asms],
                             "linked_as_sub": sorted(linked),
                             "top_candidates": [a.Name for a in tops]}
        if not tops:
            out["error_no_assembly"] = True
            return out
        asm = tops[0]
        out["assemblies"]["top"] = asm.Name
        out["n_solids_total"] = safe(lambda: len(Part.getShape(asm).Solids), 0)

        comps, joints, paths, skipped = walk(asm, doc)
        comp_paths = [p for p, _ in comps]
        out["skipped_child_types"] = skipped

        try:
            out["solve_rc"] = asm.solve()
        except Exception as exc:                          # noqa: BLE001
            out["solve_error"] = str(exc)[:300]
        doc.recompute()

        comp_out = []
        for p, o in comps:
            rec = {"path": p, "label": o.Label, "type": o.TypeId}
            try:
                sh = comp_shape(asm, p)
                rec["bbox"] = [r(x, 3) for x in (
                    sh.BoundBox.XMin, sh.BoundBox.YMin, sh.BoundBox.ZMin,
                    sh.BoundBox.XMax, sh.BoundBox.YMax, sh.BoundBox.ZMax)]
                rec["solids"] = [solid_stats(s) for s in sh.Solids]
            except Exception as exc:                      # noqa: BLE001
                rec["error"] = str(exc)[:300]
            comp_out.append(rec)
        out["components"] = comp_out

        jrecs = []
        for _, j in joints:
            try:
                jrecs.append(joint_record(asm, j, paths, comp_paths))
            except Exception:                             # noqa: BLE001
                jrecs.append({"name": j.Name,
                              "error": traceback.format_exc()[-400:]})
        out["joints"] = jrecs

        try:
            out["mechanism"] = analyse_mechanism(asm, comp_paths, jrecs)
        except Exception:                                 # noqa: BLE001
            out["mechanism"] = {"error": traceback.format_exc()[-600:]}
        try:
            out["drive"] = drive_experiment(doc, asm, comps, joints, paths,
                                            comp_paths, out["mechanism"], jrecs)
        except Exception:                                 # noqa: BLE001
            out["drive"] = {"error": traceback.format_exc()[-600:]}
        try:
            out["param"] = param_experiment(doc, asm, comps, joints, paths,
                                            comp_paths, out["mechanism"])
        except Exception:                                 # noqa: BLE001
            out["param"] = [{"error": traceback.format_exc()[-600:]}]
        return out


SlingLiftStage.run()