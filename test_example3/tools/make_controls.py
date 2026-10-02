"""Build control variants of the reference to test the grader (run in freecadcmd).

Usage (Git Bash, from FreeCAD/4_sling_lift):
    FC_REF=tests/task/reference/solution.FCStd FC_OUT=controls \
        "$FREECAD_CMD" ../../tools/make_controls.py

Writes three documents into FC_OUT:
    renamed.FCStd  every object label replaced           -> expect 6/6
    moved.FCStd    whole lift rotated 37 deg and shifted  -> expect 6/6
    dead.FCStd     bare solids, no assembly, no sketches  -> expect ~0
"""
import os
import traceback

import FreeCAD as App
import Part

REF = os.path.abspath(os.environ["FC_REF"])
OUT = os.path.abspath(os.environ.get("FC_OUT", "controls"))
os.makedirs(OUT, exist_ok=True)


def top_assembly(doc):
    asms = [o for o in doc.Objects if o.TypeId == "Assembly::AssemblyObject"]
    linked = {getattr(o, "LinkedObject", None) and o.LinkedObject.Name
              for o in doc.Objects if o.TypeId == "Assembly::AssemblyLink"}
    tops = [a for a in asms if a.Name not in linked] or asms
    return tops[0]


def placed_children(asm):
    out = []
    for ch in asm.Group:
        if ch.TypeId == "Assembly::JointGroup" or hasattr(ch, "JointType"):
            continue
        if ch.TypeId in ("App::Origin",):
            continue
        if hasattr(ch, "Placement"):
            out.append(ch)
    return out


def renamed():
    doc = App.openDocument(REF)
    for i, o in enumerate(doc.Objects):
        try:
            o.Label = "Obj%04d" % (7919 * (i + 1) % 10007)
        except Exception:                                  # noqa: BLE001
            pass
    doc.recompute()
    doc.saveAs(os.path.join(OUT, "renamed.FCStd"))
    App.closeDocument(doc.Name)


def moved():
    doc = App.openDocument(REF)
    asm = top_assembly(doc)
    T = App.Placement(App.Vector(250, -80, 40),
                      App.Rotation(App.Vector(1, 2, 3), 37))
    for ch in placed_children(asm):
        ch.Placement = T.multiply(ch.Placement)
    doc.recompute()
    rc = asm.solve()
    doc.recompute()
    print("moved: solve rc =", rc)
    doc.saveAs(os.path.join(OUT, "moved.FCStd"))
    App.closeDocument(doc.Name)


def dead():
    src = App.openDocument(REF)
    asm = top_assembly(src)
    shapes = []
    for ch in placed_children(asm):
        try:
            sh = Part.getShape(asm, ch.Name + ".", transform=True)
            if not sh.isNull() and sh.Solids:
                shapes.append(sh.copy())
        except Exception:                                  # noqa: BLE001
            pass
    App.closeDocument(src.Name)
    doc = App.newDocument("dead")
    for i, sh in enumerate(shapes):
        f = doc.addObject("Part::Feature", "Solid%03d" % i)
        f.Shape = sh
    doc.recompute()
    doc.saveAs(os.path.join(OUT, "dead.FCStd"))
    App.closeDocument(doc.Name)
    print("dead: %d bare shapes" % len(shapes))


for fn in (renamed, moved, dead):
    try:
        fn()
        print("ok:", fn.__name__)
    except Exception:                                      # noqa: BLE001
        print("FAILED:", fn.__name__)
        traceback.print_exc()
os._exit(0)