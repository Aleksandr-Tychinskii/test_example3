# Sling lift grader: notes

## Results

| Model | geometry | joints | articulates | constraints | dims drive | signatures | total |
|---|---|---|---|---|---|---|---|
| solution | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 | **6.00** |
| half_finished | 0.56 | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 | **5.56** |
| mirrored | 0.50 | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 | **5.50** |
| one_link_missing | 0.91 | 0.58 | 1.00 | 1.00 | 1.00 | 1.00 | **5.50** |
| joint_limits_swapped | 1.00 | 1.00 | 0.00 | 1.00 | 1.00 | 1.00 | **5.00** |
| dummy_joints | 1.00 | 0.33 | 0.00 | 1.00 | 1.00 | 1.00 | **4.33** |

Extra checks on copies of the reference: all labels renamed gives 6.00; the whole lift rotated and moved gives 6.00; bare solids with no assembly give 0.

## What I measure

- **Geometry**: each part is compared with the reference by volume and moments of inertia, so pose, position and mirroring don't matter. Missing parts cost. A few small differences are allowed (see the tube below). Half of this score is whether the lift is mirrored.
- **Joints**: a joint counts only if it really connects two separate parts, the axes line up, and it isn't stretched with an offset. Extra joints can only lower the score.
- **Articulates**: I check that the joint limits allow the working range, then move the slider step by step and check that the lift stays assembled and the arms move.
- **Constraints / dims drive / signatures**: sketches have driving dimensions; changing 120 → 125 actually changes parts in the assembly; the drawing's 120, Ø16, Ø16.2, Ø20 and Ø20.2 are there.

## What I ignore

Names, the document tree, joint and object counts, and the pose and position of the lift. I also don't count the same mistake twice.

## Mirror

I count the mirror as a mistake, but only −0.5. Clutch A and B are different parts, and the drawing shows where each one goes, so a mirrored lift is a different product. Everything else in that model is correct, so it keeps the rest. A lift that is only rotated scores 6/6. This can be switched with `MIRROR_IS_DEFECT` in `scoring.py`.

## Reference tube

The Ø33.7 tube in the reference looks lighter than the drawing says (its BOM weight). Three of the examples match the drawing better here. That's why the grader allows small differences from the reference.

## Notes

- dummy_joints also loses articulation, because its extra joints make the assembly fail to solve.
- No LLM judge: everything could be measured, and that is easier to check.
- Limitations: only FreeCAD's built-in Assembly is supported; articulation needs a slider; parameters are tested through sketches.