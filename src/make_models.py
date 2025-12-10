import FreeCAD as App
import Part, Mesh
import os

# Dimensions
L_block, W_block, H_block = 0.10, 0.10, 0.05
hole_r = 0.011
peg_r, peg_h = 0.01, 0.06
head_w, head_t, head_h = 0.04, 0.04, 0.06
handle_r, handle_l = 0.01, 0.20

out_dir = os.path.expanduser("~/peg_hammer_models")
os.makedirs(out_dir, exist_ok=True)

doc = App.newDocument("Models")

block = Part.makeBox(L_block, W_block, H_block)
block.translate(App.Vector(-L_block/2, -W_block/2, 0))
hole = Part.makeCylinder(hole_r, H_block + 0.01)
hole.translate(App.Vector(0, 0, -0.005))
block_hole = block.cut(hole)
doc.addObject("Part::Feature", "BlockWithHole").Shape = block_hole

peg = Part.makeCylinder(peg_r, peg_h)
doc.addObject("Part::Feature", "Peg").Shape = peg

handle = Part.makeCylinder(handle_r, handle_l)
head = Part.makeBox(head_w, head_t, head_h)
head.translate(App.Vector(-head_w/2, -head_t/2, handle_l * 0.8 - head_h/2))
hammer = handle.fuse(head)
doc.addObject("Part::Feature", "Hammer").Shape = hammer

doc.recompute()
Mesh.export([doc.getObject("BlockWithHole")], f"{out_dir}/block_with_hole.stl")
Mesh.export([doc.getObject("Peg")], f"{out_dir}/peg.stl")
Mesh.export([doc.getObject("Hammer")], f"{out_dir}/hammer.stl")
print("STLs exported to:", out_dir)

