# ~/Documents/YUKTII AI LAB/yuktii_painting/add_conveyor.py
# Run BEFORE paint_cell_v11.py:  python3 add_conveyor.py
import os

from robodk.robolink import Robolink, ITEM_TYPE_ROBOT
from robodk.robomath import transl

from paint_cell_v11 import (
    CONV_X_MAX, CONV_X_MIN, DECK_Y_MAX, DECK_Y_MIN, SLAB_THICK, SLAB_TOP_Z,
)

HERE = os.path.dirname(os.path.abspath(__file__))
STL_PATH = os.path.join(HERE, "conveyor.stl")


def write_box_stl(path, sx, sy, height):
    """Box centred in X/Y, top face at z=0, extending down by 'height'."""
    x0, x1 = -sx / 2.0, sx / 2.0
    y0, y1 = -sy / 2.0, sy / 2.0
    z0, z1 = -height, 0.0
    v = [(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0),
         (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)]
    quads = [(0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4),
             (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)]
    with open(path, "w") as fh:
        fh.write("solid box\n")
        for a, b, c, d in quads:
            for tri in ((a, b, c), (a, c, d)):
                fh.write("facet normal 0 0 0\nouter loop\n")
                for idx in tri:
                    fh.write("vertex %f %f %f\n" % v[idx])
                fh.write("endloop\nendfacet\n")
        fh.write("endsolid box\n")


def main():
    rdk = Robolink()
    robots = rdk.ItemList(ITEM_TYPE_ROBOT)
    if not robots:
        raise RuntimeError("No robot found in the station.")
    base_abs = robots[0].PoseAbs()
    station = rdk.ActiveStation()

    # Remove any conveyor from a previous run
    old = rdk.Item("Conveyor")
    while old.Valid():
        old.Delete()
        old = rdk.Item("Conveyor")

    length = CONV_X_MAX - CONV_X_MIN
    width = DECK_Y_MAX - DECK_Y_MIN
    write_box_stl(STL_PATH, length, width, SLAB_THICK)
    item = rdk.AddFile(STL_PATH)
    if not item.Valid():
        raise RuntimeError("RoboDK could not import %s" % STL_PATH)
    item.setName("Conveyor")

    # Attach to the station itself, NOT to CarFrame, so it stays put when the car moves
    item.setParent(station)
    centre_x = (CONV_X_MIN + CONV_X_MAX) / 2.0
    centre_y = (DECK_Y_MIN + DECK_Y_MAX) / 2.0
    item.setPoseAbs(base_abs * transl(centre_x, centre_y, SLAB_TOP_Z))
    try:
        item.setColor([0.25, 0.27, 0.32, 1.0])
    except Exception:
        pass
    print("Conveyor deck added: %.0f mm long, %.0f mm wide, covering y %.0f to %.0f mm from the robot base."
          % (length, width, DECK_Y_MIN, DECK_Y_MAX))
    print("Conveyor parent:", item.Parent().Name(), "(should be the station, not CarFrame)")


if __name__ == "__main__":
    main()