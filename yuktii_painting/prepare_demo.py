# ~/Documents/YUKTII AI LAB/yuktii_painting/prepare_demo.py
# Hide markers and fix the conveyor parent:  python3 prepare_demo.py
# Show the markers again:                    python3 prepare_demo.py show
import sys

from robodk.robolink import ITEM_TYPE_FRAME, ITEM_TYPE_ROBOT, ITEM_TYPE_TARGET, Robolink
from robodk.robomath import transl

from paint_cell_v11 import CONV_X_MAX, CONV_X_MIN, DECK_Y_MAX, DECK_Y_MIN, SLAB_TOP_Z


def conveyor_follows_car(frame, conv):
    """Move CarFrame 1000 mm along X and see whether the conveyor moves with it."""
    before = conv.PoseAbs().Pos()
    original = frame.PoseAbs()
    frame.setPoseAbs(transl(1000, 0, 0) * original)
    after = conv.PoseAbs().Pos()
    frame.setPoseAbs(original)
    return abs(after[0] - before[0]) > 1.0


def main():
    show = len(sys.argv) > 1 and sys.argv[1] == "show"
    rdk = Robolink()
    robots = rdk.ItemList(ITEM_TYPE_ROBOT)
    frame = rdk.Item("CarFrame", ITEM_TYPE_FRAME)
    if not robots or not frame.Valid():
        raise RuntimeError("Robot or CarFrame not found.")
    base_abs = robots[0].PoseAbs()

    conv = rdk.Item("Conveyor")
    if not conv.Valid():
        print("No Conveyor in the station. Run add_conveyor.py first.")
    else:
        print("Conveyor parent:", conv.Parent().Name())
        follows = conveyor_follows_car(frame, conv)
        print("Conveyor moves with CarFrame:", follows)
        if follows:
            station = rdk.ActiveStation()
            try:
                conv.setParentStatic(station)
            except Exception as exc:
                print("setParentStatic failed (%s), trying setParent" % exc)
                conv.setParent(station)
            centre_x = (CONV_X_MIN + CONV_X_MAX) / 2.0
            centre_y = (DECK_Y_MIN + DECK_Y_MAX) / 2.0
            conv.setPoseAbs(base_abs * transl(centre_x, centre_y, SLAB_TOP_Z))
            print("After fix, conveyor parent:", conv.Parent().Name())
            print("After fix, conveyor still moves with CarFrame:", conveyor_follows_car(frame, conv))
        else:
            print("The conveyor stays put when the car moves. Good.")

    count = 0
    for t in rdk.ItemList(ITEM_TYPE_TARGET):
        name = t.Name()
        if name.startswith("P_") or name.startswith("CHK_"):
            try:
                t.setVisible(show, show)
                count += 1
            except Exception:
                pass
    print("%s %d target markers." % ("Showed" if show else "Hid", count))


if __name__ == "__main__":
    main()
