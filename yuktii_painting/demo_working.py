# ~/Documents/YUKTII AI LAB/yuktii_painting/demo_working.py
# Plays only the zones that verified clean.  Run:  python3 demo_working.py
import json
import time

from robodk.robolink import ITEM_TYPE_PROGRAM, Robolink

from paint_cell_v9 import POSITIONS_FILE, get_items, move_car

MIN_RATIO = 0.95


def main():
    rdk = Robolink()
    robot, tool, frame = get_items(rdk)
    with open(POSITIONS_FILE) as fh:
        data = json.load(fh)
    base_abs = robot.PoseAbs()
    zones = {k: v for k, v in data["zones"].items() if v["valid_ratio"] >= MIN_RATIO}
    skipped = [k for k in data["zones"] if k not in zones]
    print("Playing:", ", ".join(zones))
    if skipped:
        print("Skipped (below %.2f):" % MIN_RATIO, ", ".join(skipped))
    order = sorted(zones.items(), key=lambda kv: (kv[1]["dy"] < 0, kv[1]["yaw"], kv[1]["dx"]))
    cur = None
    for zname, z in order:
        new = (z["dx"], z["dy"], z["yaw"])
        print("Moving car to dx=%d dy=%d yaw=%d, painting %s" % (new + (zname,)))
        move_car(frame, base_abs, cur, new)
        cur = new
        prog = rdk.Item("Paint_" + zname, ITEM_TYPE_PROGRAM)
        if not prog.Valid():
            raise RuntimeError("Program Paint_%s not found. Run paint_cell_v9.py first." % zname)
        prog.RunProgram()
        time.sleep(0.5)
        while prog.Busy():
            time.sleep(0.2)
    print("Demo finished.")


if __name__ == "__main__":
    main()