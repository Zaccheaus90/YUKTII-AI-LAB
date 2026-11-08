# ~/Documents/YUKTII AI LAB/yuktii_painting/check_levels.py
# Read-only check of tilt. Run with RoboDK open:  python3 check_levels.py
import os
import struct
import tempfile
from math import acos, degrees

from robodk.robolink import Robolink, ITEM_TYPE_FRAME, ITEM_TYPE_OBJECT, ITEM_TYPE_ROBOT
from robodk.robomath import pose_2_xyzrpw

CAR_NAME = "CAN_CAR_BLEND_V12"
FRAME_NAME = "CarFrame"


def bbox_from_stl(path):
    lo = [float("inf")] * 3
    hi = [float("-inf")] * 3
    size = os.path.getsize(path)
    with open(path, "rb") as fh:
        head = fh.read(84)
        count = struct.unpack("<I", head[80:84])[0] if len(head) == 84 else -1
        if len(head) == 84 and 84 + 50 * count == size:      # binary STL
            data = fh.read()
            for i in range(count):
                vals = struct.unpack("<12fH", data[i * 50:(i + 1) * 50])
                for v in range(3):
                    p = vals[3 + 3 * v:6 + 3 * v]
                    for k in range(3):
                        lo[k] = min(lo[k], p[k])
                        hi[k] = max(hi[k], p[k])
            return lo, hi
    with open(path, "r", errors="ignore") as fh:             # ASCII STL
        for line in fh:
            parts = line.split()
            if len(parts) == 4 and parts[0].lower() == "vertex":
                p = [float(x) for x in parts[1:]]
                for k in range(3):
                    lo[k] = min(lo[k], p[k])
                    hi[k] = max(hi[k], p[k])
    return (lo, hi) if lo[0] != float("inf") else None


def local_bbox(car):
    path = os.path.join(tempfile.gettempdir(), "car_levels_tmp.stl")
    if os.path.exists(path):
        os.remove(path)
    car.Save(path)
    res = bbox_from_stl(path) if os.path.isfile(path) else None
    if res is None:
        raise RuntimeError("Could not export/read the car geometry.")
    return res


def corners(pose, lo, hi):
    pos, vx, vy, vz = pose.Pos(), pose.VX(), pose.VY(), pose.VZ()
    pts = []
    for x in (lo[0], hi[0]):
        for y in (lo[1], hi[1]):
            for z in (lo[2], hi[2]):
                pts.append([pos[k] + vx[k] * x + vy[k] * y + vz[k] * z for k in range(3)])
    return pts


def extents(pts):
    return ([min(p[k] for p in pts) for k in range(3)],
            [max(p[k] for p in pts) for k in range(3)])


def describe(pose):
    v = pose_2_xyzrpw(pose)
    return "xyz=(%.0f, %.0f, %.0f) mm | roll=%.1f pitch=%.1f yaw=%.1f deg" % tuple(v[:6])


def main():
    rdk = Robolink()
    robots = rdk.ItemList(ITEM_TYPE_ROBOT)
    car = rdk.Item(CAR_NAME, ITEM_TYPE_OBJECT)
    frame = rdk.Item(FRAME_NAME, ITEM_TYPE_FRAME)
    if not robots or not car.Valid() or not frame.Valid():
        raise RuntimeError("Robot, car or CarFrame not found.")
    base_abs = robots[0].PoseAbs()
    frame_abs = frame.PoseAbs()
    car_abs = car.PoseAbs()

    print("Robot base (absolute):          ", describe(base_abs))
    print("CarFrame (absolute):            ", describe(frame_abs))
    rel_frame = base_abs.inv() * frame_abs
    print("CarFrame relative to robot base:", describe(rel_frame))
    tilt = degrees(acos(max(-1.0, min(1.0, rel_frame.VZ()[2]))))
    print("  -> CarFrame's up axis is tilted %.1f degrees from the robot base's up axis" % tilt)
    print("Car relative to CarFrame:       ", describe(frame_abs.inv() * car_abs))

    lo, hi = local_bbox(car)
    print("Car geometry size (model axes): %.0f x %.0f x %.0f" % tuple(hi[k] - lo[k] for k in range(3)))
    for tag, pose in (("in CarFrame coordinates", frame_abs.inv() * car_abs),
                      ("in robot-base coordinates", base_abs.inv() * car_abs),
                      ("in absolute coordinates", car_abs)):
        mins, maxs = extents(corners(pose, lo, hi))
        print("Car extents %-27s x %.0f..%.0f | y %.0f..%.0f | z %.0f..%.0f"
              % (tag + ":", mins[0], maxs[0], mins[1], maxs[1], mins[2], maxs[2]))

    conv = rdk.Item("Conveyor")
    if conv.Valid():
        print("Conveyor parent:", conv.Parent().Name(), "|", describe(conv.PoseAbs()))
    else:
        print("No Conveyor in the station.")
    print("A level car has z about 0..1492 in CarFrame AND in robot-base coordinates.")


if __name__ == "__main__":
    main()
