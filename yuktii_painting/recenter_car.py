# ~/Documents/YUKTII AI LAB/yuktii_painting/recenter_car.py
# Run with RoboDK open:  python3 recenter_car.py
import os
import struct
import tempfile

from robodk.robolink import Robolink, ITEM_TYPE_OBJECT, ITEM_TYPE_FRAME
from robodk.robomath import transl

CAR_NAME = "CAN_CAR_BLEND_V12"
FRAME_NAME = "CarFrame"
TARGET_LENGTH = 4800.0
TOL = 20.0   # mm


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


def bbox_in_parent(pose, lo, hi):
    """Bounding box of the car's local box after applying its pose (axis-aligned rotation)."""
    pos, vx, vy, vz = pose.Pos(), pose.VX(), pose.VY(), pose.VZ()
    mins = [float("inf")] * 3
    maxs = [float("-inf")] * 3
    for x in (lo[0], hi[0]):
        for y in (lo[1], hi[1]):
            for z in (lo[2], hi[2]):
                for k in range(3):
                    v = pos[k] + vx[k] * x + vy[k] * y + vz[k] * z
                    mins[k] = min(mins[k], v)
                    maxs[k] = max(maxs[k], v)
    return mins, maxs


def report(tag, mins, maxs):
    print("%s: x %.0f..%.0f | y %.0f..%.0f | z %.0f..%.0f"
          % (tag, mins[0], maxs[0], mins[1], maxs[1], mins[2], maxs[2]))
    print("   centre x=%.0f y=%.0f, floor z=%.0f" %
          ((mins[0] + maxs[0]) / 2.0, (mins[1] + maxs[1]) / 2.0, mins[2]))


def main():
    rdk = Robolink()
    car = rdk.Item(CAR_NAME, ITEM_TYPE_OBJECT)
    frame = rdk.Item(FRAME_NAME, ITEM_TYPE_FRAME)
    if not car.Valid() or not frame.Valid():
        raise RuntimeError("Car '%s' or frame '%s' not found." % (CAR_NAME, FRAME_NAME))
    if car.Parent().Name() != FRAME_NAME:
        raise RuntimeError("The car is not under %s (parent is %s)." % (FRAME_NAME, car.Parent().Name()))

    path = os.path.join(tempfile.gettempdir(), "car_recenter_tmp.stl")
    if os.path.exists(path):
        os.remove(path)
    car.Save(path)
    if not os.path.isfile(path) or os.path.getsize(path) < 100:
        raise RuntimeError("RoboDK did not export the car; cannot measure it.")
    res = bbox_from_stl(path)
    if res is None:
        raise RuntimeError("Could not read the exported STL.")
    lo, hi = res
    ext = [hi[k] - lo[k] for k in range(3)]
    print("Exported geometry extents (model axes): %.0f x %.0f x %.0f" % tuple(ext))
    if abs(max(ext) - TARGET_LENGTH) / TARGET_LENGTH > 0.02:
        print("WARNING: the export is %.0f mm long, not about %.0f. The scale is not baked "
              "into the exported geometry, so I cannot measure reliably. Nothing was changed."
              % (max(ext), TARGET_LENGTH))
        return

    mins, maxs = bbox_in_parent(car.Pose(), lo, hi)
    report("Car in %s (before)" % FRAME_NAME, mins, maxs)
    cx, cy, zmin = (mins[0] + maxs[0]) / 2.0, (mins[1] + maxs[1]) / 2.0, mins[2]
    if abs(cx) <= TOL and abs(cy) <= TOL and abs(zmin) <= TOL:
        print("Car is already centred within %.0f mm. Nothing changed." % TOL)
        return

    car.setPose(transl(-cx, -cy, -zmin) * car.Pose())
    mins, maxs = bbox_in_parent(car.Pose(), lo, hi)
    report("Car in %s (after)" % FRAME_NAME, mins, maxs)
    print("Moved the car by dx=%.0f dy=%.0f dz=%.0f inside %s." % (-cx, -cy, -zmin, FRAME_NAME))
    print("Now check the CHK_Nose / CHK_Tail markers are at the bumpers.")


if __name__ == "__main__":
    main()
    