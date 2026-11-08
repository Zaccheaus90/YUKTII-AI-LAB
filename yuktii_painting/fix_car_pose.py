# ~/Documents/YUKTII AI LAB/yuktii_painting/fix_car_pose.py
# Run with RoboDK open:  python3 fix_car_pose.py
import os
import struct
import tempfile
from math import asin, degrees

from robodk.robolink import Robolink, ITEM_TYPE_FRAME, ITEM_TYPE_OBJECT, ITEM_TYPE_TARGET
from robodk.robomath import Mat, transl

CAR_NAME = "CAN_CAR_BLEND_V12"
FRAME_NAME = "CarFrame"
TARGET_LENGTH = 4800.0
FLIP_FRONT = False     # set True if CHK_Nose ends up at the rear bumper after running
FLIP_UP = False        # set True if the car ends up upside down


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
    path = os.path.join(tempfile.gettempdir(), "car_fix_tmp.stl")
    if os.path.exists(path):
        os.remove(path)
    car.Save(path)
    if not os.path.isfile(path) or os.path.getsize(path) < 100:
        raise RuntimeError("RoboDK did not export the car, so it cannot be measured.")
    res = bbox_from_stl(path)
    if res is None:
        raise RuntimeError("Could not read the exported STL.")
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
    mins = [min(p[k] for p in pts) for k in range(3)]
    maxs = [max(p[k] for p in pts) for k in range(3)]
    return mins, maxs


def show(tag, mins, maxs):
    print("%s: x %.0f..%.0f | y %.0f..%.0f | z %.0f..%.0f" %
          (tag, mins[0], maxs[0], mins[1], maxs[1], mins[2], maxs[2]))


def det3(r):
    return (r[0][0] * (r[1][1] * r[2][2] - r[1][2] * r[2][1])
            - r[0][1] * (r[1][0] * r[2][2] - r[1][2] * r[2][0])
            + r[0][2] * (r[1][0] * r[2][1] - r[1][1] * r[2][0]))


def main():
    rdk = Robolink()
    car = rdk.Item(CAR_NAME, ITEM_TYPE_OBJECT)
    frame = rdk.Item(FRAME_NAME, ITEM_TYPE_FRAME)
    if not car.Valid() or not frame.Valid():
        raise RuntimeError("Car '%s' or frame '%s' not found." % (CAR_NAME, FRAME_NAME))
    print("Car parent:", car.Parent().Name())
    if car.Parent().Name() != FRAME_NAME:
        raise RuntimeError("The car is not under %s. Drag it under that frame in the tree first."
                           % FRAME_NAME)

    # ---- 1) Check the scale, fix it once, and verify by exporting again ----
    lo, hi = local_bbox(car)
    ext = [hi[k] - lo[k] for k in range(3)]
    print("Exported geometry extents (model axes): %.0f x %.0f x %.0f" % tuple(ext))
    if abs(max(ext) - TARGET_LENGTH) / TARGET_LENGTH > 0.02:
        scale = TARGET_LENGTH / max(ext)
        print("The car is not %.0f mm long. Scaling it once by %.5f ..." % (TARGET_LENGTH, scale))
        car.Scale(scale)
        lo, hi = local_bbox(car)
        ext = [hi[k] - lo[k] for k in range(3)]
        print("Exported extents after scaling: %.0f x %.0f x %.0f" % tuple(ext))
        if abs(max(ext) - TARGET_LENGTH) / TARGET_LENGTH > 0.02:
            print("The scale did not show up in the export. STOP: do not run this script again, "
                  "or the car may be scaled twice. Send me this output.")
            return
    else:
        print("Scale is fine (length is about %.0f mm)." % TARGET_LENGTH)

    # ---- 2) Which model axis is length, height and width? ----
    rel = frame.PoseAbs().inv() * car.PoseAbs()      # current car pose in CarFrame
    mins, maxs = extents(corners(rel, lo, hi))
    show("BEFORE (car in CarFrame)", mins, maxs)
    cols = [rel.VX(), rel.VY(), rel.VZ()]
    L = max(range(3), key=lambda k: ext[k])
    U = max([k for k in range(3) if k != L], key=lambda k: abs(cols[k][2]))
    W = [k for k in range(3) if k not in (L, U)][0]
    print("Length axis tilt from horizontal: %.0f degrees" % degrees(asin(min(1.0, abs(cols[L][2])))))
    print("Model axes: length=%s height=%s width=%s (x,y,z = 0,1,2)" % ("xyz"[L], "xyz"[U], "xyz"[W]))
    if ext[U] > ext[W]:
        print("WARNING: the 'height' axis is longer than the 'width' axis. Check the car is upright "
              "in the 3D view afterwards.")

    # ---- 3) Rebuild the orientation: length -> +X, height -> +Z, width -> +-Y ----
    sL = -1.0 if FLIP_FRONT else 1.0
    sU = -1.0 if FLIP_UP else 1.0
    sW = 1.0
    r = [[0.0] * 3 for _ in range(3)]
    r[0][L], r[1][W], r[2][U] = sL, sW, sU
    if det3(r) < 0:                    # keep a proper rotation (the car is left/right symmetric)
        sW = -sW
        r[1][W] = sW
    rot = Mat([[r[0][0], r[0][1], r[0][2], 0],
               [r[1][0], r[1][1], r[1][2], 0],
               [r[2][0], r[2][1], r[2][2], 0],
               [0, 0, 0, 1]])
    mins, maxs = extents(corners(rot, lo, hi))
    shift = (-(mins[0] + maxs[0]) / 2.0, -(mins[1] + maxs[1]) / 2.0, -mins[2])
    car.setPose(transl(shift[0], shift[1], shift[2]) * rot)   # relative to CarFrame

    rel2 = frame.PoseAbs().inv() * car.PoseAbs()
    mins2, maxs2 = extents(corners(rel2, lo, hi))
    show("AFTER  (car in CarFrame)", mins2, maxs2)
    print("Expected: x about -2400..2400, y about -1048..1048, z about 0..1492")

    # ---- 4) Visual check markers ----
    for t in rdk.ItemList(ITEM_TYPE_TARGET):
        if t.Name().startswith("CHK_"):
            t.Delete()
    hl, hw, height = maxs2[0], maxs2[1], maxs2[2]
    marks = {
        "CHK_Nose": (hl, 0, 600), "CHK_Tail": (-hl, 0, 600),
        "CHK_Left_door": (0, hw, 800), "CHK_Right_door": (0, -hw, 800),
        "CHK_Roof": (0, 0, height),
    }
    for name, (x, y, z) in marks.items():
        m = rdk.AddTarget(name, frame)
        m.setPose(transl(x, y, z))
    print("Added CHK_ markers.")

    # ---- 5) The conveyor must NOT be a child of CarFrame (it would move with the car) ----
    conv = rdk.Item("Conveyor")
    if conv.Valid() and conv.Parent().Name() == FRAME_NAME:
        conv.Delete()
        print("Deleted the Conveyor: it was attached to CarFrame. Run add_conveyor.py again.")
    elif conv.Valid():
        print("Conveyor parent:", conv.Parent().Name())
    else:
        print("No Conveyor in the station yet. Run add_conveyor.py next.")


if __name__ == "__main__":
    main()
