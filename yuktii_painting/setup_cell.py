# ~/Documents/YUKTII AI LAB/yuktii_painting/setup_cell.py
# Run with RoboDK open and your station loaded:  python3 setup_cell.py
import os
import struct
import tempfile
from math import radians

from robodk.robolink import (
    Robolink, ITEM_TYPE_ROBOT, ITEM_TYPE_FRAME, ITEM_TYPE_TOOL,
    ITEM_TYPE_OBJECT, ITEM_TYPE_TARGET,
)
from robodk.robomath import Mat, transl, rotz

try:
    from robodk.robolink import FEATURE_MESH
except ImportError:
    FEATURE_MESH = 9
try:
    from robodk.robolink import FEATURE_SURFACE
except ImportError:
    FEATURE_SURFACE = 1

# ---------------- Settings ----------------
CAR_NAME = "CAN_CAR_BLEND_V12"
FRAME_NAME = "CarFrame"
TARGET_LENGTH, TARGET_WIDTH, TARGET_HEIGHT = 4800.0, 1900.0, 1600.0   # mm
START_DX, START_DY, START_YAW = 0.0, 1700.0, 0.0   # CarFrame start, relative to robot base

# Method 3: path to the car's own file (.stl or .obj). Leave None if you don't have one.
# Example: CAR_FILE = "/home/zaccuit/Downloads/my_car.stl"
CAR_FILE = None
FILE_UNIT_FACTOR = 1.0     # multiply file coordinates by this to get mm (1000 if the file is in metres)

# Method 4: sizes you read yourself, in the model's own x, y, z directions (mm). Leave None to skip.
# Example: MANUAL_EXTENTS = (4300.0, 1800.0, 1450.0)
MANUAL_EXTENTS = None
MANUAL_CENTRE = (0.0, 0.0, 0.0)   # centre of that box in the model's own coordinates

# Leave None for automatic axis detection. Use "x", "y" or "z" (the model's own axes) to override.
LENGTH_AXIS = None
UP_AXIS = None
FLIP_FRONT = False    # set True if the nose points to -X after running
FLIP_UP = False       # set True if the car is upside down after running
ADD_MARKERS = True
AXIS_NAMES = "xyz"


# ---------------- Measuring ----------------
def bbox_from_api(item):
    """Method 1: mesh points straight from RoboDK."""
    for feature in (FEATURE_MESH, FEATURE_SURFACE):
        try:
            res = item.GetPoints(feature)
        except Exception as exc:
            print("  GetPoints(feature=%s) raised: %s" % (feature, exc))
            continue
        pts = res[0] if isinstance(res, tuple) else res
        rows = pts.rows if hasattr(pts, "rows") else pts
        try:
            rows = [list(r) for r in rows]
        except TypeError:
            rows = []
        print("  GetPoints(feature=%s) returned %d rows" % (feature, len(rows)))
        if not rows or len(rows[0]) < 3:
            continue
        if len(rows) <= 6 and len(rows[0]) > 6:       # stored column-wise
            cols = [rows[0], rows[1], rows[2]]
        else:                                         # one point per row
            cols = [[r[0] for r in rows], [r[1] for r in rows], [r[2] for r in rows]]
        print("  read %d mesh points from RoboDK" % len(cols[0]))
        return [min(c) for c in cols], [max(c) for c in cols]
    return None


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


def bbox_from_obj(path):
    lo = [float("inf")] * 3
    hi = [float("-inf")] * 3
    with open(path, "r", errors="ignore") as fh:
        for line in fh:
            if line.startswith("v "):
                p = [float(x) for x in line.split()[1:4]]
                for k in range(3):
                    lo[k] = min(lo[k], p[k])
                    hi[k] = max(hi[k], p[k])
    return (lo, hi) if lo[0] != float("inf") else None


def bbox_from_export(item):
    """Method 2: ask RoboDK to export the car to a temporary STL, then read that file."""
    path = os.path.join(tempfile.gettempdir(), "car_export_tmp.stl")
    if os.path.exists(path):
        os.remove(path)
    try:
        item.Save(path)
    except Exception as exc:
        print("  item.Save raised: %s" % exc)
        return None
    if not os.path.isfile(path) or os.path.getsize(path) < 100:
        print("  RoboDK did not write an STL export to %s" % path)
        return None
    res = bbox_from_stl(path)
    if res is None:
        print("  could not read the exported STL")
        return None
    print("  exported the car through RoboDK and read %s" % path)
    return res


def bbox_from_file(path):
    """Method 3: read the model file directly."""
    if not path:
        return None
    if not os.path.isfile(path):
        print("  CAR_FILE not found: %s" % path)
        return None
    ext = os.path.splitext(path)[1].lower()
    if ext == ".stl":
        res = bbox_from_stl(path)
    elif ext == ".obj":
        res = bbox_from_obj(path)
    else:
        print("  CAR_FILE must be .stl or .obj (got %s)" % ext)
        return None
    if res is None:
        return None
    lo = [v * FILE_UNIT_FACTOR for v in res[0]]
    hi = [v * FILE_UNIT_FACTOR for v in res[1]]
    print("  read the model file directly: %s" % path)
    return lo, hi


def bbox_from_manual():
    """Method 4: sizes typed in by hand."""
    if not MANUAL_EXTENTS:
        return None
    lo = [MANUAL_CENTRE[k] - MANUAL_EXTENTS[k] / 2.0 for k in range(3)]
    hi = [MANUAL_CENTRE[k] + MANUAL_EXTENTS[k] / 2.0 for k in range(3)]
    print("  using MANUAL_EXTENTS")
    return lo, hi


def measure(item):
    print("Measuring car...")
    for name, fn in (("RoboDK mesh", lambda: bbox_from_api(item)),
                     ("RoboDK export", lambda: bbox_from_export(item)),
                     ("model file", lambda: bbox_from_file(CAR_FILE)),
                     ("manual sizes", bbox_from_manual)):
        try:
            res = fn()
        except Exception as exc:
            print("  %s failed: %s" % (name, exc))
            res = None
        if res is not None:
            return res
    raise RuntimeError(
        "Could not measure the car. Set CAR_FILE to the car's real .stl/.obj path, or fill in "
        "MANUAL_EXTENTS (and MANUAL_CENTRE) at the top of this script.")


def fmt(v):
    return "[" + ", ".join("%.1f" % x for x in v) + "]"


# ---------------- Main ----------------
def main():
    rdk = Robolink()

    robots = rdk.ItemList(ITEM_TYPE_ROBOT)
    tools = rdk.ItemList(ITEM_TYPE_TOOL)
    if not robots:
        raise RuntimeError("No robot found in the station.")
    robot = robots[0]
    base_abs = robot.PoseAbs()

    car = rdk.Item(CAR_NAME, ITEM_TYPE_OBJECT)
    if not car.Valid():
        names = [o.Name() for o in rdk.ItemList(ITEM_TYPE_OBJECT)]
        raise RuntimeError("Car object '%s' not found. Objects in station: %s" % (CAR_NAME, names))

    frame = rdk.Item(FRAME_NAME, ITEM_TYPE_FRAME)
    if not frame.Valid():
        frame = rdk.AddFrame(FRAME_NAME)
        print("Created reference frame", FRAME_NAME)

    # ---- 1) Measure the car in its own coordinates ----
    lo, hi = measure(car)
    ext = [hi[k] - lo[k] for k in range(3)]
    print("  extents (model units): x=%.2f y=%.2f z=%.2f" % tuple(ext))
    if max(ext) < 50:
        print("  WARNING: the car looks tiny (under 50 units). If the model is in metres, "
              "set FILE_UNIT_FACTOR = 1000 or enter MANUAL_EXTENTS in mm.")

    # ---- 2) Decide which model axis is length / width / up ----
    L = AXIS_NAMES.index(LENGTH_AXIS) if LENGTH_AXIS else max(range(3), key=lambda k: ext[k])
    rest = [k for k in range(3) if k != L]
    U = AXIS_NAMES.index(UP_AXIS) if UP_AXIS else min(rest, key=lambda k: ext[k])
    if U == L:
        raise RuntimeError("LENGTH_AXIS and UP_AXIS must differ.")
    W = [k for k in range(3) if k not in (L, U)][0]
    print("  length axis=%s, width axis=%s, up axis=%s"
          % (AXIS_NAMES[L], AXIS_NAMES[W], AXIS_NAMES[U]))

    # ---- 3) Scale uniformly so the length is 4800 mm ----
    scale = TARGET_LENGTH / ext[L]
    if abs(scale - 1.0) > 1e-3:
        print("Scaling car by %.5f" % scale)
        car.Scale(scale)
        lo = [v * scale for v in lo]       # the model is scaled about its own origin
        hi = [v * scale for v in hi]
    else:
        print("Car is already 4800 mm long, no scaling needed.")

    width = hi[W] - lo[W]
    height = hi[U] - lo[U]
    print("Car size now: length=%.0f width=%.0f height=%.0f mm (targets %.0f / %.0f / %.0f)"
          % (hi[L] - lo[L], width, height, TARGET_LENGTH, TARGET_WIDTH, TARGET_HEIGHT))
    for name, got, want in (("width", width, TARGET_WIDTH), ("height", height, TARGET_HEIGHT)):
        if abs(got - want) / want > 0.15:
            print("  WARNING: %s is %.0f%% off the target. Uniform scaling keeps the model's "
                  "proportions; the paint script assumes the target size." %
                  (name, 100.0 * (got - want) / want))

    # ---- 4) Pose: model axes -> CarFrame (X forward, Y left, Z up) ----
    sL = sW = sU = 1.0
    if FLIP_FRONT:
        sL, sW = -sL, -sW
    if FLIP_UP:
        sW, sU = -sW, -sU
    r = [[0.0] * 3 for _ in range(3)]
    r[0][L], r[1][W], r[2][U] = sL, sW, sU
    det = (r[0][0] * (r[1][1] * r[2][2] - r[1][2] * r[2][1])
           - r[0][1] * (r[1][0] * r[2][2] - r[1][2] * r[2][0])
           + r[0][2] * (r[1][0] * r[2][1] - r[1][1] * r[2][0]))
    if det < 0:                       # keep a proper rotation (the car is left/right symmetric)
        sW = -sW
        r[1][W] = sW

    def rng(idx, sign):
        a, b = sign * lo[idx], sign * hi[idx]
        return min(a, b), max(a, b)

    x0, x1 = rng(L, sL)
    y0, y1 = rng(W, sW)
    z0, _ = rng(U, sU)
    t = (-(x0 + x1) / 2.0, -(y0 + y1) / 2.0, -z0)
    pose = Mat([[r[0][0], r[0][1], r[0][2], t[0]],
                [r[1][0], r[1][1], r[1][2], t[1]],
                [r[2][0], r[2][1], r[2][2], t[2]],
                [0, 0, 0, 1]])

    car.setParent(frame)
    car.setPose(pose)
    print("Car placed under '%s': centred on the frame origin, wheels at z=0." % FRAME_NAME)

    # ---- 5) Put CarFrame at its starting stop beside the robot ----
    frame.setPoseAbs(base_abs * transl(START_DX, START_DY, 0) * rotz(radians(START_YAW)))
    print("CarFrame at dx=%.0f dy=%.0f yaw=%.0f relative to the robot base."
          % (START_DX, START_DY, START_YAW))
    print("Gap between robot base axis and the car side: %.0f mm" % (START_DY - width / 2.0))

    # ---- 6) Report the frames ----
    print("\nStation check")
    print("  robot base position (abs, mm):", fmt(base_abs.Pos()))
    print("  car parent:", car.Parent().Name())
    if tools:
        tool = tools[0]
        tp = tool.PoseTool()
        print("  tool '%s' parent: %s" % (tool.Name(), tool.Parent().Name()))
        print("  tool TCP position (mm):", fmt(tp.Pos()), " Z axis:", fmt(tp.VZ()))
    else:
        print("  WARNING: no tool item found.")
    print("  The paint script assumes the car's floor is at the same height as the robot base.")

    # ---- 7) Markers where the paint script assumes the surfaces are ----
    for tgt in rdk.ItemList(ITEM_TYPE_TARGET):
        if tgt.Name().startswith("CHK_"):
            tgt.Delete()
    if ADD_MARKERS:
        hw = TARGET_WIDTH / 2.0
        pts = {
            "CHK_Left_low": (0, hw, 450), "CHK_Left_high": (0, hw, 1150),
            "CHK_Right_low": (0, -hw, 450), "CHK_Right_high": (0, -hw, 1150),
            "CHK_Roof": (0, 0, TARGET_HEIGHT),
            "CHK_Hood": (1500, 0, 1000), "CHK_Trunk": (-1500, 0, 1050),
            "CHK_Nose": (TARGET_LENGTH / 2.0, 0, 600),
            "CHK_Tail": (-TARGET_LENGTH / 2.0, 0, 600),
        }
        for name, (x, y, z) in pts.items():
            m = rdk.AddTarget(name, frame)
            m.setPose(transl(x, y, z))
        print("\nAdded CHK_ markers. Look at them in the 3D view.")


if __name__ == "__main__":
    main()