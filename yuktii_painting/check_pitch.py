# ~/Documents/YUKTII AI LAB/yuktii_painting/check_pitch.py
# Report only:   python3 check_pitch.py
# Level the car: python3 check_pitch.py apply
import os
import struct
import sys
import tempfile
from math import atan2, degrees

from robodk.robolink import Robolink, ITEM_TYPE_FRAME, ITEM_TYPE_OBJECT
from robodk.robomath import rotx, roty, transl

CAR_NAME = "CAN_CAR_BLEND_V12"
FRAME_NAME = "CarFrame"
LEVEL_TOL_DEG = 0.5


def read_vertices(path):
    size = os.path.getsize(path)
    verts = []
    with open(path, "rb") as fh:
        head = fh.read(84)
        count = struct.unpack("<I", head[80:84])[0] if len(head) == 84 else -1
        if len(head) == 84 and 84 + 50 * count == size:      # binary STL
            data = fh.read()
            for vals in struct.iter_unpack("<12fH", data):
                verts.append(vals[3:6])
                verts.append(vals[6:9])
                verts.append(vals[9:12])
            return verts
    with open(path, "r", errors="ignore") as fh:             # ASCII STL
        for line in fh:
            parts = line.split()
            if len(parts) == 4 and parts[0].lower() == "vertex":
                verts.append((float(parts[1]), float(parts[2]), float(parts[3])))
    return verts


def export_vertices(car):
    path = os.path.join(tempfile.gettempdir(), "car_pitch_tmp.stl")
    if os.path.exists(path):
        os.remove(path)
    car.Save(path)
    if not os.path.isfile(path) or os.path.getsize(path) < 100:
        raise RuntimeError("RoboDK did not export the car, so it cannot be measured.")
    verts = read_vertices(path)
    if not verts:
        raise RuntimeError("Could not read the exported STL.")
    return verts


def transform(verts, pose):
    pos, vx, vy, vz = pose.Pos(), pose.VX(), pose.VY(), pose.VZ()
    return [(pos[0] + vx[0] * x + vy[0] * y + vz[0] * z,
             pos[1] + vx[1] * x + vy[1] * y + vz[1] * z,
             pos[2] + vx[2] * x + vy[2] * y + vz[2] * z) for (x, y, z) in verts]


def lowest_by_quadrant(pts):
    best = {}
    for p in pts:
        key = (p[0] >= 0, p[1] >= 0)        # (front?, left?)
        if key not in best or p[2] < best[key][2]:
            best[key] = p
    return best


def tilt(best):
    avg = lambda items, i: sum(p[i] for p in items) / len(items)
    fr = [best[(True, True)], best[(True, False)]]
    rr = [best[(False, True)], best[(False, False)]]
    lf = [best[(True, True)], best[(False, True)]]
    rt = [best[(True, False)], best[(False, False)]]
    pitch = atan2(avg(fr, 2) - avg(rr, 2), avg(fr, 0) - avg(rr, 0))   # + = nose up
    roll = atan2(avg(lf, 2) - avg(rt, 2), avg(lf, 1) - avg(rt, 1))    # + = left side up
    return pitch, roll


def extents(pts):
    return ([min(p[k] for p in pts) for k in range(3)],
            [max(p[k] for p in pts) for k in range(3)])


def report(tag, pts):
    best = lowest_by_quadrant(pts)
    names = {(True, True): "front-left", (True, False): "front-right",
             (False, True): "rear-left", (False, False): "rear-right"}
    print(tag)
    for key in ((True, True), (True, False), (False, True), (False, False)):
        p = best[key]
        print("  lowest point %-11s x=%6.0f y=%6.0f z=%6.1f" % (names[key], p[0], p[1], p[2]))
    pitch, roll = tilt(best)
    print("  pitch %.2f deg (+ = nose up) | roll %.2f deg (+ = left side up)"
          % (degrees(pitch), degrees(roll)))
    return pitch, roll


def main():
    apply_fix = len(sys.argv) > 1 and sys.argv[1] == "apply"
    rdk = Robolink()
    car = rdk.Item(CAR_NAME, ITEM_TYPE_OBJECT)
    frame = rdk.Item(FRAME_NAME, ITEM_TYPE_FRAME)
    if not car.Valid() or not frame.Valid():
        raise RuntimeError("Car '%s' or frame '%s' not found." % (CAR_NAME, FRAME_NAME))
    if car.Parent().Name() != FRAME_NAME:
        raise RuntimeError("The car is not under %s." % FRAME_NAME)

    verts = export_vertices(car)
    print("Read %d mesh points from the car." % len(verts))
    rel = frame.PoseAbs().inv() * car.PoseAbs()
    pts = transform(verts, rel)
    mins, maxs = extents(pts)
    print("Car extents in CarFrame: x %.0f..%.0f | y %.0f..%.0f | z %.0f..%.0f"
          % (mins[0], maxs[0], mins[1], maxs[1], mins[2], maxs[2]))
    pitch, roll = report("BEFORE:", pts)

    if max(abs(degrees(pitch)), abs(degrees(roll))) <= LEVEL_TOL_DEG:
        print("The car sits level on its wheels (within %.1f deg). No tilt in the model itself." % LEVEL_TOL_DEG)
        return
    if not apply_fix:
        print("The car is tilted relative to its wheels. Run:  python3 check_pitch.py apply")
        return

    original = car.Pose()
    new_rel = rotx(-roll) * roty(pitch) * rel
    pts2 = transform(verts, new_rel)
    mins, maxs = extents(pts2)
    shift = (-(mins[0] + maxs[0]) / 2.0, -(mins[1] + maxs[1]) / 2.0, -mins[2])
    car.setPose(transl(shift[0], shift[1], shift[2]) * new_rel)

    rel2 = frame.PoseAbs().inv() * car.PoseAbs()
    pts3 = transform(verts, rel2)
    mins, maxs = extents(pts3)
    print("Car extents in CarFrame: x %.0f..%.0f | y %.0f..%.0f | z %.0f..%.0f"
          % (mins[0], maxs[0], mins[1], maxs[1], mins[2], maxs[2]))
    p2, r2 = report("AFTER:", pts3)
    if max(abs(p2), abs(r2)) >= max(abs(pitch), abs(roll)):
        car.setPose(original)
        print("The tilt did not improve, so the original pose was restored. Send me this output.")
    else:
        print("Done. Save the station (File > Save station) and look at the car in the 3D view.")


if __name__ == "__main__":
    main()
