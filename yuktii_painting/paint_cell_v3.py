# ~/Documents/YUKTII AI LAB/yuktii_painting/paint_cell_v3.py
# Build + fit:  python3 paint_cell_v3.py
# Demo run:     python3 paint_cell_v3.py demo
import json
import os
import sys
import time
from math import pi, radians

from robodk.robolink import (
    Robolink, ITEM_TYPE_ROBOT, ITEM_TYPE_FRAME, ITEM_TYPE_TOOL,
    ITEM_TYPE_TARGET, ITEM_TYPE_PROGRAM, COLLISION_ON,
)
from robodk.robomath import Mat, transl, rotx, roty, rotz

# ---------------- Parameters (mm) ----------------
CAR_LENGTH = 4800.0
CAR_WIDTH = 1900.0
CAR_HEIGHT = 1600.0
STANDOFF = 250.0
ROW_SPACING = 150.0
APPROACH = 200.0
TILT_DEG = 20.0
FRAME_NAME = "CarFrame"
DY_VALUES = [1700, 2000, 2300, 2600, 3000]    # car centre line distance from robot base
DX_VALUES = list(range(-3000, 3001, 500))     # stop positions along the conveyor
YAWS = (0, 180)                               # 180 = after the turntable
POSITIONS_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                              "car_positions_v3.json")

HALF_L, HALF_W = CAR_LENGTH / 2.0, CAR_WIDTH / 2.0
# Car frame: origin at car centre on the floor, X forward, Y left, Z up.
ZONES = {
    "Left_Front":  dict(normal=(0, 1, 0),  stroke=(1, 0, 0), row=(0, 0, 1),
                        fixed=(0, HALF_W, 0), stroke_range=(0, HALF_L - 700),
                        row_range=(450, 1150)),
    "Left_Rear":   dict(normal=(0, 1, 0),  stroke=(1, 0, 0), row=(0, 0, 1),
                        fixed=(0, HALF_W, 0), stroke_range=(-HALF_L + 700, 0),
                        row_range=(450, 1150)),
    "Right_Front": dict(normal=(0, -1, 0), stroke=(1, 0, 0), row=(0, 0, 1),
                        fixed=(0, -HALF_W, 0), stroke_range=(0, HALF_L - 700),
                        row_range=(450, 1150)),
    "Right_Rear":  dict(normal=(0, -1, 0), stroke=(1, 0, 0), row=(0, 0, 1),
                        fixed=(0, -HALF_W, 0), stroke_range=(-HALF_L + 700, 0),
                        row_range=(450, 1150)),
    "Roof":        dict(normal=(0, 0, 1),  stroke=(1, 0, 0), row=(0, 1, 0),
                        fixed=(0, 0, CAR_HEIGHT), stroke_range=(-600, 700),
                        row_range=(-HALF_W + 400, HALF_W - 400)),
    "Hood":        dict(normal=(0, 0, 1),  stroke=(1, 0, 0), row=(0, 1, 0),
                        fixed=(0, 0, 1000), stroke_range=(1000, 2000),
                        row_range=(-HALF_W + 400, HALF_W - 400)),
    "Trunk":       dict(normal=(0, 0, 1),  stroke=(1, 0, 0), row=(0, 1, 0),
                        fixed=(0, 0, 1050), stroke_range=(-2000, -1100),
                        row_range=(-HALF_W + 400, HALF_W - 400)),
}


# ---------------- Geometry helpers ----------------
def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def make_pose(origin, x_axis, z_axis):
    y_axis = cross(z_axis, x_axis)
    return Mat([
        [x_axis[0], y_axis[0], z_axis[0], origin[0]],
        [x_axis[1], y_axis[1], z_axis[1], origin[1]],
        [x_axis[2], y_axis[2], z_axis[2], origin[2]],
        [0, 0, 0, 1],
    ])


def frange(start, stop, step):
    values, v = [], start
    while v <= stop + 1e-6:
        values.append(v)
        v += step
    return values


def zone_targets(zone):
    """List of (label, pose_in_car_frame), approach point first."""
    n, s, r, f = zone["normal"], zone["stroke"], zone["row"], zone["fixed"]
    s0, s1 = zone["stroke_range"]
    z_axis = tuple(-c for c in n)
    out = []
    rows = frange(zone["row_range"][0], zone["row_range"][1], ROW_SPACING)
    for i, rv in enumerate(rows):
        a, b = (s0, s1) if i % 2 == 0 else (s1, s0)
        pts = [[f[k] + s[k] * sv + r[k] * rv + n[k] * STANDOFF for k in range(3)]
               for sv in (a, b)]
        x_dir = [pts[1][k] - pts[0][k] for k in range(3)]
        length = sum(c * c for c in x_dir) ** 0.5
        x_dir = tuple(c / length for c in x_dir)
        if i == 0:
            ap = [pts[0][k] + n[k] * APPROACH for k in range(3)]
            out.append(("approach", make_pose(ap, x_dir, z_axis)))
        out.append(("%02dS" % i, make_pose(pts[0], x_dir, z_axis)))
        out.append(("%02dE" % i, make_pose(pts[1], x_dir, z_axis)))
    return out


def variants(flip):
    """Allowed gun orientations: perpendicular first, then small tilts."""
    t = radians(TILT_DEG)
    base = [transl(0, 0, 0), rotx(t), rotx(-t), roty(t), roty(-t)]
    if flip:
        return [rotx(pi) * v for v in base]
    return base


def rel_pose(dx, dy, yaw):
    return transl(dx, dy, 0) * rotz(radians(yaw))


def car_state_pose(base_abs, state):
    return base_abs * transl(state[0], state[1], 0) * rotz(radians(state[2]))


# ---------------- RoboDK helpers ----------------
def get_items(rdk):
    robots = rdk.ItemList(ITEM_TYPE_ROBOT)
    tools = rdk.ItemList(ITEM_TYPE_TOOL)
    frame = rdk.Item(FRAME_NAME, ITEM_TYPE_FRAME)
    if not robots:
        raise RuntimeError("No robot found in the station.")
    if not tools:
        raise RuntimeError("No tool found in the station.")
    if not frame.Valid():
        raise RuntimeError("Reference frame '%s' not found." % FRAME_NAME)
    return robots[0], tools[0], frame


def ik_ok(robot, pose_base, tool_pose):
    try:
        j = robot.SolveIK(pose_base, None, tool_pose)
        return j is not None and len(j.list()) >= 6
    except Exception:
        return False


def score_at(robot, tool_pose, targets, rel, vlist):
    n = 0
    for _, p in targets:
        for v in vlist:
            if ik_ok(robot, rel * p * v, tool_pose):
                n += 1
                break
    return n


def pick_poses(robot, tool_pose, rel, targets, vlist):
    out = []
    for _, p in targets:
        chosen = p * vlist[0]
        for v in vlist:
            q = p * v
            if ik_ok(robot, rel * q, tool_pose):
                chosen = q
                break
        out.append(chosen)
    return out


# ---------------- Build and fit ----------------
def build_and_fit():
    rdk = Robolink()
    robot, tool, frame = get_items(rdk)
    robot.setPoseTool(tool)
    rdk.setCollisionActive(COLLISION_ON)
    tool_pose = tool.PoseTool()
    base_abs = robot.PoseAbs()

    print("Tool Z axis (should point out of the nozzle):",
          [round(v, 2) for v in tool_pose.VZ()])

    for p in rdk.ItemList(ITEM_TYPE_PROGRAM):
        if p.Name().startswith("Paint_"):
            p.Delete()
    for t in rdk.ItemList(ITEM_TYPE_TARGET):
        if t.Name().startswith("P_"):
            t.Delete()

    zone_t = {z: zone_targets(zone) for z, zone in ZONES.items()}
    cands = [(dx, dy, yaw) for dy in DY_VALUES for dx in DX_VALUES for yaw in YAWS]

    # 1) Decide whether the gun axis needs flipping (quick test, no tilts)
    print("Checking tool orientation...")
    totals = {}
    for flip in (False, True):
        vl = [variants(flip)[0]]
        total = 0
        for zname in ("Left_Front", "Roof"):
            total += max(score_at(robot, tool_pose, zone_t[zname],
                                  rel_pose(*c), vl) for c in cands)
        totals[flip] = total
    flip = totals[True] > totals[False]
    print("Reachable targets as defined: %d | flipped: %d -> using %s"
          % (totals[False], totals[True], "FLIPPED" if flip else "AS DEFINED"))
    vlist = variants(flip)

    # 2) Score every stop for every zone, pick one conveyor distance (dy)
    print("Searching stops (this can take a few minutes)...")
    scores = {}
    dy_total = {dy: 0 for dy in DY_VALUES}
    for dy in DY_VALUES:
        for zname, targets in zone_t.items():
            best = 0
            for dx in DX_VALUES:
                for yaw in YAWS:
                    s = score_at(robot, tool_pose, targets, rel_pose(dx, dy, yaw), vlist)
                    scores[(zname, dy, dx, yaw)] = s
                    best = max(best, s)
            dy_total[dy] += best
        print("  conveyor distance %4d mm -> total reachable %d" % (dy, dy_total[dy]))
    dy_c = max(DY_VALUES, key=lambda d: (dy_total[d], -d))
    print("Chosen conveyor distance: %d mm" % dy_c)

    # 3) Build programs, verify shortlisted stops with the full collision check
    home = rdk.AddTarget("P_Home", frame, robot)
    home.setAsJointTarget()
    home.setJoints(robot.JointsHome())

    result = {"flip": flip, "dy": dy_c, "zones": {}}
    for zname, targets in zone_t.items():
        ranked = sorted(
            [(scores[(zname, dy_c, dx, yaw)], -abs(dx), dx, yaw)
             for dx in DX_VALUES for yaw in YAWS], reverse=True)
        shortlist = ranked[:4]

        prog = rdk.AddProgram("Paint_" + zname, robot)
        prog.setFrame(frame)
        prog.setPoseTool(tool)
        prog.MoveJ(home)
        tgt_items = []
        for label, pose in targets:
            t = rdk.AddTarget("P_%s_%s" % (zname, label), frame, robot)
            t.setPose(pose)
            tgt_items.append(t)
            if label == "approach":
                prog.MoveJ(t)
            else:
                prog.MoveL(t)
        prog.MoveJ(home)

        best = None
        for (sc, _, dx, yaw) in shortlist:
            rel = rel_pose(dx, dy_c, yaw)
            for t, q in zip(tgt_items, pick_poses(robot, tool_pose, rel, targets, vlist)):
                t.setPose(q)
            frame.setPoseAbs(base_abs * rel)
            ratio = prog.Update(COLLISION_ON)[3]
            if best is None or ratio > best[0]:
                best = (ratio, dx, yaw, sc)
        ratio, dx, yaw, sc = best
        rel = rel_pose(dx, dy_c, yaw)
        for t, q in zip(tgt_items, pick_poses(robot, tool_pose, rel, targets, vlist)):
            t.setPose(q)
        frame.setPoseAbs(base_abs * rel)
        ratio = prog.Update(COLLISION_ON)[3]
        result["zones"][zname] = dict(dx=dx, yaw=yaw, valid_ratio=ratio)
        print("%-12s stop dx=%5d yaw=%3d  IK-reachable %2d/%2d  valid_ratio=%.2f"
              % (zname, dx, yaw, sc, len(targets), ratio))

    with open(POSITIONS_FILE, "w") as fh:
        json.dump(result, fh, indent=2)
    print("Saved stops to", POSITIONS_FILE)
    print("Next:  python3 paint_cell_v3.py demo")


# ---------------- Demo (animated conveyor) ----------------
def glide(frame, base_abs, a, b, steps=25):
    for i in range(1, steps + 1):
        f = i / float(steps)
        st = [a[k] + (b[k] - a[k]) * f for k in range(3)]
        frame.setPoseAbs(car_state_pose(base_abs, st))
        time.sleep(0.03)


def move_car(frame, base_abs, cur, new, dy_turn):
    if cur is None:
        frame.setPoseAbs(car_state_pose(base_abs, new))
        return
    if abs(new[2] - cur[2]) > 1:        # turntable bay: far side of the conveyor
        a = (cur[0], dy_turn, cur[2])
        b = (cur[0], dy_turn, new[2])
        c = (new[0], dy_turn, new[2])
        glide(frame, base_abs, cur, a)
        glide(frame, base_abs, a, b, 40)
        glide(frame, base_abs, b, c)
        glide(frame, base_abs, c, new)
    else:
        glide(frame, base_abs, cur, new)


def run_demo():
    rdk = Robolink()
    robot, tool, frame = get_items(rdk)
    with open(POSITIONS_FILE) as fh:
        data = json.load(fh)
    dy_c = data["dy"]
    dy_turn = max(4200, dy_c + 1200)
    base_abs = robot.PoseAbs()
    order = sorted(data["zones"].items(), key=lambda kv: (kv[1]["yaw"], kv[1]["dx"]))
    cur = None
    for zname, z in order:
        new = (z["dx"], dy_c, z["yaw"])
        print("Conveyor -> stop dx=%d yaw=%d, painting %s" % (z["dx"], z["yaw"], zname))
        move_car(frame, base_abs, cur, new, dy_turn)
        cur = new
        prog = rdk.Item("Paint_" + zname, ITEM_TYPE_PROGRAM)
        if not prog.Valid():
            raise RuntimeError("Program Paint_%s not found. Run the build first." % zname)
        prog.RunProgram()
        time.sleep(0.5)
        while prog.Busy():
            time.sleep(0.2)
    print("Demo finished.")


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "demo":
        run_demo()
    else:
        build_and_fit()