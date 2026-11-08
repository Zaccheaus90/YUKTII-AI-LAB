# paint_cell_v2.py
# Build + fit:  python3 paint_cell_v2.py
# Demo run:     python3 paint_cell_v2.py demo
import json
import os
import sys
import time
from math import cos, sin, radians

from robodk.robolink import (
    Robolink, ITEM_TYPE_ROBOT, ITEM_TYPE_FRAME, ITEM_TYPE_TOOL,
    ITEM_TYPE_TARGET, ITEM_TYPE_PROGRAM, COLLISION_ON,
    INSTRUCTION_CALL_PROGRAM,
)
from robodk.robomath import Mat, transl, rotz

# ---------------- Parameters (mm) ----------------
CAR_LENGTH = 4800.0
CAR_WIDTH = 1900.0
CAR_HEIGHT = 1600.0
STANDOFF = 250.0
ROW_SPACING = 150.0
APPROACH = 200.0
FRAME_NAME = "CarFrame"
FOOTPRINT_MARGIN = 700.0   # keep the car this far from the robot base
POSITIONS_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                              "car_positions.json")

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
    """Return list of (label, pose_in_car_frame) incl. approach, in order."""
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


def inside_footprint(dx, dy, yaw_deg):
    """True if the robot base (origin of base frame) is too close to the car."""
    yaw = radians(yaw_deg)
    xr = cos(yaw) * (-dx) + sin(yaw) * (-dy)
    yr = -sin(yaw) * (-dx) + cos(yaw) * (-dy)
    return abs(xr) < HALF_L + FOOTPRINT_MARGIN and abs(yr) < HALF_W + FOOTPRINT_MARGIN


def candidate_offsets():
    cands = []
    dys = [1400, 1800, 2200, 2600, 3000]
    dys = dys + [-d for d in dys]
    for yaw in (0, 90, 180, 270):
        for dx in range(-3200, 3201, 400):
            for dy in dys:
                if not inside_footprint(dx, dy, yaw):
                    cands.append((dx, dy, yaw))
    return cands


def car_abs_pose(base_abs, dx, dy, yaw_deg):
    return base_abs * transl(dx, dy, 0) * rotz(radians(yaw_deg))


def reachable(robot, pose_base, tool_pose):
    try:
        j = robot.SolveIK(pose_base, None, tool_pose)
        return j is not None and len(j.list()) >= 6
    except Exception:
        return False


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


def build_and_fit():
    rdk = Robolink()
    robot, tool, frame = get_items(rdk)
    robot.setPoseTool(tool)
    rdk.setCollisionActive(COLLISION_ON)
    tool_pose = tool.PoseTool()
    base_abs = robot.PoseAbs()

    for p in rdk.ItemList(ITEM_TYPE_PROGRAM):
        if p.Name().startswith("Paint_"):
            p.Delete()
    for t in rdk.ItemList(ITEM_TYPE_TARGET):
        if t.Name().startswith("P_"):
            t.Delete()

    home = rdk.AddTarget("P_Home", frame, robot)
    home.setAsJointTarget()
    home.setJoints(robot.JointsHome())

    cands = candidate_offsets()
    print("Candidate car positions per zone: %d" % len(cands))
    positions = {}
    programs = []

    for zname, zone in ZONES.items():
        targets = zone_targets(zone)

        # 1) Fast reachability search with inverse kinematics only
        scored = []
        for (dx, dy, yaw) in cands:
            rel = transl(dx, dy, 0) * rotz(radians(yaw))
            score = sum(1 for _, p in targets if reachable(robot, rel * p, tool_pose))
            scored.append((score, -(abs(dx) + abs(dy)), dx, dy, yaw))
        scored.sort(reverse=True)
        best_score = scored[0][0]
        shortlist = [c for c in scored if c[0] == best_score][:5]

        # 2) Build the program
        prog = rdk.AddProgram("Paint_" + zname, robot)
        prog.setFrame(frame)
        prog.setPoseTool(tool)
        prog.MoveJ(home)
        for label, pose in targets:
            t = rdk.AddTarget("P_%s_%s" % (zname, label), frame, robot)
            t.setPose(pose)
            if label == "approach":
                prog.MoveJ(t)
            else:
                prog.MoveL(t)
        prog.MoveJ(home)

        # 3) Verify shortlist with full check (reach + collisions), keep best
        best = None
        for (_, _, dx, dy, yaw) in shortlist:
            frame.setPoseAbs(car_abs_pose(base_abs, dx, dy, yaw))
            ratio = prog.Update(COLLISION_ON)[3]
            if best is None or ratio > best[0]:
                best = (ratio, dx, dy, yaw)
        ratio, dx, dy, yaw = best
        frame.setPoseAbs(car_abs_pose(base_abs, dx, dy, yaw))
        prog.Update(COLLISION_ON)
        positions[zname] = dict(dx=dx, dy=dy, yaw=yaw, valid_ratio=ratio)
        print("%-12s IK-reachable %2d/%2d  car offset dx=%5d dy=%5d yaw=%3d  valid_ratio=%.2f"
              % (zname, best_score, len(targets), dx, dy, yaw, ratio))
        programs.append(prog)

    with open(POSITIONS_FILE, "w") as fh:
        json.dump(positions, fh, indent=2)
    print("Saved car positions to", POSITIONS_FILE)
    print("Zone programs created. Run:  python3 paint_cell_v2.py demo")


def run_demo():
    rdk = Robolink()
    robot, tool, frame = get_items(rdk)
    with open(POSITIONS_FILE) as fh:
        positions = json.load(fh)
    base_abs = robot.PoseAbs()
    for zname in ZONES:
        pos = positions[zname]
        prog = rdk.Item("Paint_" + zname, ITEM_TYPE_PROGRAM)
        if not prog.Valid():
            raise RuntimeError("Program Paint_%s not found. Run build first." % zname)
        frame.setPoseAbs(car_abs_pose(base_abs, pos["dx"], pos["dy"], pos["yaw"]))
        print("Painting", zname)
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