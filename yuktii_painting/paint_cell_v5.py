# ~/Documents/YUKTII AI LAB/yuktii_painting/paint_cell_v5.py
# Build + fit:  python3 paint_cell_v5.py
# Demo run:     python3 paint_cell_v5.py demo
import json
import os
import re
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
CAR_WIDTH = 2097.0       # measured after scaling by setup_cell.py
CAR_HEIGHT = 1492.0      # measured after scaling by setup_cell.py
HOOD_Z = 1000.0          # assumed hood surface height; adjust after checking the CHK_Hood marker
TRUNK_Z = 1050.0         # assumed trunk surface height; adjust after checking the CHK_Trunk marker
STANDOFF = 250.0
ROW_SPACING = 150.0
APPROACH = 200.0
TILT_DEG = 20.0
FRAME_NAME = "CarFrame"
FLIP = False                                 # gun axis: False = as defined
DY_VALUES = [2000, 2300, 2600, 3000]         # car centre distance from robot base
DX_VALUES = list(range(-3000, 3001, 500))    # car position along X
YAWS = (0, 180)                              # 180 = car turned around
TOP_N = 6                                    # candidates fully verified per zone
TURN_BAY_Y = 4200.0                          # where the car turns around (demo only)
POSITIONS_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                              "car_positions_v5.json")

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
                        fixed=(0, 0, HOOD_Z), stroke_range=(1000, 2000),
                        row_range=(-HALF_W + 400, HALF_W - 400)),
    "Trunk":       dict(normal=(0, 0, 1),  stroke=(1, 0, 0), row=(0, 1, 0),
                        fixed=(0, 0, TRUNK_Z), stroke_range=(-2000, -1100),
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


def zone_passes(zone):
    """List of (start_pose, end_pose) per raster pass, in car-frame coordinates."""
    n, s, r, f = zone["normal"], zone["stroke"], zone["row"], zone["fixed"]
    s0, s1 = zone["stroke_range"]
    z_axis = tuple(-c for c in n)
    passes = []
    rows = frange(zone["row_range"][0], zone["row_range"][1], ROW_SPACING)
    for i, rv in enumerate(rows):
        a, b = (s0, s1) if i % 2 == 0 else (s1, s0)
        pts = [[f[k] + s[k] * sv + r[k] * rv + n[k] * STANDOFF for k in range(3)]
               for sv in (a, b)]
        x_dir = [pts[1][k] - pts[0][k] for k in range(3)]
        length = sum(c * c for c in x_dir) ** 0.5
        x_dir = tuple(c / length for c in x_dir)
        passes.append((make_pose(pts[0], x_dir, z_axis),
                       make_pose(pts[1], x_dir, z_axis)))
    return passes


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


def dist2(a, b):
    return sum((a[k] - b[k]) ** 2 for k in range(3))


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


def pick(robot, tool_pose, rel, p, vlist):
    """First reachable orientation variant for pose p; returns (pose, reachable)."""
    for v in vlist:
        q = p * v
        if ik_ok(robot, rel * q, tool_pose):
            return q, True
    return p * vlist[0], False


def plan_candidate(robot, tool_pose, passes, rel, vlist):
    """Keep only passes whose start and end are both reachable."""
    kept = []
    for ps, pe in passes:
        qs, ok1 = pick(robot, tool_pose, rel, ps, vlist)
        if not ok1:
            continue
        qe, ok2 = pick(robot, tool_pose, rel, pe, vlist)
        if not ok2:
            continue
        kept.append((qs, qe))
    return kept


def orient(kept):
    """Zig-zag the kept passes so each starts at the end nearest the previous one."""
    out, prev = [], None
    for qs, qe in kept:
        if prev is not None and dist2(prev, qe.Pos()) < dist2(prev, qs.Pos()):
            qs, qe = qe, qs
        out.append((qs, qe))
        prev = qe.Pos()
    return out


def clean(msg):
    return re.sub(r"\s+", " ", re.sub(r"<[^>]+>", "", str(msg))).strip()


def build_program(rdk, robot, tool, frame, home, zname, tag, kept, use_movel):
    prog = rdk.AddProgram("Paint_%s_%s" % (zname, tag), robot)
    prog.setFrame(frame)
    prog.setPoseTool(tool)
    prog.MoveJ(home)
    items = []
    for i, (qs, qe) in enumerate(kept):
        if i == 0:
            ta = rdk.AddTarget("P_%s_%s_approach" % (zname, tag), frame, robot)
            ta.setPose(qs * transl(0, 0, -APPROACH))
            items.append(ta)
            prog.MoveJ(ta)
        for label, pose in (("%02dS" % i, qs), ("%02dE" % i, qe)):
            t = rdk.AddTarget("P_%s_%s_%s" % (zname, tag, label), frame, robot)
            t.setPose(pose)
            items.append(t)
            if use_movel:
                prog.MoveL(t)
            else:
                prog.MoveJ(t)
    prog.MoveJ(home)
    return prog, items


def discard(cand):
    if cand is None:
        return
    cand["prog"].Delete()
    for t in cand["items"]:
        t.Delete()


# ---------------- Build and fit ----------------
def build_and_fit():
    rdk = Robolink()
    robot, tool, frame = get_items(rdk)
    robot.setPoseTool(tool)
    rdk.setCollisionActive(COLLISION_ON)
    tool_pose = tool.PoseTool()
    base_abs = robot.PoseAbs()
    home_j = robot.JointsHome()
    vlist = variants(FLIP)
    print("Car size used: %.0f x %.0f x %.0f mm" % (CAR_LENGTH, CAR_WIDTH, CAR_HEIGHT))
    print("Tool Z axis:", [round(v, 2) for v in tool_pose.VZ()],
          "| gun orientation:", "FLIPPED" if FLIP else "AS DEFINED")

    for p in rdk.ItemList(ITEM_TYPE_PROGRAM):
        if p.Name().startswith("Paint_"):
            p.Delete()
    for t in rdk.ItemList(ITEM_TYPE_TARGET):
        if t.Name().startswith("P_"):
            t.Delete()

    home = rdk.AddTarget("P_Home", frame, robot)
    home.setAsJointTarget()
    home.setJoints(home_j)

    result = {"zones": {}}
    for zname, zone in ZONES.items():
        passes = zone_passes(zone)

        # 1) Fast IK search: how many passes can each car position reach?
        rows = []
        for dy in DY_VALUES:
            for dx in DX_VALUES:
                for yaw in YAWS:
                    kept = plan_candidate(robot, tool_pose, passes,
                                          rel_pose(dx, dy, yaw), vlist)
                    rows.append((len(kept), -dy, -abs(dx), dx, dy, yaw, kept))
        rows.sort(key=lambda r: r[:3], reverse=True)
        print("%-12s best IK coverage %d/%d passes" % (zname, rows[0][0], len(passes)))

        # 2) Verify the best positions with the full reach + collision check
        best, done = None, False
        for (n_kept, _, _, dx, dy, yaw, kept) in rows[:TOP_N]:
            if n_kept == 0:
                break
            kept = orient(kept)
            frame.setPoseAbs(base_abs * rel_pose(dx, dy, yaw))
            for use_movel in (True, False):
                tag = "L" if use_movel else "J"
                prog, items = build_program(rdk, robot, tool, frame, home,
                                            zname, tag, kept, use_movel)
                robot.setJoints(home_j)
                res = prog.Update(COLLISION_ON)
                cand = dict(ratio=res[3], movel=use_movel, kept=n_kept, dx=dx, dy=dy,
                            yaw=yaw, msg=clean(res[4]), prog=prog, items=items)
                key = (cand["ratio"], cand["movel"], cand["kept"])
                if best is None or key > (best["ratio"], best["movel"], best["kept"]):
                    discard(best)
                    best = cand
                else:
                    discard(cand)
                if use_movel and cand["ratio"] >= 0.999:
                    done = True
                    break
            if done:
                break

        if best is None:
            print("%-12s NO reachable passes from any tested car position." % zname)
            continue

        frame.setPoseAbs(base_abs * rel_pose(best["dx"], best["dy"], best["yaw"]))
        robot.setJoints(home_j)
        res = best["prog"].Update(COLLISION_ON)
        ratio, msg = res[3], clean(res[4])
        best["prog"].setName("Paint_" + zname)
        coverage = 100.0 * best["kept"] / len(passes)
        result["zones"][zname] = dict(
            dx=best["dx"], dy=best["dy"], yaw=best["yaw"],
            move="MoveL" if best["movel"] else "MoveJ",
            valid_ratio=ratio, coverage_pct=round(coverage, 1))
        print("%-12s dx=%5d dy=%4d yaw=%3d  %s  coverage=%3.0f%%  valid_ratio=%.2f"
              % (zname, best["dx"], best["dy"], best["yaw"],
                 "MoveL" if best["movel"] else "MoveJ", coverage, ratio))
        if ratio < 0.999:
            print("    reason:", msg[:300])

    with open(POSITIONS_FILE, "w") as fh:
        json.dump(result, fh, indent=2)
    print("Saved to", POSITIONS_FILE)
    print("Next:  python3 paint_cell_v5.py demo")


# ---------------- Demo ----------------
def glide(frame, base_abs, a, b, steps=25):
    for i in range(1, steps + 1):
        f = i / float(steps)
        st = [a[k] + (b[k] - a[k]) * f for k in range(3)]
        frame.setPoseAbs(car_state_pose(base_abs, st))
        time.sleep(0.03)


def move_car(frame, base_abs, cur, new):
    if cur is None:
        frame.setPoseAbs(car_state_pose(base_abs, new))
        return
    if abs(new[2] - cur[2]) > 1:        # turn the car around far from the robot
        a = (cur[0], TURN_BAY_Y, cur[2])
        b = (cur[0], TURN_BAY_Y, new[2])
        c = (new[0], TURN_BAY_Y, new[2])
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
    base_abs = robot.PoseAbs()
    order = sorted(data["zones"].items(), key=lambda kv: (kv[1]["yaw"], kv[1]["dx"]))
    cur = None
    for zname, z in order:
        new = (z["dx"], z["dy"], z["yaw"])
        print("Moving car to dx=%d dy=%d yaw=%d, painting %s" % (new + (zname,)))
        move_car(frame, base_abs, cur, new)
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