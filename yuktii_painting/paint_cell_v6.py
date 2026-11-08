# ~/Documents/YUKTII AI LAB/yuktii_painting/paint_cell_v6.py
# Build + fit:  python3 paint_cell_v6.py
# Demo run:     python3 paint_cell_v6.py demo
import json
import os
import re
import sys
import time
from math import ceil, pi, radians

from robodk.robolink import (
    Robolink, ITEM_TYPE_ROBOT, ITEM_TYPE_FRAME, ITEM_TYPE_TOOL,
    ITEM_TYPE_TARGET, ITEM_TYPE_PROGRAM, COLLISION_ON,
)
from robodk.robomath import Mat, transl, rotx, roty, rotz

# ---------------- Parameters (mm / degrees) ----------------
CAR_LENGTH = 4800.0
CAR_WIDTH = 2097.0       # measured after scaling by setup_cell.py
CAR_HEIGHT = 1492.0      # measured after scaling by setup_cell.py
HOOD_Z = 1000.0          # assumed hood surface height
TRUNK_Z = 1050.0         # assumed trunk surface height
STANDOFF = 250.0
ROW_SPACING = 150.0
APPROACH = 200.0
TILT_DEG = 20.0
WAYPOINT_STEP = 250.0        # a checked waypoint every this many mm along each pass
MAX_JOINT_STEP_DEG = 35.0    # max joint change between waypoints (singularity / flip filter)
FRAME_NAME = "CarFrame"
FLIP = False                                 # gun axis: False = as defined
DY_VALUES = [2000, 2300, 2600, 3000]         # car centre distance from robot base
DX_VALUES = list(range(-3000, 3001, 500))    # car position along X
YAWS = (0, 180)                              # 180 = car turned around
PER_DY = 2                                   # best stops per distance sent to the path check
TOP_N = 5                                    # stops fully verified per zone
MAX_PRUNE = 12                               # max passes removed per zone if still failing
TURN_BAY_Y = 4200.0                          # where the car turns around (demo only)
POSITIONS_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                              "car_positions_v6.json")

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
    """Each pass is a list of waypoint poses (car-frame coordinates), zig-zag order."""
    n, s, r, f = zone["normal"], zone["stroke"], zone["row"], zone["fixed"]
    s0, s1 = zone["stroke_range"]
    z_axis = tuple(-c for c in n)
    passes = []
    rows = frange(zone["row_range"][0], zone["row_range"][1], ROW_SPACING)
    for i, rv in enumerate(rows):
        a, b = (s0, s1) if i % 2 == 0 else (s1, s0)
        p0 = [f[k] + s[k] * a + r[k] * rv + n[k] * STANDOFF for k in range(3)]
        p1 = [f[k] + s[k] * b + r[k] * rv + n[k] * STANDOFF for k in range(3)]
        d = [p1[k] - p0[k] for k in range(3)]
        length = sum(c * c for c in d) ** 0.5
        x_dir = tuple(c / length for c in d)
        steps = max(1, int(ceil(length / WAYPOINT_STEP)))
        wps = []
        for j in range(steps + 1):
            t = j / float(steps)
            origin = [p0[k] + d[k] * t for k in range(3)]
            wps.append(make_pose(origin, x_dir, z_axis))
        passes.append(wps)
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


def solve(robot, pose_base, approx, tool_pose):
    """Joint solution closest to 'approx' (list of 6 joints), or None."""
    try:
        j = robot.SolveIK(pose_base, approx, tool_pose)
    except Exception:
        return None
    if j is None:
        return None
    vals = j.list()
    if len(vals) < 6:
        return None
    return [float(x) for x in vals[:6]]


def ik_ok(robot, pose_base, tool_pose):
    try:
        j = robot.SolveIK(pose_base, None, tool_pose)
        return j is not None and len(j.list()) >= 6
    except Exception:
        return False


def quick_count(robot, tool_pose, passes, rel, vlist):
    """Fast filter: passes whose first and last waypoints are reachable."""
    n = 0
    for wps in passes:
        ok = True
        for p in (wps[0], wps[-1]):
            if not any(ik_ok(robot, rel * (p * v), tool_pose) for v in vlist):
                ok = False
                break
        if ok:
            n += 1
    return n


def smooth_pass(robot, tool_pose, rel, wps, vlist, start_joints):
    """Try each orientation; accept it if the IK path is continuous along the pass."""
    for v in vlist:
        approx, prev, ok = start_joints, None, True
        for w in wps:
            jl = solve(robot, rel * (w * v), approx, tool_pose)
            if jl is None:
                ok = False
                break
            if prev is not None and max(abs(a - b) for a, b in zip(jl, prev)) > MAX_JOINT_STEP_DEG:
                ok = False
                break
            prev, approx = jl, jl
        if ok:
            return [w * v for w in wps], prev
    return None, start_joints


def plan_smooth(robot, tool_pose, passes, rel, vlist, home_list):
    kept, joints = [], home_list
    for wps in passes:
        res, last = smooth_pass(robot, tool_pose, rel, wps, vlist, joints)
        if res is not None:
            kept.append(res)
            joints = last
    return kept


def ends_only_plan(robot, tool_pose, passes, rel, vlist):
    """Fallback (v5 behaviour): keep passes whose two ends are reachable."""
    kept = []
    for wps in passes:
        for v in vlist:
            if all(ik_ok(robot, rel * (p * v), tool_pose) for p in (wps[0], wps[-1])):
                kept.append([wps[0] * v, wps[-1] * v])
                break
    return kept


def orient(kept):
    """Each pass starts at the end nearest the previous pass's end."""
    out, prev = [], None
    for wps in kept:
        wps = list(wps)
        if prev is not None and dist2(prev, wps[-1].Pos()) < dist2(prev, wps[0].Pos()):
            wps.reverse()
        out.append(wps)
        prev = wps[-1].Pos()
    return out


def clean(msg):
    return re.sub(r"\s+", " ", re.sub(r"<[^>]+>", "", str(msg))).strip()


def build_program(rdk, robot, tool, frame, home, zname, tag, passes, use_movel):
    prog = rdk.AddProgram("Paint_%s_%s" % (zname, tag), robot)
    prog.setFrame(frame)
    prog.setPoseTool(tool)
    prog.MoveJ(home)
    items = []
    ta = rdk.AddTarget("P_%s_%s_approach" % (zname, tag), frame, robot)
    ta.setPose(passes[0][0] * transl(0, 0, -APPROACH))
    items.append(ta)
    prog.MoveJ(ta)
    for i, wps in enumerate(passes):
        for j, w in enumerate(wps):
            t = rdk.AddTarget("P_%s_%s_p%02d_w%02d" % (zname, tag, i, j), frame, robot)
            t.setPose(w)
            items.append(t)
            if use_movel:
                prog.MoveL(t)
            else:
                prog.MoveJ(t)
    tr = rdk.AddTarget("P_%s_%s_retreat" % (zname, tag), frame, robot)
    tr.setPose(passes[-1][-1] * transl(0, 0, -APPROACH))
    items.append(tr)
    prog.MoveJ(tr)
    prog.MoveJ(home)
    return prog, items


def discard(cand):
    if cand is None:
        return
    cand["prog"].Delete()
    for t in cand["items"]:
        t.Delete()


def check(frame, base_abs, robot, home_j, prog, dx, dy, yaw):
    frame.setPoseAbs(base_abs * rel_pose(dx, dy, yaw))
    robot.setJoints(home_j)
    res = prog.Update(COLLISION_ON)
    return res[3], clean(res[4])


def failing_pass(msg):
    m = re.search(r"_p(\d+)_w", msg)
    return int(m.group(1)) if m else None


# ---------------- Build and fit ----------------
def build_and_fit():
    rdk = Robolink()
    robot, tool, frame = get_items(rdk)
    robot.setPoseTool(tool)
    rdk.setCollisionActive(COLLISION_ON)
    tool_pose = tool.PoseTool()
    base_abs = robot.PoseAbs()
    home_j = robot.JointsHome()
    home_list = [float(x) for x in home_j.list()[:6]]
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
        total = len(passes)

        # 1) Quick IK filter over every stop
        rows = []
        for dy in DY_VALUES:
            for dx in DX_VALUES:
                for yaw in YAWS:
                    n = quick_count(robot, tool_pose, passes, rel_pose(dx, dy, yaw), vlist)
                    rows.append((n, dy, dx, yaw))
        print("%-12s quick IK coverage up to %d/%d passes"
              % (zname, max(r[0] for r in rows), total))

        # 2) Path-continuity check on the best stops of EVERY conveyor distance
        shortlist = []
        for dy in DY_VALUES:
            sub = sorted([r for r in rows if r[1] == dy],
                         key=lambda r: (r[0], -abs(r[2])), reverse=True)[:PER_DY]
            shortlist += sub
        cands = []
        for (n, dy, dx, yaw) in shortlist:
            if n == 0:
                continue
            kept = plan_smooth(robot, tool_pose, passes, rel_pose(dx, dy, yaw), vlist, home_list)
            if kept:
                cands.append((len(kept), dy, -abs(dx), dx, yaw, kept))
        cands.sort(key=lambda r: r[:3], reverse=True)

        if not cands:
            print("             no continuous path found; falling back to start/end-only passes")
            top = sorted(rows, key=lambda r: (r[0], r[1], -abs(r[2])), reverse=True)[:3]
            for (n, dy, dx, yaw) in top:
                kept = ends_only_plan(robot, tool_pose, passes, rel_pose(dx, dy, yaw), vlist)
                if kept:
                    cands.append((len(kept), dy, -abs(dx), dx, yaw, kept))
        if not cands:
            print("%-12s NO reachable passes from any tested stop." % zname)
            continue

        # 3) Full reach + collision verification (MoveL and MoveJ versions)
        best, done = None, False
        for (n_kept, dy, _, dx, yaw, kept) in cands[:TOP_N]:
            kept = orient(kept)
            for use_movel in (True, False):
                tag = "L" if use_movel else "J"
                prog, items = build_program(rdk, robot, tool, frame, home,
                                            zname, tag, kept, use_movel)
                ratio, msg = check(frame, base_abs, robot, home_j, prog, dx, dy, yaw)
                cand = dict(ratio=ratio, movel=use_movel, kept=n_kept, dx=dx, dy=dy, yaw=yaw,
                            msg=msg, prog=prog, items=items, passes=kept, tag=tag)
                key = (cand["ratio"], cand["movel"], cand["kept"])
                if best is None or key > (best["ratio"], best["movel"], best["kept"]):
                    discard(best)
                    best = cand
                else:
                    discard(cand)
                if use_movel and ratio >= 0.999:
                    done = True
                    break
            if done:
                break

        # 4) Remove passes that still fail, one at a time
        pruned = 0
        while (best["ratio"] < 0.999 and pruned < MAX_PRUNE and len(best["passes"]) > 1):
            idx = failing_pass(best["msg"])
            if idx is None or idx >= len(best["passes"]):
                break
            del best["passes"][idx]
            best["passes"] = orient(best["passes"])
            discard(best)
            prog, items = build_program(rdk, robot, tool, frame, home, zname,
                                        best["tag"], best["passes"], best["movel"])
            ratio, msg = check(frame, base_abs, robot, home_j, prog,
                               best["dx"], best["dy"], best["yaw"])
            best.update(prog=prog, items=items, ratio=ratio, msg=msg)
            pruned += 1

        best["prog"].setName("Paint_" + zname)
        ratio, msg = check(frame, base_abs, robot, home_j, best["prog"],
                           best["dx"], best["dy"], best["yaw"])
        coverage = 100.0 * len(best["passes"]) / total
        result["zones"][zname] = dict(
            dx=best["dx"], dy=best["dy"], yaw=best["yaw"],
            move="MoveL" if best["movel"] else "MoveJ",
            valid_ratio=ratio, passes_kept=len(best["passes"]), passes_total=total,
            coverage_pct=round(coverage, 1))
        print("%-12s dx=%5d dy=%4d yaw=%3d  %s  passes=%d/%d (%3.0f%%)  valid_ratio=%.2f%s"
              % (zname, best["dx"], best["dy"], best["yaw"],
                 "MoveL" if best["movel"] else "MoveJ",
                 len(best["passes"]), total, coverage, ratio,
                 "  (removed %d failing passes)" % pruned if pruned else ""))
        if ratio < 0.999:
            print("    reason:", msg[:300])

    with open(POSITIONS_FILE, "w") as fh:
        json.dump(result, fh, indent=2)
    print("Saved to", POSITIONS_FILE)
    print("Next:  python3 paint_cell_v6.py demo")


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