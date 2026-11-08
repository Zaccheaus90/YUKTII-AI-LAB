# ~/Documents/YUKTII AI LAB/yuktii_painting/diag_approach.py
# Run with RoboDK open:  python3 diag_approach.py
from robodk.robolink import COLLISION_ON, Robolink
from robodk.robomath import transl

from paint_cell_v8 import APPROACH, FLIP, ZONES, get_items, rel_pose, variants, zone_passes

# (zone, dx, dy, yaw): one working Right case, then Left cases
CASES = [
    ("Right_Front", 0, 3300, 0),
    ("Left_Front", 0, 3300, 180),
    ("Left_Front", 0, 2300, 180),
    ("Left_Front", 0, -2300, 0),
    ("Left_Front", 0, -3300, 0),
]
STEPS = 40


def describe(entries):
    out = []
    for e in entries:
        try:
            if isinstance(e, (list, tuple)):
                out.append(" / ".join(x.Name() if hasattr(x, "Name") else str(x) for x in e))
            else:
                out.append(e.Name() if hasattr(e, "Name") else str(e))
        except Exception as exc:
            out.append("<%s>" % exc)
    return out


def main():
    rdk = Robolink()
    robot, tool, frame = get_items(rdk)
    robot.setPoseTool(tool)
    rdk.setCollisionActive(COLLISION_ON)
    tool_pose = tool.PoseTool()
    base_abs = robot.PoseAbs()
    home_list = [float(x) for x in robot.JointsHome().list()[:6]]
    vlist = variants(FLIP)
    print("Home joints:", [round(v, 1) for v in home_list])
    print()

    for zone, dx, dy, yaw in CASES:
        print("CASE %s  car dx=%d dy=%d yaw=%d" % (zone, dx, dy, yaw))
        rel = rel_pose(dx, dy, yaw)
        frame.setPoseAbs(base_abs * rel)
        first = zone_passes(ZONES[zone])[0][0] * vlist[0]
        approach = first * transl(0, 0, -APPROACH)

        try:
            sol = robot.SolveIK(rel * approach, home_list, tool_pose)
            target = [float(x) for x in sol.list()[:6]] if sol is not None else []
        except Exception as exc:
            print("    IK error:", exc)
            continue
        if len(target) < 6:
            print("    approach pose not reachable (no IK solution)")
            continue
        print("    approach joints:", [round(v, 1) for v in target])

        hit = None
        for i in range(STEPS + 1):
            f = i / float(STEPS)
            joints = [home_list[k] + (target[k] - home_list[k]) * f for k in range(6)]
            robot.setJoints(joints)
            if rdk.Collisions() > 0:
                hit = (f, joints, describe(rdk.CollisionItems()))
                break
        if hit is None:
            print("    no collision on the home -> approach move")
        else:
            print("    FIRST COLLISION at %.0f%% of the move" % (hit[0] * 100))
            print("    joints there:", [round(v, 1) for v in hit[1]])
            print("    touching:", hit[2])
        print()

    robot.setJoints(home_list)


if __name__ == "__main__":
    main()