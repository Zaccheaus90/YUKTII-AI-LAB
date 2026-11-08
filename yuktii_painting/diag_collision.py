# ~/Documents/YUKTII AI LAB/yuktii_painting/diag_collision.py
# Run with RoboDK open:  python3 diag_collision.py
from math import radians

from robodk.robolink import (
    Robolink, ITEM_TYPE_ROBOT, ITEM_TYPE_FRAME, COLLISION_ON,
)
from robodk.robomath import transl, rotz

FRAME_NAME = "CarFrame"
# (dx, dy, yaw): same distances with the car unturned (0) and turned (180); the last is a far baseline
STOPS = [(0, 6000, 0), (0, 2000, 0), (0, 2000, 180), (0, 2300, 0), (0, 2300, 180),
         (0, 3000, 0), (0, 3000, 180)]


def describe(entries):
    out = []
    for e in entries:
        try:
            if isinstance(e, (list, tuple)):
                parts = []
                for x in e:
                    parts.append(x.Name() if hasattr(x, "Name") else str(x))
                out.append(" / ".join(parts))
            else:
                out.append(e.Name() if hasattr(e, "Name") else str(e))
        except Exception as exc:
            out.append("<%s>" % exc)
    return out


def main():
    rdk = Robolink()
    robot = rdk.ItemList(ITEM_TYPE_ROBOT)[0]
    frame = rdk.Item(FRAME_NAME, ITEM_TYPE_FRAME)
    if not frame.Valid():
        raise RuntimeError("Frame '%s' not found." % FRAME_NAME)
    rdk.setCollisionActive(COLLISION_ON)
    base_abs = robot.PoseAbs()
    home = robot.JointsHome()
    print("Home joints (deg):", [round(float(v), 1) for v in home.list()[:6]])
    print()
    for dx, dy, yaw in STOPS:
        frame.setPoseAbs(base_abs * transl(dx, dy, 0) * rotz(radians(yaw)))
        robot.setJoints(home)
        n = rdk.Collisions()
        line = "car dx=%5d dy=%5d yaw=%3d -> collision pairs at home: %s" % (dx, dy, yaw, n)
        print(line)
        if n:
            try:
                print("    colliding items:", describe(rdk.CollisionItems()))
            except Exception as exc:
                print("    could not list colliding items:", exc)


if __name__ == "__main__":
    main()
    