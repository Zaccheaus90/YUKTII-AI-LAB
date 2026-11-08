# ~/Documents/YUKTII AI LAB/yuktii_painting/check_fit.py
from robodk.robolink import Robolink, ITEM_TYPE_FRAME, ITEM_TYPE_TARGET
from robodk.robomath import transl

POINTS = {
    "CHK_Left_door": (0, 950, 800),
    "CHK_Right_door": (0, -950, 800),
    "CHK_Roof": (0, 0, 1600),
    "CHK_Front": (2400, 0, 600),
    "CHK_Rear": (-2400, 0, 600),
}

rdk = Robolink()
frame = rdk.Item("CarFrame", ITEM_TYPE_FRAME)
for t in rdk.ItemList(ITEM_TYPE_TARGET):
    if t.Name().startswith("CHK_"):
        t.Delete()
for name, (x, y, z) in POINTS.items():
    t = rdk.AddTarget(name, frame)
    t.setPose(transl(x, y, z))
print("Added check markers. Look at them in the 3D view.")