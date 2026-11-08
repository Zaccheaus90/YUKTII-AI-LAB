# list_items.py
from robodk.robolink import Robolink

rdk = Robolink()
items = rdk.ItemList()
print("Total items in station:", len(items))
for it in items:
    print("%-30s type=%s" % (it.Name(), it.Type()))