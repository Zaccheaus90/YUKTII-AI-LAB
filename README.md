# Robotic Car Painting Cell Simulation

Practical assessment for **Yuktii AI Labs** (Robotics Simulation Intern).

- **Name:** Afolabi Zaccheaus D.
- **Software used:** RoboDK (Python API), run on Ubuntu Linux
- **Robot selected:** Comau Smart5 NJ 130-2.6 (6-axis industrial arm) with a Generic Paint Sprayer tool

## What this project does

A single industrial robot paints the exterior surfaces of a car body in a simulated cell. The car is scaled to about 4,800 mm long (4,800 x 2,097 x 1,492 mm after uniform scaling of the model), stands on a conveyor deck, and is indexed to different stops so the robot can reach each surface. The robot follows raster (zig-zag) paint paths at a constant 250 mm stand-off with the gun tilted at most 20 degrees from the surface normal.

## Repository contents

| File | Purpose |
|---|---|
| `setup_cell.py` | Measures the imported car model, scales it, orients it, and places it in `CarFrame` (centred, wheels at z = 0). Adds `CHK_` marker targets so the placement can be checked visually. |
| `recenter_car.py` | Verifies the car is centred inside `CarFrame`. |
| `add_conveyor.py` | Adds the conveyor deck object to the station. |
| `paint_cell_v11.py` | Main program: generates raster paths per zone, searches car stops, checks reach and collisions, builds the robot programs, and runs the demo. |
| `Simulation_Files/` | RoboDK station file (`.rdk`) and the saved stop positions (`car_positions_v11.json`). |

Earlier versions of the path generator (`paint_cell_v2.py` to `paint_cell_v10.py`) and the diagnostic scripts (`diag_collision.py`, `diag_approach.py`) are kept to show how the approach developed.

## Approach

1. **Cell and car.** The car model is measured, scaled uniformly to 4,800 mm length, and placed in a reference frame (`CarFrame`) with X forward, Y left and Z up.
2. **Surface zones.** The body is split into Left Front, Left Rear, Right Front, Right Rear, Roof, Hood and Trunk.
3. **Path planning.** Each zone gets a zig-zag raster path (150 mm row spacing) with a waypoint every 250 mm, a 250 mm stand-off, and an approach point 200 mm off the surface. The gun axis points into the surface and may tilt up to 20 degrees if the perpendicular pose is out of reach.
4. **Car stops.** For each zone the program searches car positions on the conveyor deck (five lanes, stops every 500 mm along it). Candidates are screened by inverse kinematics, by a collision test with the robot at home, and by a joint-continuity check along each pass (maximum 60 degrees between waypoints).
5. **Collision checking.** The surviving candidates are built as RoboDK programs and verified with RoboDK's reach and collision check. Passes that still fail are removed one at a time and the program is re-checked.
6. **Demo.** The car enters on the conveyor, moves to each stop (including sideways lane changes on the deck), the robot paints, and the car leaves.

## Results

Results from the final run (`paint_cell_v11.py`, saved in `car_positions_v11.json`):

| Zone | Car stop (position along conveyor, lane) | Gun roll | Passes painted | Moves | Reach/collision check |
|---|---|---|---|---|---|
| Right Front | 0 mm, lane 3300 mm | 0 deg | 5 of 5 (100%) | MoveJ | passed |
| Right Rear | 0 mm, lane 3300 mm | 0 deg | 4 of 5 (80%) | MoveJ | passed |
| Roof | 0 mm, lane 2000 mm | 0 deg | 6 of 9 (67%) | MoveJ | passed |
| Hood | -1500 mm, lane 2300 mm | 180 deg | 5 of 9 (56%) | MoveJ | passed |
| Trunk | 1500 mm, lane 2000 mm | 0 deg | 8 of 9 (89%) | MoveJ | passed |
| Left Front | no usable stop found | n/a | 0 of 5 | n/a | not reachable |
| Left Rear | no usable stop found | n/a | 0 of 5 | n/a | not reachable |

Overall, 28 of 47 raster passes (about 60%) are painted. Within the five zones that have a working stop, 28 of 37 passes (about 76%) are painted. "Passed" means RoboDK reported no reach or collision problems for the object pairs it checks. The lane is the distance from the robot base to the car's centre line; the conveyor deck is wide enough for the car to be shifted sideways between stops.

## Limitations

- **Left side not painted.** The robot stands on one side of the conveyor, so the car's far side is out of reach. A turntable or a second robot on the other side of the conveyor would be needed, and neither is modelled.
- **Partial coverage.** Passes that were out of reach, discontinuous, or in collision were dropped, so most zones are only partly covered.
- **Joint moves.** All painted zones use joint moves (MoveJ) between waypoints, so the gun does not travel in perfectly straight lines. Waypoints are 250 mm apart to limit the deviation.
- **Gun roll.** The Hood program rotates the gun 180 degrees about its own axis. This has not been checked against a real gun's hose and fan orientation.
- **Car size.** The car was scaled uniformly to 4,800 mm length, so its width (2,097 mm) and height (1,492 mm) differ from the 1,900 mm and 1,600 mm targets by about +10% and -7%.
- **Conveyor.** The conveyor is a visual and kinematic model. The car is moved between stops by changing its pose in the simulation, and no drive system is modelled.
- **No paint physics.** Paint thickness, spray pattern and overspray are not simulated.

## How to run

Requirements: RoboDK, Python 3 and the RoboDK Python package.

```
pip install robodk
```

On Linux, RoboDK may need its bundled libraries on the library path:

```
export LD_LIBRARY_PATH=$HOME/RoboDK/bin/lib:$LD_LIBRARY_PATH
```

1. Open RoboDK with the station file from `Simulation_Files/`.
2. `python3 add_conveyor.py` (adds the conveyor deck)
3. `python3 paint_cell_v11.py` (searches stops and builds the programs)
4. `python3 paint_cell_v11.py demo` (plays the full sequence)

The station must contain the robot, the `Generic Paint Sprayer` tool, a reference frame named `CarFrame` and the car object `CAN_CAR_BLEND_V12`.

## Demo video and report

The technical report, screenshots and demo video are in the Google Drive folder submitted with this assessment.
