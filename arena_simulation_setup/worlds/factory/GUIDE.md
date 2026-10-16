# Build and test a big Arena room in Gazebo

This is a complete starting world. Follow the steps in order to edit room
geometry, compare wall thicknesses, place boxes, generate a 2D preview, and test
robot and pedestrian motion.

## 1. Install it in the active workspace

The world must be inside the same Arena workspace used by `arena launch`. For
the standard `/opt/arena_ws` workspace, unzip it with:

```bash
cd /opt/arena_ws/src/Arena/arena_simulation_setup/worlds
unzip /path/to/student_room_tutorial.zip
```

The resulting structure must include:

```text
student_room_tutorial/
├── GUIDE.md
├── SCENARIO_GUIDE.md
├── 0/world.yaml
├── scenarios/
│   ├── README.md
│   ├── default/scenario.json
│   └── student_skeleton/scenario.json
├── assets/Common/Wall/
│   ├── thin_wall/thin_wall.yaml
│   ├── standard_wall/standard_wall.yaml
│   └── thick_wall/thick_wall.yaml
├── assets/Common/Object/tutorial_box/
│   ├── tutorial_box.sdf
│   └── annotation.yaml
├── support/gazebo_wall_dimensions.patch
├── map.png
└── map.yaml
```

Rebuild and source that workspace:

```bash
export ARENA_WS=/opt/arena_ws
export ARENA_DIR="$ARENA_WS/src/Arena"
cd "$ARENA_WS"

# This Arena version originally forced all Gazebo walls to 5 cm. Install the
# included compatibility patch once if the active checkout does not have it.
if ! grep -q 'thickness=segment.width' \
  "$ARENA_DIR/arena_runtime/arena_runtime/arena_runtime/sim/gazebo_simulator/gazebo_simulator.py"; then
  git -C "$ARENA_DIR" apply \
    "$ARENA_DIR/arena_simulation_setup/worlds/student_room_tutorial/support/gazebo_wall_dimensions.patch"
fi

colcon build --symlink-install \
  --packages-select arena_simulation_setup arena_runtime

source install/setup.bash
```

Confirm the installed package contains the world:

```bash
test -f "$(ros2 pkg prefix arena_simulation_setup)/share/arena_simulation_setup/worlds/student_room_tutorial/0/world.yaml" \
  && echo "student_room_tutorial is installed"
```

Do not continue until the confirmation is printed.

## 2. Edit the room

Open `student_room_tutorial/0/world.yaml`. It is valid YAML with notes beside
every section. Its main entries are:

- `zones`: named navigable rooms or corridors;
- `corners`: ordered floor-polygon coordinates in metres;
- `walls`: explicit collision segments joining those corners;
- `kind`: a wall preset controlling height, width, and material;
- `doors` and `elevators`: empty extension points in this exercise;
- `entities.static`: permanent boxes belonging to the room.

The supplied room is 20 m by 15 m. When changing a corner, also change the
connected wall endpoints. The polygon and wall geometry are separate.

## 3. Change wall thickness

Each wall selects one local preset:

| Wall kind | Width |
|---|---:|
| `thin_wall` | 0.05 m |
| `standard_wall` | 0.15 m |
| `thick_wall` | 0.40 m |

Presets live at `assets/Common/Wall/<kind>/<kind>.yaml`. Change a wall's `kind`
in `world.yaml`, or edit a preset's `width` and `height`. The 2D map represents
wall centre lines; compare exact wall thickness in Gazebo.

## 4. Place permanent boxes

The included `tutorial_box` is a 1 m cube. A permanent entry looks like:

```yaml
- name: box_south_west
  model: tutorial_box
  pose: [4.0, 3.0, 0.0]
  bbox: [[-0.5, 0.5], [-0.5, 0.5], [0.0, 1.0]]
```

- `name` must be unique;
- `pose` is `[x, y, yaw]` in metres and radians;
- `bbox` lets the preview renderer draw the footprint;
- the local SDF controls the actual Gazebo geometry.

Keep a 1 m box's centre more than 0.5 m from walls, then add navigation
clearance. Copy an entry and change its name and pose to add another box.

## 5. Regenerate and inspect permanent geometry

Run this after every `world.yaml` geometry or box change:

```bash
ros2 run arena_simulation_setup touch_world student_room_tutorial \
  --assets red \
  --assets-label blue
```

Open:

```text
/opt/arena_ws/src/Arena/arena_simulation_setup/worlds/
student_room_tutorial/map.png
```

White is navigable space, black is the boundary, red rectangles are permanent
box footprints, and blue text contains entity names. If generation fails,
check YAML indentation, missing colons, wall kind names, and duplicate entities.

This PNG checks permanent geometry only. It does not show scenario robots or
pedestrians; inspect those in Gazebo/RViz.

## 6. Run the completed example

Explicitly select the supplied `default` scenario:

```bash
arena launch \
  sim:=gazebo \
  world:=student_room_tutorial \
  robot:=jackal \
  human:=hunav \
  tm_robots:=scenario \
  tm_obstacles:=scenario \
  scenario_file:=default
```

Check wall connections and thicknesses, box positions, pedestrian motion, and
whether the Jackal completes all three goals without collisions.

Stop and relaunch after an edit. A running environment does not automatically
reload the world or scenario.

## 7. Create a student scenario

Scenario files in this tutorial are JSON. Read `SCENARIO_GUIDE.md`, then find
the editable starting file here:

```text
student_room_tutorial/scenarios/student_skeleton/scenario.json
```

To preserve the clean skeleton, copy its directory:

```bash
cd "$ARENA_DIR/arena_simulation_setup/worlds/student_room_tutorial/scenarios"
cp -r student_skeleton my_test
```

The scenario name is now `my_test`, and its required file is
`my_test/scenario.json`. Edit it, validate it, and launch it:

```bash
python3 -m json.tool my_test/scenario.json >/dev/null \
  && echo "scenario JSON is valid"

cd "$ARENA_WS"
arena launch \
  sim:=gazebo \
  world:=student_room_tutorial \
  robot:=jackal \
  human:=hunav \
  tm_robots:=scenario \
  tm_obstacles:=scenario \
  scenario_file:=my_test
```

`SCENARIO_GUIDE.md` explains every JSON section, temporary objects, moving and
stationary people, ordered robot goals, multiple robots, selection, validation,
and visualization.

## 8. Completion exercise

1. Resize the room to 24 m by 18 m and update every wall endpoint.
2. Choose different thicknesses for opposite walls.
3. Move the three boxes and add a fourth with a unique name.
4. Regenerate `map.png` and verify all permanent footprints.
5. Copy `student_skeleton` to a new scenario name.
6. Create a robot route containing at least three `goto` phases.
7. Add one crossing and one stationary pedestrian.
8. Validate the JSON and select it with `scenario_file:=...`.
9. Test in Gazebo and correct collisions or unreachable goals.

Completing these steps covers the full cycle: design, preview, simulate, test,
and revise.

## Troubleshooting: world not found

If an error mentions an installed path ending in
`worlds/student_room_tutorial`, the folder is not installed in the currently
sourced workspace. Check its source location, rebuild `arena_simulation_setup`,
source `install/setup.bash`, and repeat the installation test from step 1.
