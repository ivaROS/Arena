# Scenario guide and annotated JSON draft

## Where a scenario belongs

Every scenario is a directory under this world's `scenarios/` folder:

```text
student_room_tutorial/scenarios/<scenario_name>/scenario.json
```

The supplied choices are:

- `default/scenario.json`: complete robot and pedestrian demonstration;
- `student_skeleton/scenario.json`: minimal file intended for the student to
  copy or edit.

To create a test called `box_test`, copy the skeleton directory and keep the
required filename:

```bash
cd "$ARENA_DIR/arena_simulation_setup/worlds/student_room_tutorial/scenarios"
cp -r student_skeleton box_test
```

The new file is `scenarios/box_test/scenario.json`. Select it during launch
with `scenario_file:=box_test`. The argument is the directory name, not the
filename and not a full path.

## JSON rules

Unlike `world.yaml`, scenario JSON cannot contain comments. Keep explanations
in this guide. JSON requires double quotes, commas between entries, no trailing
comma, and matching `{}` and `[]` characters.

Validate a scenario before launch:

```bash
python3 -m json.tool \
  "$ARENA_DIR/arena_simulation_setup/worlds/student_room_tutorial/scenarios/box_test/scenario.json" \
  >/dev/null && echo "scenario JSON is valid"
```

## Annotated structure

The following is explanatory JSON-like text. Do not paste its comments into a
real JSON file; edit the supplied `student_skeleton/scenario.json` instead.

```text
{
  "static": [
    // Temporary objects used only by this scenario.
    {
      "name": "temporary_box_1",      // unique entity name
      "model": "tutorial_box",        // Object asset identifier
      "pose": [6.0, 11.0, 0.0]        // [x metres, y metres, yaw radians]
    }
  ],

  "dynamic": [
    // Pedestrians controlled by HuNav when human:=hunav.
    {
      "name": "pedestrian_1",         // unique pedestrian name
      "model": "arenian",             // pedestrian asset
      "pose": [10.0, 2.0, 1.5708],    // initial [x, y, yaw]
      "velocity": 1.0,                 // walking speed in m/s
      "desired_velocity": 1.0,         // HuNav desired speed
      "waypoints": [                   // positions visited in order
        {"x": 10.0, "y": 13.0, "z": 0.0},
        {"x": 10.0, "y": 2.0, "z": 0.0}
      ]
    }
  ],

  "robots": [
    // Supply one entry per launched robot, in the same order.
    {
      "start": [2.0, 2.0, 0.0],       // initial [x, y, yaw]
      "phases": [                     // tasks completed in order
        {"goto": [17.0, 2.0, 0.0]},
        {"goto": [17.0, 12.0, 1.5708]}
      ]
    }
  ],

  "regions": {}                       // optional advanced crowd regions
}
```

## Stationary and moving pedestrians

A moving pedestrian should have a positive velocity and at least two useful
waypoints. Keep every waypoint inside the room and away from box footprints.

A stationary pedestrian uses zero velocity and one waypoint equal to its pose:

```json
{
  "name": "stationary_pedestrian",
  "model": "arenian",
  "pose": [14.0, 7.5, 0.0],
  "velocity": 0.0,
  "desired_velocity": 0.0,
  "waypoints": [
    {"x": 14.0, "y": 7.5, "z": 0.0}
  ]
}
```

## One robot versus multiple robots

For one robot, supply one `robots` entry and launch `robot:=jackal`. For two,
add a second entry and launch `robot:=jackal,jackal`. If the counts differ,
Arena only matches the available entries and prints a warning.

## How to see a scenario

`touch_world` previews permanent world geometry; it does not draw scenario
robots or pedestrians. Inspect scenario motion in Gazebo/RViz:

```bash
arena launch \
  sim:=gazebo \
  world:=student_room_tutorial \
  robot:=jackal \
  human:=hunav \
  tm_robots:=scenario \
  tm_obstacles:=scenario \
  scenario_file:=student_skeleton
```

Use `scenario_file:=default` for the completed example. Stop and relaunch after
editing so Arena loads the scenario again.
