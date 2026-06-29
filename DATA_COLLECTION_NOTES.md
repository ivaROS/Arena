# Data Collection Notes

## Workspace State

- The workspace was corrected to use `https://github.com/ivaROS/Arena.git`.
- Current top-level branch: `jazzy`.
- Current top-level commit checked during this work: `02d9024`.
- Important submodule revision: `arena_evaluation` at `3e0e19e`, which includes
  the ROS 2 `TopicMetadata.offered_qos_profiles` fix for rosbag writing.

## What Was Observed

- The earlier smoke command used `--scale-episodes 0.1`; for one-episode stages
  this rounded to `0` planned episodes, so those runs were not useful as data
  collection tests.
- A real one-episode run using `map_empty` scenario `1.json` launched Gazebo and
  started the episode, but DWB timed out after 60 seconds.
- A simpler one-episode run using `scenario_empty` succeeded and wrote a row to
  `progress.csv`.
- GPU was not meaningfully used because these tests ran Gazebo in headless mode.
  This path is mostly CPU-bound unless running a GPU-backed simulator/rendering
  path such as Isaac or a GPU-enabled Gazebo rendering workflow.

## Fixes Made

- `arena_simulation_setup/launch/robot.launch.py`
  - Changed recorder launch arguments from one nested list to normal argv tokens:
    `--dir`, then `record_data_dir`.
  - Reason: the recorder process was starting, but the requested benchmark
    output directory was not being honored.

- `arena_evaluation/arena_evaluation/data_recorder_node.py`
  - Preserved absolute recorder paths instead of always rewriting them under the
    package share directory.
  - Reason: benchmark runs pass an absolute directory like
    `/opt/arena_ws/data/benchmarks/<run_id>/<contestant>/<stage>`, and rosbag
    output should live there.

- `arena_evaluation/benchmark/runner.py`
  - Included each step's `record_dir` in the environment grouping key.
  - Reason: the recorder opens its rosbag writer when the environment is
    spawned. Reusing one environment across multiple stages meant later stages
    could be written into the first stage's recording folder.

## Useful Commands

From the host:

```bash
cd /home/hussam-arena/arena_ws
source ./arena
```

Build only the packages needed for Gazebo/DWB benchmark collection:

```bash
colcon build --symlink-install --continue-on-error \
  --cmake-args -DPython3_ROOT_DIR=/opt/venv/bin/python3 -DBUILD_TESTING=OFF \
  --base-paths /opt/arena_ws/src \
  --build-base /opt/arena_ws/build \
  --install-base /opt/arena_ws/install \
  --packages-select arena_simulation_setup arena_evaluation arena_bringup arena_runtime arena_robots task_generator task_generator_msgs arena_rclpy_mixins arena_models arena_planners
```

Run a quick passing data-collection smoke:

```bash
ros2 run arena_evaluation benchmark sim:=gazebo headless:=true \
  --run-id smoke-empty-dwb \
  --suite '{"stages":[{"name":"scenario_empty","map":"map_empty","robot":"jackal","episodes":1,"tm_robots":"scenario","tm_obstacles":"scenario","timeout":"120s","config":{"scenario":{"file":"empty"}}}]}' \
  --contest '[{"name":"dwb","mobile.local_planner":"dwb","mobile.inter_planner":"navigate_w_replanning_time"}]'
```

Inspect outputs:

```bash
arena evaluation status smoke-empty-dwb
arena evaluation tail smoke-empty-dwb
find /opt/arena_ws/data/benchmarks/smoke-empty-dwb -maxdepth 5 -type f | sort
```

Run the built-in map-empty suite:

```bash
arena benchmark sim:=gazebo headless:=true --suite map_empty --contest basic
```

Avoid `--scale-episodes 0.1` for one-episode suites because it can round to
zero episodes.

Run a small standard Nav2 planner comparison:

```bash
ros2 run arena_evaluation benchmark sim:=gazebo headless:=true \
  --run-id planner-compare-nav2-smoke \
  --suite '{"stages":[{"name":"empty","map":"map_empty","robot":"jackal","episodes":1,"tm_robots":"scenario","tm_obstacles":"scenario","timeout":"120s","config":{"scenario":{"file":"empty"}}},{"name":"map_empty_1","map":"map_empty","robot":"jackal","episodes":1,"tm_robots":"scenario","tm_obstacles":"scenario","timeout":"120s","config":{"scenario":{"file":"1.json"}}}]}' \
  --contest '[{"name":"dwb","mobile.local_planner":"dwb","mobile.inter_planner":"navigate_w_replanning_time"},{"name":"rpp","mobile.local_planner":"regulated_pure_pursuit","mobile.inter_planner":"navigate_w_replanning_time"},{"name":"graceful","mobile.local_planner":"graceful","mobile.inter_planner":"navigate_w_replanning_time"},{"name":"mppi","mobile.local_planner":"mppi","mobile.inter_planner":"navigate_w_replanning_time"}]'
```

In the first comparison run, all four planners completed the `empty` stage and
timed out on `map_empty_1`. The non-empty scenario logs showed unresolved
scenario models such as `shelf1`, `shelf2`, and `shelf3`, which is the next
area to investigate before treating the planner scores as meaningful.
