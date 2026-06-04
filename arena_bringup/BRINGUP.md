# Arena bringup usage

Most Arena sessions start with `arena launch`. It is a bash composite that
either:

- **brings up a fresh runtime** if none exists (sim + `arena_node` via
  `arena_runtime.launch.py`, then waits for `/arena/register_env`), or
- **attaches additively** to an already-running runtime, with a sim-mismatch
  check on `sim:=`.

Either way it then spawns `env_n` task-generator envs and, unless
`headless:=true` (or explicit `rviz:=false`), runs `arena viz --all` so each
env gets a rviz window.

For the decoupled flow (runtime stays up, envs and viz come and go), the
three underlying verbs can be used independently. See
[CLI verbs](#cli-verbs) below.

The full argument surface for `arena_runtime.launch.py` and
`task_generator.launch.py` is in [launch/README.md](launch/README.md).

---

## Three-verb model

| Verb | Launch file | What it starts |
|---|---|---|
| `arena runtime [args]` | `arena_runtime.launch.py` | Sim + `arena_node`, no envs |
| `arena env [args]` | `task_generator.launch.py` | One task-generator env; waits for `/arena/register_env` (10s warning cadence if runtime is absent) |
| `arena viz [target]` | (ros2 run) | Attaches rviz to a running env; see [arena viz](#arena-viz) |

`arena launch` orchestrates all three (skipping the runtime step if one is
already up) and is the canonical entry point for most sessions.

---

## Minimum-viable invocations

### 1. Smoke check — dummy sim, empty map

No physics engine. Useful for verifying that the ROS graph comes up without
hardware or GPU.

```bash
arena launch \
    sim:=dummy \
    world:=map_empty \
    robot:=jackal \
    tm_robots:=explore \
    tm_obstacles:=random
```

| Arg | Implication |
|---|---|
| `sim:=dummy` | No physics engine; a `map→dummy` static TF is published instead |
| `world:=map_empty` | Loads the empty map from `arena_simulation_setup` |
| `robot:=jackal` | Single jackal; `mobile` adapter defaults to `none` (dummy sim has no nav2) |
| `tm_robots:=explore` | Robot gets fresh random goals continuously |
| `tm_obstacles:=random` | Random static/dynamic obstacles placed each episode |

The `human` arg defaults to `dummy` when `sim=dummy`, so no human simulator
is started.

---

### 2. Gazebo + jackal + random obstacles

```bash
arena launch \
    sim:=gazebo \
    world:=map_empty \
    robot:=jackal \
    mobile.local_planner:=teb \
    tm_robots:=explore \
    tm_obstacles:=random
```

| Arg | Implication |
|---|---|
| `sim:=gazebo` | Starts gz-sim 8 (dart physics, ogre renderer). `human` defaults to `hunav` |
| `world:=map_empty` | Resolved to `arena_simulation_setup/worlds/map_empty/worlds/map_empty.world`; falls back to `configs/gazebo/empty.sdf` if absent |
| `mobile.local_planner:=teb` | TEB local planner; `mobile` adapter defaults to `nav2` for gazebo, the override lands as `robot.mobile.local_planner` and is forwarded to nav2's bringup |
| `headless` | Omitted → `false` (sim GUI visible, rviz shown). Pass `headless:=true` to hide the sim GUI (rviz also suppressed unless `rviz:=true` is set explicitly) |

To suppress the HuNavSim agent manager when no human obstacles are needed,
add `human:=dummy` to the command above.

---

### 3. Gazebo + jackal + HuNavSim

```bash
arena launch \
    sim:=gazebo \
    world:=map_empty \
    robot:=jackal \
    human:=hunav \
    tm_robots:=explore \
    tm_obstacles:=random
```

`human:=hunav` starts `hunav_agent_manager` in the task-generator namespace.
Human pedestrian models are managed by the HuNavSim plugin; the
`tm_obstacles` mode controls non-human obstacles separately.

---

### 4. Isaac + multi-robot via task_config

Isaac must be installed and `arena feature isaac` must be set up before launch.

```bash
arena launch \
    sim:=isaac \
    world:=map_empty \
    task_config:=$(ros2 pkg prefix arena_bringup)/share/arena_bringup/configs/tasks/default.yaml \
    tm_obstacles:=random \
    headless:=true
```

| Arg | Implication |
|---|---|
| `sim:=isaac` | Runs `arena feature isaac launch` via bash. `mobile` adapter defaults to `nav2` |
| `task_config:=<path>` | Structured `TaskModeSpec` YAML; overrides `tm_robots`. Use to split a fleet across multiple task modes |
| `headless:=true` | Sim GUI hidden; rviz suppressed (no GUI at all) |

Multi-robot fleet with two modes:

```bash
cat > /tmp/fleet.yaml << 'EOF'
task_modes:
  - kind: scenario
    produces: GOTO_POSE
    assignments: [jackal_0]
    config: {}
  - kind: explore
    produces: GOTO_POSE
    assignments: []
    config: {}
EOF

arena launch \
    sim:=isaac \
    world:=map_empty \
    robot:=jackal \
    task_config:=/tmp/fleet.yaml \
    tm_obstacles:=random \
    headless:=true
```

`jackal_0` follows a scenario; every other jackal explores. See
[configs/tasks/README.md](configs/tasks/README.md) for the full schema.

---

### 5. Multiple parallel environments

```bash
arena launch \
    sim:=gazebo \
    world:=map_empty \
    robot:=jackal \
    env_n:=3
```

| Arg | Implication |
|---|---|
| `env_n:=3` | Three task-generator instances under `arena/env_0/task_generator_node`, `arena/env_1/...`, `arena/env_2/...`. `arena_node` self-orchestrates the fleet via `/arena/spawn_env`. |

Slot positions are placed by the shelf packer in `arena_node` based on each env's `WorldExtent`; spacing is governed by the `slot_buffer` ROS parameter on `arena_node` (default 5 m).

---

### 6. Runtime / client mode (dynamic envs)

The runtime (`arena_node`) and the simulator can be launched without any
task-generator envs, then envs and viz can be added or removed at will.

```bash
# Terminal 1: runtime only.
arena runtime sim:=gazebo world:=map_empty
```

Then attach pieces from other terminals:

```bash
# Add an env. Multiple invocations stack (different robot/task each).
arena env robot:=jackal tm_robots:=explore tm_obstacles:=random

# Or use arena launch, which detects the existing runtime and attaches
# additively rather than bringing up a fresh one. Errors on sim:= mismatch.
arena launch sim:=gazebo env_n:=1 robot:=burger tm_robots:=random

# Attach rviz to an existing env (auto-pick, by id, or all).
arena viz
arena viz 0
arena viz --all
```

`arena env` and `arena viz` both wait forever (10s warning cadence) if the
runtime or env isn't up yet, so terminal ordering doesn't matter.

To tear an env down by id, call the cleanup service (or use
`arena cleanup <env_id>`, see [CLI verbs](#cli-verbs)).

---

### 7. RL training mode

```bash
arena train sim:=gazebo world:=map_empty robot:=jackal \
    train_config:=/path/to/train_config.yaml
```

Training includes `arena_runtime.launch.py` directly (runtime-only, no
auto-spawn). `train_agent.py` reads `n_envs` from the YAML and spawns envs
via `/arena/spawn_env`.

---

## headless and rviz

| Arg | Default | Meaning |
|---|---|---|
| `headless` | `false` | `true` = hide the sim GUI (server-only mode for Gazebo). Implicitly sets `rviz:=false` unless `rviz:=true` is explicit. |
| `rviz` | `true` | Controls whether `arena viz --all` is called after envs come up. Ignored when `headless:=true` unless overridden. |
| `viz.view` | `map` | Camera view in rviz: `map` (TopDownOrtho), `robot` (Orbit on robot base), `robot3p` (ThirdPersonFollower on robot base). |
| `viz.robot` | `0` | Robot index in the fleet for `viz.view:=robot*`. `all` spawns one rviz window per robot. Ignored when `view=map`. |

Examples:

```bash
# Sim GUI visible, rviz shown (default)
arena launch sim:=gazebo

# Sim GUI hidden, no rviz
arena launch sim:=gazebo headless:=true

# Sim GUI hidden, rviz shown (explicit override)
arena launch sim:=gazebo headless:=true rviz:=true

# Sim GUI visible, no rviz
arena launch sim:=gazebo rviz:=false
```

---

## arena viz

Attaches rviz to one or more running envs after launch (out-of-band).

```bash
arena viz                              # auto-pick if exactly one env is running
arena viz <env_id>                     # match by env id (last path component)
arena viz --ns <ns>                    # explicit namespace
arena viz --all                        # one rviz window per running env
arena viz view:=robot3p                # third-person follower on robot 0
arena viz 1 view:=robot robot:=-1      # orbit the last robot of env 1
arena viz --all robot:=all             # all envs, every robot, one window each
```

Any `key:=value` is forwarded straight to
[rviz_config.launch.py](../utils/rviz_utils/launch/rviz_config.launch.py). Under
`arena launch` the same tokens take a `viz.` prefix to disambiguate from
runtime args; `arena viz` accepts the prefixed form too. The launch file
declares `view` and `robot` as ROS params on the rviz_config node.

Waits forever for a matching env to appear (10s warning cadence), mirroring
`arena env`'s wait for the runtime. Once at least one env is up: a single
match with no arg auto-picks; multiple matches with no arg print the list
and exit non-zero with a hint to use `--all` or `<env_id>`.

---

## Common options

```bash
log_level:=debug     # verbose output from all nodes
use_sim_time:=false  # real-time clock (unusual, only for real robots)
complexity:=2        # AMCL (position unknown); 3 = SLAM
record_data_dir:=/tmp/arena_run  # enable data recording
```

## Cap-scoped overrides

Per-robot caps (`mobile`, `arm`, `lift`) are configured at launch via three
shapes of argument:

| Shape | Example | Lands as | Purpose |
|---|---|---|---|
| `<cap>:=<kind>` | `mobile:=rosnav_rl`, `mobile:=drl` | `robot.<cap>_adapter` | Pick which `Bringup` runs for the cap. |
| `<cap>.<key>:=<val>` | `mobile.local_planner:=teb`, `mobile.planner:=drlvo` | `robot.<cap>.<key>` | Override a value from `caps/<cap>.yaml`. |
| `<adapter-kwarg>:=<val>` | `global_planner:=smac`, `global_planner:=nav2/navfn` | `robot.<cap>.<key>` (via the adapter's launch file) | Adapter-internal launch kwargs (nav2 planner names, or the `<family>/<kind>` form consumed by the `drl` adapter). |

The cap-scoped form is the recommended style because it's self-documenting and
maps unambiguously to a single cap. To discover what `<key>`s a cap accepts,
read the robot's `arena_robots/.../robots/<name>/caps/<cap>.yaml`: every
top-level key there is overridable by the same name with the cap prefix.

Contestant args in benchmark YAMLs use the same shapes and the same forwarding
rules (see
[benchmark README](../arena_evaluation/arena_evaluation/configs/benchmark/README.md#contestant-args)).

## CLI verbs

`source arena` (from `~/arena_ws`) loads a bash function that wraps the
common entry points. Verbs relevant to bringup:

| Verb | Wraps | Purpose |
|---|---|---|
| `arena launch [args]` | bash composite | All-in-one: `arena runtime` + N × `arena env` + optional `arena viz --all`. |
| `arena runtime [args]` | `arena_runtime.launch.py` | Runtime-only launch (sim + `arena_node`, no envs). |
| `arena env [args]` | `task_generator.launch.py` | Attach one task-generator env to a running runtime. |
| `arena viz [target]` | `ros2 run rviz_utils rviz_config` | Attach rviz to a running env; see [arena viz](#arena-viz). |
| `arena cleanup <env_id>` | `/arena/cleanup_namespace` service | Force-clean an env's namespace by id (calls the service for both the `env_<id>_` and `env_<id>/` prefixes, covering gazebo and isaac layouts). |
| `arena train [args]` | `arena_training` feature launcher | RL training entry, see section 7 above. |

None of these verbs killall anything. `arena launch` checks for an existing
runtime via `/arena/register_env`: if present, it attaches additively
(spawning `env_n` more envs against the existing runtime) and errors out
only if `sim:=` on the command line mismatches the running runtime's `sim`
parameter. `arena runtime` will fail if another `/arena` node is already
registered (ROS doesn't allow duplicate node names); kill the prior one
manually or call `arena cleanup` on its envs first.

## Benchmark mode

Benchmark runs are driven by the `arena benchmark` CLI verb, which launches
`arena_evaluation/launch/benchmark.launch.py`. Configuration lives in
[arena_evaluation/configs/benchmark/](../arena_evaluation/configs/benchmark/README.md).

```bash
arena benchmark sim:=gazebo headless:=true suite:=basic contest:=basic
```

The runner groups steps by `(contestant, robot, simulator)`. One env is
spawned per group; stage transitions within the group are pushed via
`QueueEpisode` rather than a respawn. The env is despawned only between
groups (i.e. between contestants, or when the robot changes).

Total run time scales as `bringup_time × num_contestants + episode_time ×
total_episodes`, not `bringup_time × num_steps`.

`env_n` caps the number of parallel groups (parallel contestants). Results
land under `$ARENA_DATA_DIR/benchmarks/<run_id>/`. Resume an interrupted run
with `arena benchmark --resume <run_id>`.
