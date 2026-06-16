[![Discord](https://img.shields.io/badge/Discord-Join%20chat-5865F2?logo=discord&logoColor=white)](https://discord.gg/GNTTf9DKyp)

# Arena-Rosnav

A modular ROS 2 (Jazzy) platform for researching and benchmarking autonomous robot
navigation in 2D and 3D simulated environments. It supports:

- **Simulators** — Gazebo, Isaac Sim (and a physics-free `dummy` for ROS-graph testing).
- **Classical planners** — Nav2 (DWB, TEB, MPPI, …).
- **Trainable deep-RL** — [rosnav_rl](https://github.com/Arena-Rosnav/rosnav-rl) (PPO/SAC/TD3, DreamerV3).
- **External research planners** — DRL-VO, CrowdNav, SICNav, … via the [arena_planners](arena_planners/README.md) bridge.
- **Crowds** — HuNavSim pedestrians; plus **benchmarking** and **evaluation** tooling.

Everything runs inside a Docker image and is driven by a single `arena` CLI.

---

## Table of contents

- [Installation](#installation)
- [Quick start](#quick-start)
- [Running a simulation](#running-a-simulation)
- [Planners](#planners)
- [Training (rosnav_rl)](#training-rosnav_rl)
- [Benchmarking](#benchmarking)
- [Evaluation](#evaluation)
- [CLI reference](#cli-reference)
- [Development](#development)
- [Troubleshooting](#troubleshooting)

---

## Installation

**Prerequisites:** [Docker](https://docs.docker.com/engine/install/) with the
[nvidia-container-toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)
for GPU support. Your user must be in the `docker` group.

### 1. Base install

```sh
curl https://raw.githubusercontent.com/ivaROS/Arena/jazzy/install.sh > install.sh
bash install.sh
```

Follow the prompts (yellow text). This clones the workspace (default `~/arena_ws`),
builds the Docker image, and prints how to proceed. The installer creates an `arena`
entry point at the workspace root — **every command below assumes you have run
`source arena` from the workspace directory.**

```sh
cd ~/arena_ws        # your workspace path
source arena         # loads the `arena` CLI and starts the container
```

### 2. Optional features

Install at least one simulator. Features are opt-in and pulled on demand:

```sh
arena feature gazebo install      # Gazebo simulator
arena feature isaac install       # Isaac Sim simulator (heavier; needs more VRAM)
arena feature training install    # rosnav_rl DRL training stack
arena feature planners install    # external DRL planner bridge (DRL-VO, CrowdNav, SICNav)
arena feature evaluation install  # benchmarking / metrics tooling
arena feature vllm install        # optional local LLM backend (for prompt-based tasks)

arena feature robots add jackal   # pull a robot's assets/deps before first use
```

The CLI handles dependencies and `colcon build`; rebuild only when prompted.

> **Persist your image after updates.** After installing features or pulling new
> code, run `arena feature docker commit` to bake the current container state into
> the image — otherwise the changes are re-applied on every fresh container.

<details>
<summary><b>vllm</b> (local LLM backend) details</summary>

Runs a local vLLM server plus a LiteLLM proxy that speaks the Gemini API, so GPT
consumers in `task_generator` transparently hit local inference. Defaults target an
11 GB 2080 Ti (Qwen3-0.6B, 40% GPU util). Tune via
[`_meta/docker/features/vllm/config.yaml`](_meta/docker/features/vllm/config.yaml)
(`model`, `gpu_memory_utilization`, `max_model_len`, `port`/`proxy_port`), then
`arena feature vllm update`. Stop it to free VRAM with `arena feature docker stop`.
</details>

---

## Quick start

```sh
cd ~/arena_ws && source arena

# Classical Nav2 in Gazebo, random goals + obstacles, no GUI
arena launch sim:=gazebo world:=map_empty robot:=jackal \
    tm_robots:=explore tm_obstacles:=random headless:=true

# Visualize a running sim (attach rviz)
arena viz --all
```

`arena launch` brings up the whole stack (simulator + `arena_node` runtime + one or
more task-generator environments). It is **additive**: launching again attaches more
environments to the running runtime rather than restarting it.

---

## Running a simulation

A launch is composed from cap-scoped arguments. The most useful ones:

| Arg | Values | Meaning |
|---|---|---|
| `sim:=` | `gazebo`, `isaac`, `dummy` | Simulator backend (`dummy` = no physics, ROS graph only). |
| `world:=` | `map_empty`, `hospital_1`, … | World/map from `arena_simulation_setup`. |
| `robot:=` | `jackal`, `turtlebot4`, … | Robot model. Run `arena feature robots add <name>` once before first use. |
| `mobile:=` | `nav2`, `rosnav_rl`, `drl`, `manual` | Which navigation stack drives the robot (see [Planners](#planners)). |
| `tm_robots:=` | `explore`, `random`, `scenario` | Robot task mode — `explore` = continuous random goals. |
| `tm_obstacles:=` | `random`, `scenario`, … | Obstacle / pedestrian placement mode. |
| `human:=` | `hunav`, `dummy` | Pedestrian simulator (HuNavSim) or none. |
| `headless:=` | `true`/`false` | `true` = server-only sim, no GUI/rviz. |
| `env_n:=` | integer | Number of parallel environments (throughput). |

Examples:

```sh
# Gazebo + jackal + Nav2 (TEB local planner) + a pedestrian crowd
arena launch sim:=gazebo world:=map_empty robot:=jackal \
    mobile:=nav2 mobile.local_planner:=teb \
    tm_robots:=explore tm_obstacles:=random human:=hunav

# Isaac Sim
arena launch sim:=isaac world:=map_empty robot:=jackal tm_robots:=explore

# Three parallel environments
arena launch sim:=gazebo world:=map_empty robot:=jackal env_n:=3
```

To discover the keys a cap accepts, read
`arena_robots/.../robots/<name>/caps/<cap>.yaml` — every top-level key is overridable
as `mobile.<key>:=<val>`. Full argument reference: [arena_bringup/BRINGUP.md](arena_bringup/BRINGUP.md).

---

## Planners

Arena exposes three kinds of navigation stack via `mobile:=`:

### 1. Classical — `mobile:=nav2`

The Nav2 stack. Pick the local planner with `mobile.local_planner:=` (`dwb`, `teb`,
`mppi`, `rotation_shim`, `graceful`, `regulated_pure_pursuit`).

```sh
arena launch sim:=gazebo robot:=jackal mobile:=nav2 mobile.local_planner:=teb
```

### 2. Trainable deep-RL — `mobile:=rosnav_rl`

Runs an agent trained with Arena's own RL pipeline (see [Training](#training-rosnav_rl)):

```sh
arena launch sim:=gazebo robot:=jackal mobile:=rosnav_rl mobile.agent:=<agent_name>
```

The agent folder lives in `arena_training/agents/<agent_name>/` and must contain
`training_config.yaml` + `best_model.zip`.

### 3. External research planners — `mobile:=drl`

The [arena_planners](arena_planners/README.md) bridge runs research planners that each
live in their own isolated Python venv (own deps/weights), talking to Arena over a
ZeroMQ/msgpack `step(features) -> [v, omega]` contract. **These are integrated for
running and benchmarking — they ship pre-trained weights or are model-based; they are
not trained inside Arena.** Available:

| Planner | Type | Notes |
|---|---|---|
| `drlvo` | DRL (pre-trained) | DRL-VO; weights fetched from Hugging Face. |
| `crowdnav` | DRL (pre-trained) | CrowdNav / SARL. |
| `sicnav` | MPC (model-based) | SICNav bilevel-MPC crowd navigation; CasADi/IPOPT (no acados). |

```sh
# one-time per planner: fetch its venv + weights
arena feature planners add drlvo
arena feature planners ls                 # [x] ready, [ ] pending

# run it (global plan from Nav2's navfn; crowd from HuNavSim)
arena launch sim:=gazebo world:=map_empty robot:=jackal \
    mobile:=drl mobile.planner:=drlvo mobile.global_planner:=nav2/navfn \
    tm_robots:=explore tm_obstacles:=random human:=hunav
```

Swap `mobile.planner:=sicnav` / `crowdnav` for the others. Set
`mobile.global_planner:=none` to disable the global plan. To add your own planner,
implement the `step()` contract — see
[arena_planners/planners/README.md](arena_planners/planners/README.md).

**SICNav + HSL/MA57 (optional, faster multi-human MPC).** SICNav runs out of the
box on IPOPT's default MUMPS solver (limited to ~3 humans). For real crowds
(5 humans) install the HSL/MA57 linear solver — it's free for academics but
licensed per user, so it can't be shipped with the repo. Each collaborator obtains
their own licence and runs the provided installer; full step-by-step instructions
(licence application → download → build → wire-in) are in
[arena_planners/planners/sicnav/README.md](arena_planners/planners/sicnav/README.md#installing-hslma57).

---

## Training (rosnav_rl)

Arena's training pipeline trains **rosnav_rl** agents. (The external `mobile:=drl`
planners above are *not* trained within Arena — to train a custom learned planner,
configure rosnav_rl.) Install the stack first with `arena feature training install`.

```sh
# Config resolved from arena_training/configs/ (or pass an absolute path)
arena train sim:=gazebo mobile:=rosnav_rl train_config:=sb_training_config.yaml

# Model-based RL
arena train sim:=gazebo mobile:=rosnav_rl train_config:=dreamer_training_config.yaml
```

- `arena train` brings up the runtime and the trainer together. The number of parallel
  training environments comes from `arena_cfg.general.n_envs` in the config YAML.
- Watch it live with `arena viz --all`.
- Metrics log to **Weights & Biases** automatically.
- Trained agents are saved to `arena_training/agents/<agent_name>/`
  (`training_config.yaml` + `best_model.zip`), then runnable with
  `mobile:=rosnav_rl mobile.agent:=<agent_name>`.

| Config (`arena_training/configs/`) | Framework |
|---|---|
| `sb_training_config.yaml` | Stable-Baselines3 — PPO / SAC / TD3 (default) |
| `dreamer_training_config.yaml` | DreamerV3 (model-based) |

Agent architecture, observation space, reward, and curriculum are configured in the
training YAML / [rosnav_rl](arena_training/deps/rosnav_rl). Full details:
[arena_training/README.md](arena_training/README.md).

---

## Benchmarking

Run reproducible, multi-contestant evaluation suites with `arena benchmark`. A suite
defines episodes/stages; contestants are planner / robot / simulator combinations.

```sh
arena benchmark sim:=gazebo headless:=true suite:=basic contest:=basic
```

- The runner groups steps by `(contestant, robot, simulator)`, spawns one env per
  group, and pushes stage transitions via `QueueEpisode` (no respawn within a group).
- `env_n:=` caps the number of parallel contestants.
- Results land under `$ARENA_DATA_DIR/benchmarks/<run_id>/`.
- Resume an interrupted run: `arena benchmark --resume <run_id>`.

Contestant args use the same `mobile:=…` / `mobile.<key>:=…` shapes as launch, so you
can pit any planners against each other (e.g. `nav2`+teb vs `drl`+sicnav vs a trained
`rosnav_rl` agent). Suites/contestants are configured in
[arena_evaluation/.../configs/benchmark/](arena_evaluation/arena_evaluation/configs/benchmark/README.md).

---

## Evaluation

Per-episode metrics (success, collisions, time, path length, …) are recorded by
[arena_evaluation](arena_evaluation/README.md) under `$ARENA_DATA_DIR`. Use
`arena evaluation` for post-hoc analysis/plots of recorded runs; see the
[arena_evaluation README](arena_evaluation/README.md).

---

## CLI reference

`source arena` loads a bash wrapper. Common verbs:

| Verb | Purpose |
|---|---|
| `arena launch [args]` | All-in-one: runtime + N environments (+ optional rviz). |
| `arena runtime [args]` | Runtime only (sim + `arena_node`, no environments). |
| `arena env [args]` | Attach one task-generator environment to a running runtime. |
| `arena train [args]` | RL training entry point. |
| `arena benchmark [args]` | Run an evaluation suite. |
| `arena viz [target]` | Attach rviz to a running env (`arena viz --all`). |
| `arena feature <f> <add\|install\|ls\|update>` | Manage optional features (`gazebo`, `isaac`, `training`, `planners`, `robots`, `vllm`). |
| `arena cleanup <env_id>` | Force-clean an environment's namespace. |
| `arena build` / `arena deps` | Rebuild / resync workspace dependencies. |

No verb kills anything implicitly; `arena launch` attaches additively to an existing
runtime. Full bringup/launch semantics: [arena_bringup/BRINGUP.md](arena_bringup/BRINGUP.md).

---

## Development

### Linting

Linting is handled by [Ruff](https://docs.astral.sh/ruff/) via
[pre-commit](https://pre-commit.com/). Config: root [`pyproject.toml`](pyproject.toml);
hook pin: [`.pre-commit-config.yaml`](.pre-commit-config.yaml).

```bash
pip install pre-commit && pre-commit install   # one-time
pre-commit run -a                              # run on the whole repo
ruff check .                                   # check without pre-commit
```

If a hook auto-fixes something the commit aborts with the fixes left unstaged —
`git add` and re-commit. [`.github/workflows/lint.yml`](.github/workflows/lint.yml)
runs the same hooks on `jazzy` and PRs.

---

## Troubleshooting

### Unknown runtime specified 'nvidia'

```sh
sudo nvidia-ctk runtime configure --runtime=docker     && sudo systemctl restart docker
sudo nvidia-ctk runtime configure --runtime=containerd && sudo systemctl restart containerd
```

### rviz fails to open / crashes on launch

On hosts with incompatible/missing GPU drivers, force software rendering by adding to
`.env` at the workspace root (expect lower framerates):

```sh
LIBGL_ALWAYS_SOFTWARE=1
```

### A planner won't build / a robot won't spawn

Most optional pieces are pulled on demand: `arena feature planners add <name>` for a
DRL planner, `arena feature robots add <name>` before launching a robot the first
time. `arena feature planners ls` / `arena feature robots ls` show what's ready.
