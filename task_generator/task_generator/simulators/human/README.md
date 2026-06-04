# task_generator human simulator

`BaseHumanSimulator` manages the pedestrian lifecycle (spawn, move, remove)
and the per-episode obstacle bookkeeping layer. Implementations are registered
in `HumanSimulatorRegistry`. See also [sim interface](../sim/README.md) for the
physics-simulator counterpart (`BaseSim`).

## `BaseHumanSimulator`

[`__init__.py:19`](__init__.py#L19)

```python
class BaseHumanSimulator(NodeInterface, abc.ABC):
```

Holds a `KnownObstacles` table that tracks every spawned obstacle by name,
its `spawned` flag, and its `ObstacleLayer`. On `__init__` it subscribes to
`<namespace>/arena_peds`, caches ped positions in `_ped_positions_xy`, and
calls `self._simulator.attach_human_simulator(self)` so the mechanism shim
(`MechanismITF`, see [sim interface](../../../../arena_runtime/arena_runtime/arena_runtime/sim/README.md))
can read ground-truth ped positions and dispatch ped teleports through this
class. Public methods delegate to both the physics `BaseSim` and the
human-sim `_*_impl` methods:

| Public method | Purpose |
| --- | --- |
| `spawn_obstacles(obstacles, layer)` | spawn or move static obstacles; layer defaults to `INUSE` |
| `spawn_dynamic_obstacles(obstacles)` | spawn or move dynamic obstacles (`INUSE`) |
| `spawn_world(walls, doors)` | spawn world geometry in both sim and human-sim layers |
| `unuse_obstacles()` | call `_remove_obstacles_impl`, then flip all `INUSE` layers to `UNUSED` |
| `remove_obstacles(purge)` | remove all obstacles at or below `purge` layer from both layers; `WORLD` survives unless `purge >= WORLD` |
| `spawn_robot(robots)` | spawn in physics sim, then call `_spawn_robot_impl` |
| `remove_robot(robots)` | remove from physics sim, then call `_remove_robot_impl` |
| `move_robot(robots)` | move in physics sim, then call `_move_robot_impl` |

### `HumanSimulator` Protocol surface

`BaseHumanSimulator` satisfies the Protocol the mechanism shim reads from.
Inherited defaults work for all current subclasses; override only for
specialized teleport semantics (e.g. resetting an internal hunav agent list).

| Method | Signature |
| --- | --- |
| `pedestrian_positions_xy()` | `() -> Iterable[tuple[str, tuple[float, float]]]` (sync, reads `_ped_positions_xy`) |
| `pedestrian_teleport(destinations)` | `(Mapping[str, tuple[float, float]]) -> bool` (async, dispatches via `simulator.pedestrian_update`) |

### Abstract `_impl` methods

Every subclass must implement:

| Method | Purpose |
| --- | --- |
| `_spawn_obstacles_impl(obstacles)` | human-sim side: register or prepare static obstacles |
| `_spawn_dynamic_obstacles_impl(obstacles)` | human-sim side: register pedestrian agents |
| `_remove_obstacles_impl()` | human-sim side: signal removal of current episode's obstacles |
| `_spawn_walls_impl(walls)` | human-sim side: ingest wall geometry |
| `_spawn_doors_impl(doors)` | human-sim side: ingest door geometry |
| `_spawn_robot_impl(robots)` | human-sim side: register robots |
| `_remove_robot_impl(robots)` | human-sim side: deregister robots |
| `_move_robot_impl(robots)` | human-sim side: update robot positions |

## PROMPT registration

`TM_Prompt` is not registered centrally. Each `BaseHumanSimulator` subclass
that supports LLM-driven obstacle generation registers its own `TM_Obstacles`
variant (including the system-prompt text and response parser) via
`_register_task_modes` at class-definition time. This co-locates the prompt
with the simulator that will animate the resulting agents.

## Registered implementations

[`__init__.py:318`](__init__.py#L318) — `HumanSimulatorRegistry` maps
`Constants.HumanSimulator` keys to async factory functions:

| Key | Class | File | Notes |
| --- | --- | --- | --- |
| `dummy` | `DummyHumanSimulator` | [`dummy.py`](dummy.py) | no-op stubs; used in test/offline contexts |
| `hunav` | `HunavHumanSimulator` | [`hunav/hunav.py`](hunav/hunav.py) | integrates with the HuNavSim pedestrian simulator |
| `isaac` | `IsaacHumanSimulator` | [`isaac.py`](isaac.py) | Isaac Sim pedestrian integration |

## HuNavSim default agent template

`HunavHumanSimulator` derives pedestrian parameters from a `HunavDynamicObstacle`
instance. At module import time
([`hunav/__init__.py:326`](hunav/__init__.py#L326)), the class-level default is
loaded from:

```
arena_simulation_setup/configs/hunav/default.yaml
```

via `_load_config()`. This file sets the default behavior parameters
(`behavior`, `desired_velocity`, `radius`, `behavior_tree`, etc.) for every
pedestrian not otherwise configured. The path is resolved via
`get_package_share_directory("arena_simulation_setup")` at startup.

## Adding a new BaseHumanSimulator

1. Create `simulators/human/<name>.py` with a class extending
   `BaseHumanSimulator`; implement all eight `_*_impl` abstract methods.
2. Add `<NAME> = "<name>"` to `Constants.HumanSimulator` in
   [`constants/__init__.py`](../../constants/__init__.py).
3. Register a lazy async factory in [`simulators/human/__init__.py`](__init__.py):

```python
@HumanSimulatorRegistry.register(Constants.HumanSimulator.MY_SIM)
async def _my_sim(**kwargs):
    from .my_sim import MyHumanSimulator
    return await MyHumanSimulator.create(**kwargs)
```

If the implementation supports LLM-driven obstacle generation, call
`_register_task_modes` in the class body to register a `TM_Obstacles`
subclass (with prompt text) under `Constants.TaskMode.TM_Obstacles.PROMPT`.
