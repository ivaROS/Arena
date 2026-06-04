# arena_simulation_setup

Worlds, assets, environment configs, and the `Identifier` registry for Arena.
Simulators and `task_generator` resolve world layouts, 3D models, materials,
pedestrian agents, and environment templates through the types defined here.

## Guides

- [Identifier registry](src/arena_simulation_setup/tree/README.md) — `Identifier`
  ABC, resolver hierarchy, shipped Identifier types, adding a new one.
- [Worlds](worlds/README.md) — per-world directory layout, `world.yaml` schema,
  scenario schema, local asset overrides, `WorldIdentifier` resolution.
- [Environment configs](configs/environment/README.md) — what an environment
  YAML binds; obstacle group schema; `EnvironmentIdentifier` resolution.
- [Wall presets](configs/walls/README.md) — `WallDescription` YAML schema;
  sub-wall types; how `world.yaml` references a wall preset by `kind:`.
- [HuNav configs](configs/hunav/README.md) — `default.yaml` agent template;
  shared behavior tree library under `behavior_trees/`.
- [Authoring a world](AUTHORING.md) — end-to-end guide: create dir, author
  `world.yaml`, generate map, add scenario, validate.

## CLI

| Script | Usage | Effect |
|---|---|---|
| `download_assets` | `download_assets [provider] [relpath]` | Fetches assets from a named network provider via `ros2 run arena_models arena_models net <provider> fetch` |
| `generate_world` | `generate_world "<prompt>" [-e endpoint] [-o outdir]` | Posts a natural-language prompt to a generation server; extracts the returned zip into `worlds/<outdir>/` |
| `model_staging` | `model_staging <install_dir>` | Creates symlinks in `<install_dir>` for all known robot models and writes a `deps` file |
| `touch_world` | `touch_world <world_name> [--all] [--resolution N] [--assets color] ...` | Renders a preview `map.png` + `map.yaml` into the world dir for inspection; `--all` regenerates canonical per-level `world.yaml` + maps. The runtime renders its own map in-process, so this is an authoring aid, not required after editing. |

Scripts live under [scripts/](scripts).

## Internals

Key `Identifier` types from `arena_simulation_setup.tree`:

- **`WorldIdentifier`** ([tree/World/World.py](src/arena_simulation_setup/tree/World/World.py))
  — resolves a world name to `ASS_DIR / 'worlds' / <name>`; `.load()` returns
  a `World` view that exposes `world.load()` → `WorldDescription`,
  `world.map`, and `world.scenario`.
- **`ObjectIdentifier`** ([tree/assets/Object.py](src/arena_simulation_setup/tree/assets/Object.py))
  — resolves `<domain>/Object/<name>` to a `ModelWrapper` (SDF + USD).
- **`PedestrianIdentifier`** ([tree/assets/Pedestrian.py](src/arena_simulation_setup/tree/assets/Pedestrian.py))
  — resolves `<domain>/Pedestrian/<name>` to a `ModelWrapper` (SDF only).
- **`MaterialIdentifier`** ([tree/assets/Material.py](src/arena_simulation_setup/tree/assets/Material.py))
  — resolves `<domain>/Material/<name>` to a `Material`; supports `tint`
  modifier.
- **`WallIdentifier`** ([tree/Wall.py](src/arena_simulation_setup/tree/Wall.py))
  — resolves `<domain>/Wall/<name>/<name>.yaml` to a `WallDescription`;
  realizes to `(WallSegment[], Obstacle[])`.
- **`EnvironmentIdentifier`** ([tree/configs/environment.py](src/arena_simulation_setup/tree/configs/environment.py))
  — resolves `configs/environment/<name>.yaml` to an `EnvironmentDescription`
  (untyped dict).
- **`ParametrizedIdentifier`** ([tree/configs/parametrized.py](src/arena_simulation_setup/tree/configs/parametrized.py))
  — resolves an XML file in `arena_bringup/configs/parametrized/` to a
  `ParametrizedConfig` with `STATIC`, `INTERACTIVE`, and `DYNAMIC` obstacle lists.

See [tree/README.md](src/arena_simulation_setup/tree/README.md) for resolver
order, the `DynamicPaths` singleton, and how to add a new Identifier.