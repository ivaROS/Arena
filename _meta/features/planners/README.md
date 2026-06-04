# arena feature planners

CLI for managing planner submodules of the `arena_planners` SDK. Each planner is its own GitHub submodule plus a per-planner venv. This feature handles checkout, weight fetch, and removal.

## Commands

```sh
arena feature planners ls                    # list available planners (registered + locally-present)
arena feature planners check                 # init/uninit status for each
arena feature planners add <name> [<name>…]  # init submodule(s) + fetch HF weights
arena feature planners add --all             # init every registered planner
arena feature planners rm <name>             # deinit submodule (keeps the gitmodules entry)
arena feature planners update                # git submodule update --recursive within the SDK
arena feature planners uninstall             # deinit every initialized planner
```

## What `add` does

1. `git submodule update --init --checkout arena_planners` (ensures SDK submodule is present).
2. `git submodule update --init --checkout planners/<name>` inside the SDK.
3. If `planners/<name>/weights.yaml` exists: `hf_hub_download` each entry from the listed HF repo, symlink into `planners/<name>/<dest>`. See [SDK planners/README.md](../../../arena_planners/planners/README.md#weightsyaml) for the manifest schema.

## Requirements

`huggingface_hub` and `pyyaml` must be available in the Arena venv. Weight fetch fails loudly if either is missing.
