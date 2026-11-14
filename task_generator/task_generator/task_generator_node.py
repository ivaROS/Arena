#! /usr/bin/env python3
from .node import TaskGenerator


def _enable_stack_dumps() -> None:
    """`kill -USR1 <pid>` appends all thread stacks to ~/.ros/log/tg_stacks_<pid>.txt.

    Lets a hung task generator be diagnosed from inside the container, where py-spy
    is unavailable (no SYS_PTRACE).
    """
    import faulthandler
    import os
    import signal

    try:
        log_dir = os.path.join(os.environ.get("ROS_LOG_DIR") or os.path.expanduser("~/.ros/log"))
        os.makedirs(log_dir, exist_ok=True)
        # Kept open for the process lifetime; faulthandler writes to the raw fd.
        f = open(os.path.join(log_dir, f"tg_stacks_{os.getpid()}.txt"), "a")  # noqa: SIM115
        faulthandler.register(signal.SIGUSR1, file=f, all_threads=True, chain=False)
    except Exception:  # never let diagnostics break startup
        pass


def _enable_tracemalloc() -> None:
    """Opt-in allocation tracing (ARENA_TRACEMALLOC=1; frames via ARENA_TRACEMALLOC_FRAMES).

    First `kill -USR2 <pid>` stores a baseline snapshot; each later one appends the top
    allocation growth since that baseline (grouped by traceback) to
    ~/.ros/log/tg_tracemalloc_<pid>.txt. Off by default: tracing costs CPU and memory.
    """
    import os

    if os.environ.get("ARENA_TRACEMALLOC") != "1":
        return
    import datetime
    import signal
    import tracemalloc

    frames = int(os.environ.get("ARENA_TRACEMALLOC_FRAMES", "6"))
    tracemalloc.start(frames)
    log_dir = os.environ.get("ROS_LOG_DIR") or os.path.expanduser("~/.ros/log")
    out = os.path.join(log_dir, f"tg_tracemalloc_{os.getpid()}.txt")
    state: dict[str, object] = {}
    ignore = (tracemalloc.Filter(False, tracemalloc.__file__), tracemalloc.Filter(False, "<frozen importlib._bootstrap>"))

    def _on_usr2(signum: int, frame: object) -> None:
        del signum, frame
        snap = tracemalloc.take_snapshot().filter_traces(ignore)
        with open(out, "a") as fh:
            now = datetime.datetime.now().isoformat(timespec="seconds")
            if "base" not in state:
                state["base"] = snap
                cur, peak = tracemalloc.get_traced_memory()
                fh.write(f"=== {now} baseline stored (traced {cur / 1e6:.1f} MB, peak {peak / 1e6:.1f} MB)\n")
                return
            diff = snap.compare_to(state["base"], "traceback")
            total = sum(d.size_diff for d in diff)
            fh.write(f"=== {now} growth since baseline: {total / 1e6:+.2f} MB over {len(diff)} tracebacks\n")
            for d in sorted(diff, key=lambda d: d.size_diff, reverse=True)[:25]:
                if d.size_diff <= 0:
                    break
                fh.write(f"\n{d.size_diff / 1024:+10.1f} KiB  {d.count_diff:+8d} blocks  (now {d.size / 1024:.1f} KiB)\n")
                for line in d.traceback.format(limit=frames):
                    fh.write(f"    {line}\n")
            fh.write("\n")

    signal.signal(signal.SIGUSR2, _on_usr2)


def main(args: list[str] | None = None) -> None:
    del args
    _enable_stack_dumps()
    _enable_tracemalloc()
    TaskGenerator.run_main(aiomonitor=True)


if __name__ == '__main__':
    import time

    time.sleep(5)
    main()
