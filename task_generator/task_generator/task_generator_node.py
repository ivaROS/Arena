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


def main(args: list[str] | None = None) -> None:
    del args
    _enable_stack_dumps()
    TaskGenerator.run_main(aiomonitor=True)


if __name__ == '__main__':
    import time

    time.sleep(5)
    main()
