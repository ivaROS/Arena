from __future__ import annotations

import math
from typing import TYPE_CHECKING

import numpy as np

from task_generator.shared import Orientation, Pose

if TYPE_CHECKING:
    from task_generator.tasks import TaskContext


async def random_placement(ctx: TaskContext, safe_dist: float = 0.5, level_id: str = "") -> Pose:
    """Return a single random free pose on the current map for robot placement."""
    biggest_robot = max((r.safe_distance for r in ctx.robots.values()), default=safe_dist)
    points = ctx.world_manager.get_positions_on_map(n=1, safe_dist=biggest_robot, level_id=level_id)
    yaw = 2 * math.pi * float(np.random.default_rng().random())
    return Pose(points[0], orientation=Orientation.from_yaw(yaw))
