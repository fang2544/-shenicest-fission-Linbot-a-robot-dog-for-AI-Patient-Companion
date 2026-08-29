"""在医院二维占据栅格上执行确定性的 A* 路径规划。"""

import heapq
import math
from typing import Dict, List, Optional, Tuple

from hospital_escort_mvp.voxel_mapping import Costmap2D


Cell = Tuple[int, int]


class PlanningError(RuntimeError):
    pass


def _nearest_traversable(costmap: Costmap2D, target: Cell, allow_unknown: bool) -> Cell:
    def valid(cell: Cell) -> bool:
        return costmap.contains(*cell) and (
            costmap.value(*cell) == 0 or (allow_unknown and costmap.value(*cell) == -1)
        )

    if valid(target):
        return target
    for radius in range(1, max(costmap.width, costmap.height)):
        candidates = []
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                if max(abs(dx), abs(dy)) != radius:
                    continue
                cell = (target[0] + dx, target[1] + dy)
                if valid(cell):
                    candidates.append(cell)
        if candidates:
            return min(candidates, key=lambda c: math.hypot(c[0] - target[0], c[1] - target[1]))
    raise PlanningError(f'no traversable cell near {target}')


def plan_path(
    costmap: Costmap2D,
    start_world: Tuple[float, float],
    goal_world: Tuple[float, float],
    allow_unknown: bool = False,
) -> List[Tuple[float, float]]:
    """规划八邻域路径，并禁止沿障碍物拐角斜穿。"""
    raw_start = costmap.world_to_cell(*start_world)
    raw_goal = costmap.world_to_cell(*goal_world)
    if not costmap.contains(*raw_start) or not costmap.contains(*raw_goal):
        raise PlanningError('start or goal is outside the observed map')
    start = _nearest_traversable(costmap, raw_start, allow_unknown)
    goal = _nearest_traversable(costmap, raw_goal, allow_unknown)
    moves = ((1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
             (1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)),
             (-1, 1, math.sqrt(2)), (-1, -1, math.sqrt(2)))

    frontier = [(0.0, start)]
    came_from: Dict[Cell, Optional[Cell]] = {start: None}
    cost_so_far = {start: 0.0}

    def traversable(cell: Cell) -> bool:
        if not costmap.contains(*cell):
            return False
        value = costmap.value(*cell)
        return value == 0 or (allow_unknown and value == -1)

    while frontier:
        _, current = heapq.heappop(frontier)
        if current == goal:
            break
        for dx, dy, step_cost in moves:
            nxt = (current[0] + dx, current[1] + dy)
            if not traversable(nxt):
                continue
            if dx and dy and (
                not traversable((current[0] + dx, current[1]))
                or not traversable((current[0], current[1] + dy))
            ):
                continue
            unknown_penalty = 3.0 if costmap.value(*nxt) == -1 else 1.0
            new_cost = cost_so_far[current] + step_cost * unknown_penalty
            if nxt not in cost_so_far or new_cost < cost_so_far[nxt]:
                cost_so_far[nxt] = new_cost
                priority = new_cost + math.hypot(goal[0] - nxt[0], goal[1] - nxt[1])
                heapq.heappush(frontier, (priority, nxt))
                came_from[nxt] = current

    if goal not in came_from:
        raise PlanningError('no collision-free path to goal')
    cells = []
    cursor: Optional[Cell] = goal
    while cursor is not None:
        cells.append(cursor)
        cursor = came_from[cursor]
    cells.reverse()
    return [costmap.cell_to_world(*cell) for cell in cells]
