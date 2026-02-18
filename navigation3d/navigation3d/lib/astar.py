import math
import heapq
from typing import Dict, List, Optional, Set, Tuple
from navigation3d.lib.octomap_reader import OctomapReader

GridIdx = Tuple[int, int, int]

class AStar:
    def __init__(self, map_reader: OctomapReader, bbox_margin: float):
        self.map = map_reader
        self.bbox_margin = bbox_margin
        self.neighbor_steps = [
            (dx, dy, dz)
            for dx in (-1, 0, 1)
            for dy in (-1, 0, 1)
            for dz in (-1, 0, 1)
            if not (dx == 0 and dy == 0 and dz == 0)
        ]

    def euclid(self, a: GridIdx, b: GridIdx) -> float:
        return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2 + (a[2]-b[2])**2)

    def in_bounds(self, n: GridIdx, bounds: Tuple[int, int, int, int, int, int]) -> bool:
        return (bounds[0] <= n[0] <= bounds[1] and
                bounds[2] <= n[1] <= bounds[3] and
                bounds[4] <= n[2] <= bounds[5])

    def reconstruct_path(self, came_from: Dict[GridIdx, GridIdx], cur: GridIdx) -> List[GridIdx]:
        path = [cur]
        while cur in came_from:
            cur = came_from[cur]
            path.append(cur)
        path.reverse()
        return path

    def solve_a_star(self, start_world: List[float], goal_world: List[float]) -> Optional[List[GridIdx]]:

        start_idx = self.map.world_to_grid(start_world)
        goal_idx = self.map.world_to_grid(goal_world)

        if self.map.is_occupied_astar(start_idx) or self.map.is_occupied_astar(goal_idx):
            return None

        margins_to_try = [self.bbox_margin, self.bbox_margin * 4.0, 50.0]

        for i, margin in enumerate(margins_to_try):
            bounds = self._calculate_bounds(start_idx, goal_idx, margin)

            path = self._compute_astar(start_idx, goal_idx, bounds)

            if path:
                if i > 0:
                    pass
                return path

        return None

    def _calculate_bounds(self, start: GridIdx, goal: GridIdx, margin_meters: float) -> Tuple[int, int, int, int, int, int]:
        if margin_meters == float('inf'):
            min_val = -1000000
            max_val = 1000000
            return (min_val, max_val, min_val, max_val, min_val, max_val)

        m = int(math.ceil(margin_meters / self.map.grid_res))

        minx = min(start[0], goal[0]) - m
        maxx = max(start[0], goal[0]) + m
        miny = min(start[1], goal[1]) - m
        maxy = max(start[1], goal[1]) + m
        minz = min(start[2], goal[2]) - m
        maxz = max(start[2], goal[2]) + m

        return (minx, maxx, miny, maxy, minz, maxz)

    def _compute_astar(self, start_idx: GridIdx, goal_idx: GridIdx, bounds: Tuple[int, int, int, int, int, int]) -> Optional[List[GridIdx]]:

        if not self.in_bounds(start_idx, bounds) or not self.in_bounds(goal_idx, bounds):
            return None

        open_heap: List[Tuple[float, float, GridIdx]] = []
        heapq.heappush(open_heap, (self.euclid(start_idx, goal_idx), 0.0, start_idx))

        came_from: Dict[GridIdx, GridIdx] = {}
        gscore: Dict[GridIdx, float] = {start_idx: 0.0}

        while open_heap:
            f, g, cur = heapq.heappop(open_heap)

            if cur == goal_idx:
                return self.reconstruct_path(came_from, cur)

            if g > gscore.get(cur, float('inf')):
                continue

            for dx, dy, dz in self.neighbor_steps:
                nb = (cur[0] + dx, cur[1] + dy, cur[2] + dz)

                if not self.in_bounds(nb, bounds):
                    continue

                if self.map.is_occupied_astar(nb):
                    continue

                step_cost = math.sqrt(dx*dx + dy*dy + dz*dz)
                ng = g + step_cost

                if ng < gscore.get(nb, float("inf")):
                    came_from[nb] = cur
                    gscore[nb] = ng
                    nf = ng + self.euclid(nb, goal_idx)
                    heapq.heappush(open_heap, (nf, ng, nb))

        return None