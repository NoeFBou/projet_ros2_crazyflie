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

    def in_bounds(self, n: GridIdx, minx, maxx, miny, maxy, minz, maxz) -> bool:
        return (minx <= n[0] <= maxx and miny <= n[1] <= maxy and minz <= n[2] <= maxz)

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

        if self.map.is_occupied(start_idx) or self.map.is_occupied(goal_idx):
            return None
        m = int(math.ceil(self.bbox_margin / self.map.grid_res))

        minx = min(start_idx[0], goal_idx[0]) - m
        maxx = max(start_idx[0], goal_idx[0]) + m
        miny = min(start_idx[1], goal_idx[1]) - m
        maxy = max(start_idx[1], goal_idx[1]) + m
        minz = min(start_idx[2], goal_idx[2]) - m
        maxz = max(start_idx[2], goal_idx[2]) + m


        if not self.in_bounds(start_idx,minx,maxx,miny,maxy,minz,maxz) or not self.in_bounds(goal_idx,minx,maxx,miny,maxy,minz,maxz):
            #self.get_logger().warn("Start/goal out of bounded region.")
            return None
        # if self.map.is_occupied(start_idx) or self.map.is_occupied(goal_idx):
        #     #self.get_logger().warn("Start or goal is in collision (occupied after inflation).")
        #     return None
        # 26 neighbor
        # neighbor_steps = [(dx, dy, dz)
        #                   for dx in (-1, 0, 1)
        #                   for dy in (-1, 0, 1)
        #                   for dz in (-1, 0, 1)
        #                   if not (dx == 0 and dy == 0 and dz == 0)]
        open_heap: List[Tuple[float, float, GridIdx]] = []
        heapq.heappush(open_heap, (self.euclid(start_idx, goal_idx), 0.0, start_idx))

        came_from: Dict[GridIdx, GridIdx] = {}
        gscore: Dict[GridIdx, float] = {start_idx: 0.0}
        visited = 0

        while open_heap:
            f, g, cur = heapq.heappop(open_heap)
            visited += 1
            if cur == goal_idx:
                #self.get_logger().info(f"Path found. Visited={visited}")
                return self.reconstruct_path(came_from, cur)

            for dx, dy, dz in self.neighbor_steps:
                nb = (cur[0] + dx, cur[1] + dy, cur[2] + dz)
                if not self.in_bounds(nb,minx,maxx,miny,maxy,minz,maxz) or self.map.is_occupied(nb) :
                    continue

                step_cost = math.sqrt(dx*dx + dy*dy + dz*dz)
                ng = g + step_cost
                if ng < gscore.get(nb, float("inf")):
                    came_from[nb] = cur
                    gscore[nb] = ng
                    nf = ng + self.euclid(nb, goal_idx)
                    heapq.heappush(open_heap, (nf, ng, nb))

        return None