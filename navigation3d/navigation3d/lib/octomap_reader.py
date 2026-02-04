import math
from typing import Set, Tuple, List
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

GridIdx = Tuple[int, int, int]

class OctomapReader:

    def __init__(self,grid_res:float,inflation:float):
        self.grid_res = grid_res
        self.inflation = inflation
        self.occupied = set()
        self.occupied_inflated  = set()
        self.map_ready = False

    def update_map(self, msg: PointCloud2):
        occ = set()
        gen = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        for x, y, z in gen:
            ix = math.floor(x / self.grid_res)
            iy = math.floor(y / self.grid_res)
            iz = math.floor(z / self.grid_res)
            occ.add((ix, iy, iz))

        self.occupied = occ
        self.occupied_inflated = self._inflate(occ)
        self.map_ready = True

    def is_occupied(self, idx: GridIdx) -> bool:
        return idx in self.occupied_inflated

    #lit l 'octomap publié sous la forme d un nuage de point et la voxelise pour obtenir la grille d occupation
    def read_octomap(self, msg: PointCloud2) :#-> Set[GridIdx]:
        occ = set()
        gen = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        for x, y, z in gen:
            ix = math.floor(x / self.grid_res)
            iy = math.floor(y / self.grid_res)
            iz = math.floor(z / self.grid_res)
            occ.add((ix, iy, iz))

        self.occupied = occ
        self.occupied_inflated = self._inflate(occ)
        self.map_ready = True

    def _inflate(self, occ: Set[GridIdx]) -> Set[GridIdx]:

        if self.inflation <= 1e-6:
            return set(occ)

        r = int(math.ceil(self.inflation / self.grid_res))
        inflated: Set[GridIdx] = set()

        offsets = [(dx, dy, dz)
                   for dx in range(-r, r+1)
                   for dy in range(-r, r+1)
                   for dz in range(-r, r+1)
                   if (dx*dx + dy*dy + dz*dz) <= (r*r)]

        for (ix, iy, iz) in occ:
            for dx, dy, dz in offsets:
                inflated.add((ix + dx, iy + dy, iz + dz))
        return inflated

    def world_to_grid(self, p_xyz: List[float]) -> GridIdx:
        grid_res = float(self.grid_res)
        return (math.floor(p_xyz[0] / grid_res),
                math.floor(p_xyz[1] / grid_res),
                math.floor(p_xyz[2] / grid_res))

    def grid_to_world(self, idx: GridIdx) -> Tuple[float, float, float]:
        grid_res = float(self.grid_res)
        return ((idx[0] + 0.5) * grid_res,
                (idx[1] + 0.5) * grid_res,
                (idx[2] + 0.5) * grid_res)