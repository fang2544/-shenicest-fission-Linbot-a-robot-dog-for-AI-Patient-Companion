"""医院建图使用的轻量三维占据地图。

地图在三维体素中保存对数概率。规划层主动投影为 2.5D 代价地图；在机器人尚未
提供腿式通行性和落足点控制接口前，这是单层医院环境的安全规划模型。
"""

from dataclasses import dataclass
import gzip
import json
import math
from pathlib import Path
from typing import Dict, Iterable, Iterator, List, Sequence, Tuple


Voxel = Tuple[int, int, int]


@dataclass(frozen=True)
class Costmap2D:
    resolution: float
    origin_x: float
    origin_y: float
    width: int
    height: int
    data: Tuple[int, ...]  # -1 未知，0 可通行，100 占据或膨胀障碍

    def index(self, x: int, y: int) -> int:
        return y * self.width + x

    def value(self, x: int, y: int) -> int:
        return self.data[self.index(x, y)]

    def world_to_cell(self, x: float, y: float) -> Tuple[int, int]:
        return (
            int(math.floor((x - self.origin_x) / self.resolution)),
            int(math.floor((y - self.origin_y) / self.resolution)),
        )

    def cell_to_world(self, x: int, y: int) -> Tuple[float, float]:
        return (
            self.origin_x + (x + 0.5) * self.resolution,
            self.origin_y + (y + 0.5) * self.resolution,
        )

    def contains(self, x: int, y: int) -> bool:
        return 0 <= x < self.width and 0 <= y < self.height


class VoxelOccupancyMap:
    """使用传感器射线增量更新空闲区和占据区的体素地图。"""

    def __init__(
        self,
        resolution: float = 0.10,
        hit_log_odds: float = 0.85,
        miss_log_odds: float = -0.40,
        occupied_threshold: float = 0.60,
        min_log_odds: float = -2.0,
        max_log_odds: float = 3.5,
    ) -> None:
        if resolution <= 0.0:
            raise ValueError('resolution must be positive')
        self.resolution = float(resolution)
        self.hit_log_odds = float(hit_log_odds)
        self.miss_log_odds = float(miss_log_odds)
        self.occupied_threshold = float(occupied_threshold)
        self.min_log_odds = float(min_log_odds)
        self.max_log_odds = float(max_log_odds)
        self._voxels: Dict[Voxel, float] = {}

    def world_to_voxel(self, point: Sequence[float]) -> Voxel:
        values = tuple(
            int(math.floor(float(v) / self.resolution)) for v in point[:3]
        )
        return values  # type: ignore

    def voxel_center(self, voxel: Voxel) -> Tuple[float, float, float]:
        return tuple((v + 0.5) * self.resolution for v in voxel)  # type: ignore

    @staticmethod
    def _ray(start: Voxel, end: Voxel) -> Iterator[Voxel]:
        """生成包含起点和终点的保守三维 DDA 射线。"""
        delta = tuple(end[i] - start[i] for i in range(3))
        steps = max(abs(v) for v in delta)
        if steps == 0:
            yield start
            return
        last = None
        for step in range(steps + 1):
            voxel = tuple(
                int(round(start[i] + delta[i] * step / steps)) for i in range(3)
            )
            if voxel != last:
                yield voxel  # type: ignore
                last = voxel

    def _update(self, voxel: Voxel, increment: float) -> None:
        value = self._voxels.get(voxel, 0.0) + increment
        self._voxels[voxel] = min(self.max_log_odds, max(self.min_log_odds, value))

    def integrate(
        self,
        points: Iterable[Sequence[float]],
        sensor_origin: Sequence[float],
        max_range: float = 8.0,
        max_rays: int = 12000,
    ) -> int:
        """融合测量终点，并从传感器原点沿射线标记空闲空间。

        返回有效射线数量。忽略超出最大距离或包含非有限数值的点，并通过最大
        射线数限制单次回调耗时。
        """
        origin = tuple(float(v) for v in sensor_origin[:3])
        origin_voxel = self.world_to_voxel(origin)
        accepted = 0
        for point in points:
            if accepted >= max_rays:
                break
            xyz = tuple(float(v) for v in point[:3])
            if len(xyz) != 3 or not all(math.isfinite(v) for v in xyz):
                continue
            distance = math.sqrt(sum((xyz[i] - origin[i]) ** 2 for i in range(3)))
            if distance < self.resolution or distance > max_range:
                continue
            end = self.world_to_voxel(xyz)
            ray = list(self._ray(origin_voxel, end))
            for voxel in ray[:-1]:
                self._update(voxel, self.miss_log_odds)
            self._update(end, self.hit_log_odds)
            accepted += 1
        return accepted

    def occupied_voxels(self) -> Iterator[Tuple[Voxel, float]]:
        for voxel, probability_log_odds in self._voxels.items():
            if probability_log_odds >= self.occupied_threshold:
                yield voxel, probability_log_odds

    def clear(self) -> None:
        self._voxels.clear()

    def save(self, path: str) -> None:
        """把完整三维体素状态保存为可移植的压缩 JSON 文件。"""
        target = Path(path)
        target.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            'format': 'hospital_voxel_map_v1',
            'resolution': self.resolution,
            'voxels': [[x, y, z, value] for (x, y, z), value in self._voxels.items()],
        }
        with gzip.open(target, 'wt', encoding='utf-8') as stream:
            json.dump(payload, stream, separators=(',', ':'))

    def load(self, path: str) -> None:
        with gzip.open(path, 'rt', encoding='utf-8') as stream:
            payload = json.load(stream)
        if payload.get('format') != 'hospital_voxel_map_v1':
            raise ValueError('unsupported voxel map format')
        if not math.isclose(float(payload['resolution']), self.resolution):
            raise ValueError('saved map resolution does not match configured resolution')
        self._voxels = {
            (int(row[0]), int(row[1]), int(row[2])): float(row[3])
            for row in payload['voxels']
        }

    def to_costmap(
        self,
        floor_z: float,
        min_obstacle_height: float = 0.15,
        max_obstacle_height: float = 1.80,
        inflation_radius: float = 0.55,
        padding: float = 1.0,
    ) -> Costmap2D:
        if not self._voxels:
            raise ValueError('cannot project an empty voxel map')
        keys = tuple(self._voxels)
        min_vx = min(v[0] for v in keys) - int(math.ceil(padding / self.resolution))
        max_vx = max(v[0] for v in keys) + int(math.ceil(padding / self.resolution))
        min_vy = min(v[1] for v in keys) - int(math.ceil(padding / self.resolution))
        max_vy = max(v[1] for v in keys) + int(math.ceil(padding / self.resolution))
        width, height = max_vx - min_vx + 1, max_vy - min_vy + 1
        data: List[int] = [-1] * (width * height)

        # 净空高度带内只要观测到空闲即可通行，但占据体素始终优先判为障碍。
        min_z = floor_z + min_obstacle_height
        max_z = floor_z + max_obstacle_height
        obstacles = set()
        for (vx, vy, vz), log_odds in self._voxels.items():
            z = (vz + 0.5) * self.resolution
            if not (min_z <= z <= max_z):
                continue
            index = (vy - min_vy) * width + (vx - min_vx)
            if log_odds < self.occupied_threshold:
                data[index] = 0
            else:
                obstacles.add((vx - min_vx, vy - min_vy))

        radius_cells = int(math.ceil(inflation_radius / self.resolution))
        radius_sq = (inflation_radius / self.resolution) ** 2
        for ox, oy in obstacles:
            for dy in range(-radius_cells, radius_cells + 1):
                for dx in range(-radius_cells, radius_cells + 1):
                    x, y = ox + dx, oy + dy
                    if 0 <= x < width and 0 <= y < height and dx * dx + dy * dy <= radius_sq:
                        data[y * width + x] = 100

        return Costmap2D(
            resolution=self.resolution,
            origin_x=min_vx * self.resolution,
            origin_y=min_vy * self.resolution,
            width=width,
            height=height,
            data=tuple(data),
        )
