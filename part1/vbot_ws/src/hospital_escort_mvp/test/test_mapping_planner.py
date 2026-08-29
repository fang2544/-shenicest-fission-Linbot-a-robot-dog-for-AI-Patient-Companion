from pathlib import Path
import struct

import pytest

from hospital_escort_mvp.grid_planner import PlanningError, plan_path
from hospital_escort_mvp.mapping_planner_node import _xyz_points
from hospital_escort_mvp.voxel_mapping import Costmap2D, VoxelOccupancyMap
from sensor_msgs.msg import PointCloud2, PointField


def test_voxel_map_carves_free_space_and_marks_obstacle():
    voxel_map = VoxelOccupancyMap(resolution=0.5)
    for _ in range(2):
        voxel_map.integrate([(2.0, 0.0, 0.5)], (0.0, 0.0, 0.5))
    costmap = voxel_map.to_costmap(
        floor_z=0.0, min_obstacle_height=0.1, max_obstacle_height=1.0,
        inflation_radius=0.0, padding=0.0,
    )
    assert costmap.value(*costmap.world_to_cell(2.0, 0.0)) == 100
    assert costmap.value(*costmap.world_to_cell(1.0, 0.0)) == 0


def test_voxel_map_round_trip(tmp_path: Path):
    source = VoxelOccupancyMap(resolution=0.25)
    source.integrate([(1.0, 0.0, 0.5)], (0.0, 0.0, 0.5))
    path = tmp_path / 'map.json.gz'
    source.save(str(path))
    restored = VoxelOccupancyMap(resolution=0.25)
    restored.load(str(path))
    assert list(restored.occupied_voxels()) == list(source.occupied_voxels())


def test_astar_routes_around_wall_gap():
    width = height = 7
    data = [0] * (width * height)
    for y in range(6):
        data[y * width + 3] = 100
    costmap = Costmap2D(1.0, 0.0, 0.0, width, height, tuple(data))
    path = plan_path(costmap, (1.5, 1.5), (5.5, 1.5))
    assert path[0] == (1.5, 1.5)
    assert path[-1] == (5.5, 1.5)
    assert any(y > 5.0 for _, y in path)
    assert all(costmap.value(*costmap.world_to_cell(x, y)) == 0 for x, y in path)


def test_astar_rejects_fully_blocked_goal():
    data = [0, 100, 0, 100, 100, 100, 0, 100, 0]
    costmap = Costmap2D(1.0, 0.0, 0.0, 3, 3, tuple(data))
    with pytest.raises(PlanningError):
        plan_path(costmap, (0.5, 0.5), (2.5, 2.5))


def test_point_cloud_parser_honors_organized_row_padding():
    cloud = PointCloud2()
    cloud.height, cloud.width = 2, 1
    cloud.point_step, cloud.row_step = 12, 16
    cloud.fields = [
        PointField(
            name=name,
            offset=index * 4,
            datatype=PointField.FLOAT32,
            count=1,
        )
        for index, name in enumerate(('x', 'y', 'z'))
    ]
    cloud.data = (
        struct.pack('<fff', 1.0, 2.0, 3.0) + b'pad!'
        + struct.pack('<fff', 4.0, 5.0, 6.0) + b'pad!'
    )
    assert list(_xyz_points(cloud, limit=10)) == [
        (1.0, 2.0, 3.0),
        (4.0, 5.0, 6.0),
    ]
