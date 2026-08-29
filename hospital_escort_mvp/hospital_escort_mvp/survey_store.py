"""保存真实楼层和点位采集结果的 YAML 数据层。"""

from pathlib import Path
from typing import Any, Dict, Optional

import yaml


class SurveyStore:
    def __init__(self, path: str, reference_walk_speed_mps: float = 0.4) -> None:
        self.path = Path(path)
        self.reference_walk_speed_mps = float(reference_walk_speed_mps)

    def _load(self) -> Dict[str, Any]:
        if self.path.exists():
            with self.path.open('r', encoding='utf-8') as stream:
                value = yaml.safe_load(stream)
            if isinstance(value, dict):
                return value
        return {
            'hospital': 'surveyed_hospital',
            'cost_model': {
                'unit': 'seconds',
                'reference_walk_speed_mps': self.reference_walk_speed_mps,
                'description': '现场建图与TF测点生成；边权重需现场计时复核',
            },
            'floors': {},
            'edges': [],
        }

    def _save(self, data: Dict[str, Any]) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.path.with_suffix(self.path.suffix + '.tmp')
        with temporary.open('w', encoding='utf-8') as stream:
            yaml.safe_dump(
                data, stream, allow_unicode=True, sort_keys=False, default_flow_style=False
            )
        temporary.replace(self.path)

    def add_floor(self, floor: str, map_name: str) -> None:
        data = self._load()
        floor_data = data['floors'].setdefault(floor, {'map_name': map_name, 'nodes': {}})
        floor_data['map_name'] = map_name
        floor_data.setdefault('nodes', {})
        self._save(data)

    def add_node(
        self,
        floor: str,
        map_name: str,
        node_id: str,
        name: str,
        node_type: str,
        x: float,
        y: float,
        yaw: float,
        elevator_id: Optional[str] = None,
    ) -> None:
        data = self._load()
        floor_data = data['floors'].setdefault(floor, {'map_name': map_name, 'nodes': {}})
        floor_data['map_name'] = map_name
        node = {
            'name': name,
            'type': node_type,
            'x': round(float(x), 3),
            'y': round(float(y), 3),
            'yaw': round(float(yaw), 4),
        }
        if elevator_id:
            node['elevator_id'] = elevator_id
        floor_data.setdefault('nodes', {})[node_id] = node
        self._save(data)

    def add_edge(
        self,
        edge_id: str,
        from_node: str,
        to_node: str,
        edge_type: str,
        travel_time_sec: float,
        bidirectional: bool = True,
        distance_m: Optional[float] = None,
        elevator_id: Optional[str] = None,
    ) -> None:
        data = self._load()
        edge = {
            'id': edge_id,
            'from': from_node,
            'to': to_node,
            'type': edge_type,
            'travel_time_sec': float(travel_time_sec),
            'bidirectional': bool(bidirectional),
        }
        if distance_m is not None and distance_m >= 0.0:
            edge['distance_m'] = float(distance_m)
        if elevator_id:
            edge['elevator_id'] = elevator_id
        data['edges'] = [item for item in data.get('edges', []) if item['id'] != edge_id]
        data['edges'].append(edge)
        self._save(data)
