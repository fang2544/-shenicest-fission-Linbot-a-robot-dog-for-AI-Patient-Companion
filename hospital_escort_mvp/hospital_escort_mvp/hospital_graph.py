"""校验医院多楼层拓扑，并按总耗时规划最优路线。"""

from dataclasses import asdict, dataclass
import heapq
import math
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import yaml


@dataclass(frozen=True)
class HospitalNode:
    node_id: str
    name: str
    floor: str
    map_name: str
    node_type: str
    x: float
    y: float
    yaw: float
    elevator_id: Optional[str] = None


@dataclass(frozen=True)
class HospitalEdge:
    edge_id: str
    from_node: str
    to_node: str
    edge_type: str
    travel_time_sec: float
    bidirectional: bool
    distance_m: Optional[float] = None
    elevator_id: Optional[str] = None


@dataclass(frozen=True)
class RouteStep:
    from_node: HospitalNode
    to_node: HospitalNode
    edge: HospitalEdge
    equivalent_distance_m: float


@dataclass(frozen=True)
class HospitalRoute:
    start_node: HospitalNode
    goal_node: HospitalNode
    steps: Tuple[RouteStep, ...]
    total_time_sec: float
    equivalent_distance_m: float

    def as_dict(self) -> Dict[str, Any]:
        return {
            'start_node': self.start_node.node_id,
            'goal_node': self.goal_node.node_id,
            'total_time_sec': self.total_time_sec,
            'equivalent_distance_m': self.equivalent_distance_m,
            'steps': [
                {
                    'from': step.from_node.node_id,
                    'to': step.to_node.node_id,
                    'type': step.edge.edge_type,
                    'edge_id': step.edge.edge_id,
                    'travel_time_sec': step.edge.travel_time_sec,
                    'equivalent_distance_m': step.equivalent_distance_m,
                    'from_map': step.from_node.map_name,
                    'to_map': step.to_node.map_name,
                }
                for step in self.steps
            ],
        }


class HospitalGraph:
    def __init__(
        self,
        nodes: Dict[str, HospitalNode],
        edges: List[HospitalEdge],
        reference_walk_speed_mps: float = 0.6,
        name: str = 'hospital',
    ) -> None:
        if reference_walk_speed_mps <= 0.0:
            raise ValueError('reference_walk_speed_mps must be positive')
        self.name = name
        self.nodes = nodes
        self.edges = edges
        self.reference_walk_speed_mps = float(reference_walk_speed_mps)
        self._adjacency: Dict[str, List[Tuple[str, HospitalEdge]]] = {
            node_id: [] for node_id in nodes
        }
        self._validate_and_index()

    def _validate_and_index(self) -> None:
        for edge in self.edges:
            if edge.from_node not in self.nodes or edge.to_node not in self.nodes:
                raise ValueError(f'edge {edge.edge_id} references an unknown node')
            if edge.travel_time_sec <= 0.0:
                raise ValueError(f'edge {edge.edge_id} must have positive travel time')
            source, target = self.nodes[edge.from_node], self.nodes[edge.to_node]
            if edge.edge_type == 'walk' and source.map_name != target.map_name:
                raise ValueError(f'walk edge {edge.edge_id} cannot cross maps')
            if edge.edge_type == 'elevator':
                elevator_ids = {source.elevator_id, target.elevator_id, edge.elevator_id}
                if source.map_name == target.map_name or len(elevator_ids) != 1:
                    raise ValueError(
                        f'elevator edge {edge.edge_id} must connect matching elevator stops'
                    )
            if edge.edge_type not in {'walk', 'elevator'}:
                raise ValueError(f'unsupported edge type {edge.edge_type!r}')
            self._adjacency[edge.from_node].append((edge.to_node, edge))
            if edge.bidirectional:
                self._adjacency[edge.to_node].append((edge.from_node, edge))

    @classmethod
    def from_yaml(cls, path: str) -> 'HospitalGraph':
        with Path(path).open('r', encoding='utf-8') as stream:
            config = yaml.safe_load(stream)
        if not isinstance(config, dict):
            raise ValueError('hospital graph YAML must be a mapping')
        nodes: Dict[str, HospitalNode] = {}
        for floor, floor_data in config.get('floors', {}).items():
            map_name = str(floor_data['map_name'])
            for node_id, value in floor_data.get('nodes', {}).items():
                if node_id in nodes:
                    raise ValueError(f'duplicate node id {node_id!r}')
                nodes[node_id] = HospitalNode(
                    node_id=node_id,
                    name=str(value['name']),
                    floor=str(floor),
                    map_name=map_name,
                    node_type=str(value['type']),
                    x=float(value['x']),
                    y=float(value['y']),
                    yaw=float(value.get('yaw', 0.0)),
                    elevator_id=value.get('elevator_id'),
                )
        edges = [
            HospitalEdge(
                edge_id=str(value['id']),
                from_node=str(value['from']),
                to_node=str(value['to']),
                edge_type=str(value['type']),
                travel_time_sec=float(value['travel_time_sec']),
                bidirectional=bool(value.get('bidirectional', True)),
                distance_m=(
                    float(value['distance_m']) if 'distance_m' in value else None
                ),
                elevator_id=value.get('elevator_id'),
            )
            for value in config.get('edges', [])
        ]
        speed = config.get('cost_model', {}).get('reference_walk_speed_mps', 0.6)
        return cls(
            nodes=nodes,
            edges=edges,
            reference_walk_speed_mps=float(speed),
            name=str(config.get('hospital', 'hospital')),
        )

    def plan(self, start_node: str, goal_node: str) -> HospitalRoute:
        if start_node not in self.nodes or goal_node not in self.nodes:
            raise ValueError('start or goal node does not exist')
        frontier = [(0.0, start_node)]
        cost = {start_node: 0.0}
        parent: Dict[str, Tuple[str, HospitalEdge]] = {}
        while frontier:
            current_cost, current = heapq.heappop(frontier)
            if current_cost > cost[current]:
                continue
            if current == goal_node:
                break
            for neighbor, edge in self._adjacency[current]:
                candidate = current_cost + edge.travel_time_sec
                if neighbor not in cost or candidate < cost[neighbor]:
                    cost[neighbor] = candidate
                    parent[neighbor] = current, edge
                    heapq.heappush(frontier, (candidate, neighbor))
        if goal_node not in cost:
            raise RuntimeError(f'no route from {start_node} to {goal_node}')
        steps: List[RouteStep] = []
        cursor = goal_node
        while cursor != start_node:
            previous, edge = parent[cursor]
            equivalent = edge.travel_time_sec * self.reference_walk_speed_mps
            steps.append(
                RouteStep(
                    from_node=self.nodes[previous],
                    to_node=self.nodes[cursor],
                    edge=edge,
                    equivalent_distance_m=equivalent,
                )
            )
            cursor = previous
        steps.reverse()
        total_time = cost[goal_node]
        return HospitalRoute(
            start_node=self.nodes[start_node],
            goal_node=self.nodes[goal_node],
            steps=tuple(steps),
            total_time_sec=total_time,
            equivalent_distance_m=total_time * self.reference_walk_speed_mps,
        )

    def as_dict(self) -> Dict[str, Any]:
        return {
            'hospital': self.name,
            'cost_model': {
                'unit': 'seconds',
                'reference_walk_speed_mps': self.reference_walk_speed_mps,
            },
            'nodes': [asdict(node) for node in self.nodes.values()],
            'edges': [
                {
                    **asdict(edge),
                    'equivalent_distance_m': math.ceil(
                        edge.travel_time_sec * self.reference_walk_speed_mps * 10
                    ) / 10,
                }
                for edge in self.edges
            ],
        }
