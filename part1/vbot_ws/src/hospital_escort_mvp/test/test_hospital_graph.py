from pathlib import Path

import pytest

from hospital_escort_mvp.hospital_graph import (
    HospitalEdge,
    HospitalGraph,
    HospitalNode,
)


def test_demo_graph_plans_time_optimal_multi_floor_route():
    path = Path(__file__).parents[1] / 'config' / 'hospital_graph.yaml'
    graph = HospitalGraph.from_yaml(str(path))
    route = graph.plan('parking', 'cardiology')
    assert [step.edge.edge_type for step in route.steps] == [
        'walk',
        'elevator',
        'walk',
    ]
    assert [step.to_node.node_id for step in route.steps] == [
        'elevator_A_B1',
        'elevator_A_F3',
        'cardiology',
    ]
    assert route.total_time_sec == 123.0
    assert route.equivalent_distance_m == pytest.approx(73.8)
    assert route.steps[1].from_node.map_name == 'hospital_B1'
    assert route.steps[1].to_node.map_name == 'hospital_F3'


def test_planner_uses_time_not_geometric_distance():
    nodes = {
        key: HospitalNode(key, key, 'F1', 'map', 'poi', x, 0.0, 0.0)
        for key, x in [('start', 0.0), ('near', 1.0), ('far', 10.0), ('goal', 2.0)]
    }
    edges = [
        HospitalEdge('slow_1', 'start', 'near', 'walk', 50.0, True, 1.0),
        HospitalEdge('slow_2', 'near', 'goal', 'walk', 50.0, True, 1.0),
        HospitalEdge('fast_1', 'start', 'far', 'walk', 5.0, True, 10.0),
        HospitalEdge('fast_2', 'far', 'goal', 'walk', 5.0, True, 8.0),
    ]
    route = HospitalGraph(nodes, edges).plan('start', 'goal')
    assert [step.to_node.node_id for step in route.steps] == ['far', 'goal']
    assert route.total_time_sec == 10.0


def test_walk_edge_cannot_connect_independent_maps():
    nodes = {
        'a': HospitalNode('a', 'a', 'F1', 'map_F1', 'poi', 0.0, 0.0, 0.0),
        'b': HospitalNode('b', 'b', 'F2', 'map_F2', 'poi', 0.0, 0.0, 0.0),
    }
    edge = HospitalEdge('invalid', 'a', 'b', 'walk', 10.0, True)
    with pytest.raises(ValueError, match='cannot cross maps'):
        HospitalGraph(nodes, [edge])
