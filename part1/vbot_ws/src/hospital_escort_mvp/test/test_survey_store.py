from pathlib import Path

import yaml

from hospital_escort_mvp.hospital_graph import HospitalGraph
from hospital_escort_mvp.survey_store import SurveyStore


def test_showroom_route_stays_inside_small_demo_area():
    path = Path(__file__).parents[1] / 'config' / 'showroom_graph.yaml'
    graph = HospitalGraph.from_yaml(str(path))
    assert all(abs(node.x) <= 2.0 and abs(node.y) <= 1.5 for node in graph.nodes.values())
    config = yaml.safe_load(path.read_text(encoding='utf-8'))
    policy = config['pickup_policy']
    assert policy['home_node'] == 'cardiology_3f'
    assert policy['arbitrary_coordinates_allowed'] is False
    assert set(policy['allowed_pickup_nodes']).issubset(graph.nodes)
    assert set(policy['allowed_destination_nodes']).issubset(graph.nodes)
    assert set(policy['allowed_pickup_nodes']) == {'garage_pickup'}
    route = graph.plan('cardiology_3f', 'blood_draw_1f')
    assert route.total_time_sec == 50.0
    assert any(step.edge.edge_type == 'elevator' for step in route.steps)
    assert route.start_node.floor == 'F3'
    assert route.goal_node.floor == 'F1'


def test_survey_store_builds_navigator_compatible_graph(tmp_path):
    path = tmp_path / 'survey.yaml'
    store = SurveyStore(str(path))
    store.add_node('F1', 'real_F1', 'lobby', '大厅', 'poi', 1.0, 2.0, 0.1)
    store.add_node(
        'F1', 'real_F1', 'lift_F1', '电梯F1', 'elevator',
        3.0, 2.0, 0.0, elevator_id='A',
    )
    store.add_node(
        'F2', 'real_F2', 'lift_F2', '电梯F2', 'elevator',
        4.0, 5.0, 1.57, elevator_id='A',
    )
    store.add_node('F2', 'real_F2', 'clinic', '诊室', 'poi', 8.0, 5.0, 3.14)
    store.add_edge('walk_F1', 'lobby', 'lift_F1', 'walk', 10.0, distance_m=3.0)
    store.add_edge(
        'lift_A', 'lift_F1', 'lift_F2', 'elevator', 45.0, elevator_id='A'
    )
    store.add_edge('walk_F2', 'lift_F2', 'clinic', 'walk', 15.0, distance_m=5.0)
    graph = HospitalGraph.from_yaml(str(path))
    route = graph.plan('lobby', 'clinic')
    assert route.total_time_sec == 70.0
    assert route.goal_node.map_name == 'real_F2'
