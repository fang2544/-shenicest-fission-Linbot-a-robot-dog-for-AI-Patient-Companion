from hospital_escort_mvp.hospital_graph import HospitalEdge, HospitalGraph, HospitalNode
from hospital_escort_mvp.wait_time_planner import WaitTimeEstimator, optimize_visit_groups


def _graph():
    nodes = {
        key: HospitalNode(key, key, 'F1', 'map', 'poi', x, 0.0, 0.0)
        for key, x in [('start', 0), ('near_busy', 1), ('far_quiet', 4)]
    }
    edges = [
        HospitalEdge('near', 'start', 'near_busy', 'walk', 5, True),
        HospitalEdge('far', 'start', 'far_quiet', 'walk', 15, True),
        HospitalEdge('between', 'near_busy', 'far_quiet', 'walk', 12, True),
    ]
    return HospitalGraph(nodes, edges)


def test_hospital_api_queue_beats_shortest_walking_order():
    estimator = WaitTimeEstimator({'near_busy': 10, 'far_quiet': 10})
    estimator.update({'wait_times_sec': {'near_busy': 900, 'far_quiet': 5}})
    visits = [
        {'visit_id': 'near', 'steps': [{'target': 'near_busy'}]},
        {'visit_id': 'far', 'steps': [{'target': 'far_quiet'}]},
    ]
    order, report = optimize_visit_groups(_graph(), 'start', visits, estimator)
    assert [v['visit_id'] for v in order] == ['far', 'near']
    assert report['estimates'][0]['source'] == 'hospital_api'


def test_queue_length_formula_and_stale_historical_fallback():
    now = [1000.0]
    estimator = WaitTimeEstimator(
        {'exam': 600}, stale_after_sec=30, clock=lambda: now[0]
    )
    estimator.update({'departments': {
        'exam': {'queue_length': 6, 'avg_service_time_sec': 120, 'active_counters': 2}
    }})
    assert estimator.estimate('exam').seconds == 360
    assert estimator.estimate('exam').source == 'hospital_api'
    now[0] += 31
    assert estimator.estimate('exam').source == 'historical_ewma'


def test_configured_estimate_when_no_hospital_interface():
    estimator = WaitTimeEstimator({'exam': {'estimated_wait_sec': 420}})
    estimate = estimator.estimate('exam')
    assert estimate.seconds == 420
    assert estimate.source == 'configured_estimate'
