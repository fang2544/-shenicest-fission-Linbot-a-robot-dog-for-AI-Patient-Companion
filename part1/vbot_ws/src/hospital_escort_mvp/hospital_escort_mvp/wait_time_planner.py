"""为无固定顺序的医院检查计算包含等待时间的访问顺序。"""

from dataclasses import dataclass
import itertools
import time
from typing import Any, Dict, Iterable, List, Tuple


@dataclass(frozen=True)
class WaitEstimate:
    seconds: float
    source: str


class WaitTimeEstimator:
    """读取医院排队数据；接口不可用时使用可重复的离线估算。"""

    def __init__(
        self,
        fallback: Dict[str, Any],
        stale_after_sec: float = 120.0,
        ewma_alpha: float = 0.35,
        clock=time.time,
    ) -> None:
        self._fallback = fallback or {}
        self._stale_after = max(1.0, float(stale_after_sec))
        self._alpha = min(1.0, max(0.01, float(ewma_alpha)))
        self._clock = clock
        self._live: Dict[str, Tuple[float, float]] = {}
        self._history: Dict[str, float] = {}

    @staticmethod
    def _seconds(value: Any) -> float:
        if isinstance(value, (int, float)):
            return max(0.0, float(value))
        if not isinstance(value, dict):
            raise ValueError('queue value must be seconds or an object')
        if 'estimated_wait_sec' in value:
            return max(0.0, float(value['estimated_wait_sec']))
        queue = max(0.0, float(value.get('queue_length', 0.0)))
        service = max(0.0, float(value.get('avg_service_time_sec', 0.0)))
        counters = max(1.0, float(value.get('active_counters', 1.0)))
        return queue * service / counters

    def update(self, payload: Dict[str, Any]) -> Dict[str, float]:
        values = payload.get('wait_times_sec', payload.get('departments', {}))
        if not isinstance(values, dict):
            raise ValueError('queue payload requires wait_times_sec/departments mapping')
        observed_at = float(payload.get('timestamp', self._clock()))
        parsed = {}
        for node_id, value in values.items():
            seconds = self._seconds(value)
            parsed[str(node_id)] = seconds
            self._live[str(node_id)] = (seconds, observed_at)
            old = self._history.get(str(node_id), seconds)
            self._history[str(node_id)] = self._alpha * seconds + (1.0 - self._alpha) * old
        return parsed

    def estimate(self, node_id: str, now: float = None) -> WaitEstimate:
        current = self._clock() if now is None else float(now)
        if node_id in self._live:
            seconds, observed_at = self._live[node_id]
            if current - observed_at <= self._stale_after:
                return WaitEstimate(seconds, 'hospital_api')
        if node_id in self._history:
            return WaitEstimate(self._history[node_id], 'historical_ewma')
        value = self._fallback.get(node_id, 0.0)
        return WaitEstimate(self._seconds(value), 'configured_estimate')


def optimize_visit_groups(
    graph,
    start_node: str,
    groups: Iterable[Dict[str, Any]],
    estimator: WaitTimeEstimator,
) -> Tuple[List[Dict[str, Any]], Dict[str, Any]]:
    """寻找行走时间与预计等待时间之和最小的检查顺序。"""
    visits = list(groups)
    if len(visits) > 8:
        raise ValueError('at most eight reorderable visit groups are supported')
    best = None
    infeasible = 0
    for permutation in itertools.permutations(visits):
        cursor = start_node
        travel = 0.0
        waiting = 0.0
        estimates = []
        try:
            for visit in permutation:
                for step in visit['steps']:
                    target = str(step['target'])
                    travel += graph.plan(cursor, target).total_time_sec
                    cursor = target
                wait_node = str(visit.get('wait_node', cursor))
                estimate = estimator.estimate(wait_node)
                # 排队数据表示当前等待时间；途中行走和前序检查会消耗这段等待时间。
                projected_wait = max(0.0, estimate.seconds - (travel + waiting))
                waiting += projected_wait
                estimates.append({
                    'visit_id': str(visit['visit_id']),
                    'node': wait_node,
                    'reported_wait_sec': estimate.seconds,
                    'projected_wait_on_arrival_sec': projected_wait,
                    'source': estimate.source,
                })
        except RuntimeError:
            infeasible += 1
            continue
        key = (travel + waiting, waiting, travel, tuple(v['visit_id'] for v in permutation))
        if best is None or key < best[0]:
            best = (key, list(permutation), estimates)
    if best is None:
        raise RuntimeError('no feasible order for remaining examination visits')
    key, order, estimates = best
    return order, {
        'objective_sec': key[0],
        'travel_sec': key[2],
        'wait_sec': key[1],
        'order': [str(v['visit_id']) for v in order],
        'estimates': estimates,
        'infeasible_permutations': infeasible,
    }
