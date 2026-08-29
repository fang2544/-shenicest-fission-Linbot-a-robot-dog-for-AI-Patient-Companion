from pathlib import Path
from typing import Any, Dict

import yaml


REQUIRED_POI_FIELDS = {'name', 'floor', 'map_id', 'x', 'y', 'z', 'yaw', 'stop_distance'}


def load_hospital_config(path: str) -> Dict[str, Any]:
    config_path = Path(path)
    with config_path.open('r', encoding='utf-8') as stream:
        config = yaml.safe_load(stream)

    if not isinstance(config, dict):
        raise ValueError('Hospital configuration must be a YAML mapping')

    locations = config.get('locations')
    route = config.get('route')
    if not isinstance(locations, dict) or not locations:
        raise ValueError('Configuration must contain non-empty locations')
    if not isinstance(route, list) or not route:
        raise ValueError('Configuration must contain a non-empty route')

    for key, poi in locations.items():
        if not isinstance(poi, dict):
            raise ValueError(f'POI {key!r} must be a mapping')
        missing = REQUIRED_POI_FIELDS - set(poi)
        if missing:
            raise ValueError(f'POI {key!r} is missing fields: {sorted(missing)}')

    missing_route = [key for key in route if key not in locations]
    if missing_route:
        raise ValueError(f'Route references unknown POIs: {missing_route}')
    return config
