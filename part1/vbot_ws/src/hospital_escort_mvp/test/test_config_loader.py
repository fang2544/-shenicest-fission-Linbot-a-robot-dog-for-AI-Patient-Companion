from pathlib import Path

import pytest

from hospital_escort_mvp.config_loader import load_hospital_config


def test_default_config_has_valid_single_floor_route():
    path = Path(__file__).parents[1] / 'config' / 'hospital_F1.yaml'
    config = load_hospital_config(str(path))
    assert config['map_id'] == 'hospital_F1'
    assert len(config['route']) == 6
    assert set(config['route']).issubset(config['locations'])
    assert {config['locations'][key]['floor'] for key in config['route']} == {'F1'}


def test_unknown_route_poi_is_rejected(tmp_path):
    path = tmp_path / 'bad.yaml'
    path.write_text(
        'locations:\n  lobby:\n    name: lobby\n    floor: F1\n    map_id: F1\n'
        '    x: 0\n    y: 0\n    z: 0\n    yaw: 0\n    stop_distance: 1\n'
        'route: [missing]\n',
        encoding='utf-8',
    )
    with pytest.raises(ValueError, match='unknown POIs'):
        load_hospital_config(str(path))
