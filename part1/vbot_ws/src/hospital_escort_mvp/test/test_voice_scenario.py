from pathlib import Path

import yaml


def test_showroom_voice_scenario_has_deterministic_two_stage_commands():
    path = Path(__file__).parents[1] / 'config' / 'showroom_voice_scenario.yaml'
    scenario = yaml.safe_load(path.read_text(encoding='utf-8'))
    assert scenario['minimum_confidence'] >= 0.6
    assert 'garage_pickup' in scenario['pickup_commands']
    assert set(scenario['destination_commands']) == {
        'registration_1f', 'pharmacy_1f', 'cardiology_3f'
    }
    assert all(
        command['phrases'] and command['accepted_reply']
        for command in scenario['destination_commands'].values()
    )


def test_cardiology_itinerary_has_ordered_exams_wait_and_return():
    path = Path(__file__).parents[1] / 'config' / 'cardiology_itinerary.yaml'
    scenario = yaml.safe_load(path.read_text(encoding='utf-8'))
    targets = [step['target'] for step in scenario['itinerary']]
    assert targets == [
        'blood_draw_1f',
        'ecg_2f',
        'echo_waiting_2f',
        'cardiac_ultrasound_2f',
        'cardiology_3f',
    ]
    assert scenario['wait']['clinical_duration_sec'] == 1800
    assert len(scenario['wait']['jokes']) == 2
    assert scenario['itinerary'][2]['wait_after_arrival'] is True
