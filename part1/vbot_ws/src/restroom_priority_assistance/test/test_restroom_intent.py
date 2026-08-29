from restroom_priority_assistance.restroom_priority_node import match_destination


DESTINATIONS = {
    'restroom_1f': {
        'label': '一楼无障碍卫生间',
        'phrases': ['我想上厕所', '先去卫生间'],
    }
}


def test_restroom_phrase_matches():
    target, _ = match_destination(DESTINATIONS, '聆灵，我想上厕所')
    assert target == 'restroom_1f'


def test_unrelated_speech_does_not_match():
    assert match_destination(DESTINATIONS, '下一项检查是什么') is None
