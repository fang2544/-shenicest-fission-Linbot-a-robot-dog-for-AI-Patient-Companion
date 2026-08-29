from function_msgs.action import GoalNav
from function_msgs.srv import ControlFollowing, NavigateToTarget, SetSpeak
from slam_msgs.srv import LocCmd, QueryMap, SaveMap, SetSlamMode


def test_goal_navigation_contract():
    goal = GoalNav.Goal()
    for field in ('control', 'x', 'y', 'z', 'yaw'):
        assert hasattr(goal, field)
    result = GoalNav.Result()
    for field in ('success', 'message', 'error_code'):
        assert hasattr(result, field)


def test_following_modes_required_by_mvp_exist():
    request = ControlFollowing.Request()
    assert request.STOP == 0
    assert request.FOLLOWING == 1
    assert request.NAV_TO_POINT == 3
    assert request.ACCOMPANY == 8
    for field in ('stop_distance', 'max_xvel', 'goal_frame', 'pre_check'):
        assert hasattr(request, field)


def test_high_level_mvp_interfaces_are_importable():
    interface_classes = [
        NavigateToTarget,
        SetSpeak,
        LocCmd,
        QueryMap,
        SaveMap,
        SetSlamMode,
    ]
    assert all(
        hasattr(interface, 'Request') and hasattr(interface, 'Response')
        for interface in interface_classes
    )
