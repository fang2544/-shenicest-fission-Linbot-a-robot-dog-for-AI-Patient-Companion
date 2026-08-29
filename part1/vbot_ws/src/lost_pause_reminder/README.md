# lost_pause_reminder

ROS 2 Humble 节点：在跟随状态下根据 UWB 距离判定跟丢，暂停跟随和导航，
并播放表情与语音提醒。也支持手持控制器按钮暂停/恢复。

## 构建与运行

将本目录放到工作空间的 `src/lost_pause_reminder`，并确保
`vbot_ros2_msgs` 也位于 `src`：

```bash
source /opt/ros/humble/setup.bash
cd ~/vbot_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select lowlevel_msg function_msgs speech_msgs \
  peripheral_msgs lost_pause_reminder
source install/setup.bash
ros2 launch lost_pause_reminder lost_pause_reminder.launch.py
```

参数见 `config/lost_pause_reminder.yaml`。上线前必须确认话题、服务、action、
QoS、按钮 bit mask 和表情编码：

```bash
ros2 topic list -t
ros2 service list -t
ros2 action list -t
ros2 topic info /lowlevel/UwbState -v
ros2 topic echo /lowlevel/UwbState
ros2 topic echo /lowlevel/WirelessController
```

若发布端为 `BEST_EFFORT`，将对应的 QoS 参数改为 `best_effort`。

出于运动安全考虑，“转向最后已知方向”仍不会下发：公开接口没有说明安全的原地
转向调用方式。确认厂商接口语义后再实现该动作。
