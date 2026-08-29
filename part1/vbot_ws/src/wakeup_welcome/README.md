# wakeup_welcome

ROS 2 Humble 节点：收到 Vbot `WakeupInfo` 后播放欢迎表情并下发 TTS。

## 构建

将本目录放入工作空间的 `src/wakeup_welcome`，并确保
`vbot_ros2_msgs` 也位于 `src`：

```bash
source /opt/ros/humble/setup.bash
cd ~/vbot_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select speech_msgs peripheral_msgs wakeup_welcome
source install/setup.bash
```

## 运行

```bash
ros2 run wakeup_welcome wakeup_welcome_node
# 或
ros2 launch wakeup_welcome wakeup_welcome.launch.py
```

参数集中在 `config/wakeup_welcome.yaml`。真机部署前请确认接口名称和 QoS：

```bash
ros2 topic list -t
ros2 service list -t
ros2 topic info /speech/WakeupInfo -v
```

如果唤醒发布端使用 `BEST_EFFORT`，将 `wakeup_qos_reliability` 改为
`best_effort`。
