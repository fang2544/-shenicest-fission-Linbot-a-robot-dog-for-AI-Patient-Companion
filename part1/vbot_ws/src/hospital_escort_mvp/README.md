# hospital_escort_mvp

医院 2.5D 建图、定位、陪诊导航和等待时间感知任务规划功能包。

完整项目架构、五个功能包关系、调用链路、仿真方式和真机接入边界统一记录在仓库根目录 `README.md`。本文件只保留该功能包的直接使用方法。

## 核心节点

| 节点 | 作用 |
| --- | --- |
| `mapping_planner` | 点云三维建图、2.5D 障碍投影和单层 A* 规划 |
| `multi_floor_navigator` | 多楼层拓扑规划、地图切换、重定位和分段导航 |
| `floor_surveyor` | 真机建图、地图保存和点位采集 |
| `cardiology_itinerary` | 心内科陪诊、等待时间规划和临时点位重排 |
| `pickup_dispatcher` | 校验白名单接客点并下发任务 |
| `mock_vbot` | 无真机时模拟 VITA 接口、TF 和传感器数据 |

## 构建与测试

```bash
source /opt/ros/humble/setup.bash
cd /vbot_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select \
  restroom_priority_assistance hospital_escort_mvp
source install/setup.bash
colcon test --packages-select hospital_escort_mvp
colcon test-result --verbose
```

## 无真机演示

```bash
ros2 launch hospital_escort_mvp mock_cardiology_showroom.launch.py
```

语音触发陪诊：

```bash
ros2 topic pub --once /asr/result function_msgs/msg/AsrResult \
  "{transcript: 开始陪诊, source_type: 0, reject: false, confidence: 0.98}"
```

## 真实楼层建图

```bash
ros2 launch hospital_escort_mvp floor_survey.launch.py \
  floor:=F1 map_name:=hospital_real_F1

ros2 service call /hospital_floor_surveyor/start_mapping \
  std_srvs/srv/Trigger '{}'

# 遥控机器人覆盖走廊和诊室门口，并完成回环。

ros2 service call /hospital_floor_surveyor/finish_and_save_map \
  std_srvs/srv/Trigger '{}'
```

机器人停在科室、电梯或厕所门外的安全位置后，通过点位记录服务读取 `map -> base_link` TF。跨楼层导航还需要工作人员、电梯接口或楼层检测节点确认机器人已经到达目标楼层。

## 等待时间规划

医院系统可向 `/hospital/queue_status` 发布两种 JSON：

```json
{"wait_times_sec":{"blood_draw_1f":300,"ecg_2f":120}}
```

```json
{"departments":{"blood_draw_1f":{"queue_length":8,"avg_service_time_sec":90,"active_counters":2}}}
```

规划器优先使用实时接口；接口数据过期后使用历史指数加权平均；没有任何接口数据时使用 `config/cardiology_itinerary.yaml` 中的保守估算。

## 安全边界

- 本包规划的是平面和多楼层分段路线，不是楼梯、负障碍或机器狗落足点规划器。
- 仿真通过不代表真机运动安全，真实部署必须重新标定机器人轮廓、膨胀半径、速度和停止距离。
- 厕所、科室和电梯点位应记录在门外或候梯区，不能放在封闭空间内部。
