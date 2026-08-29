# 贡献指南

## 开发基线

使用 Ubuntu 22.04、ROS 2 Humble 和 Python 3.10。提交前先运行 `rosdep install`、`colcon build`、相关包测试和 `scripts/quality/check_repository_clean.sh`。

## 修改边界

- 业务逻辑放在五个功能包中；共享仿真逻辑放在 `vbot_simulation`。
- 不直接修改 `vbot_ws/src/vbot_ros2_msgs` 的生成型接口文件。
- ROS 参数放入 `config/*.yaml`，不要在节点中硬编码医院坐标、话术或接口地址。
- 新增运动行为时必须提供取消、超时、限速和失败状态。
- 新增医疗话术时必须说明安全边界，并由医疗专业人员审核后再部署。

## 提交检查

```bash
./scripts/quality/check_repository_clean.sh
cd vbot_ws
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

不得提交 `build/`、`install/`、`log/`、运行报告、地图输出、压缩包、`__pycache__`、`.pyc`、`__MACOSX` 或 `._*`。
