# VBot 统一急救模式

本包把 VBot 官方语音、导航和 TTS 接口串成统一高优先级急救链路：

`/asr/result` → 急救类型与地点识别 → `/goal_nav` 抢占当前任务 → Nav2 配送 →
`/speech/SpeechTextData` 分步播报对应指导。

当前急救类型与背负物资：

- 心脏骤停：AED、一次性手套、呼吸膜。
- 严重出血：止血敷料、无菌纱布、弹性绷带、一次性手套。
- 严重过敏：急救箱、肾上腺素自动注射器封存盒。
- 低血糖：血糖仪、葡萄糖凝胶封存盒、一次性手套。
- 未分类急救：综合急救箱、急救药品封存盒。

机器人不判断剂量、不自动给药。处方药和自动注射器只交由有资质医护人员、
患者本人或受过训练者按医嘱及产品标签使用。

## 启动

```bash
source /opt/ros/humble/setup.bash
source /vbot_ws/install_sim/setup.bash
ros2 launch aed_emergency_response aed_emergency.launch.py use_sim_time:=true
```

示例语句：`心电图室有人心脏骤停，马上送AED过来`、
`抽血窗口有人严重出血，马上送急救箱`、`有人严重过敏，快送急救药品来`。

状态 JSON 发布在 `/aed_emergency/status`，RViz 标记发布在
`/aed_emergency/markers`。地点、关键词、阈值和指导语均在
`config/aed_emergency.yaml` 中配置。

## 安全边界

这是机器人任务编排与导航验证包，不是医疗器械，也不能诊断患者或决定是否电击。
真实事件必须立即启动院内急救流程并拨打 120；AED 是否电击只听从合规 AED
设备的分析和提示。部署前须由医院急救/设备管理人员审核话术、地点和运行策略。
