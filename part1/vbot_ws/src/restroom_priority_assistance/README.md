# 语音找厕所最高优先功能

本包独立订阅 `/asr/result`，识别“我想上厕所”“先去卫生间”等表达，向
`/hospital/priority_destination` 发布最高优先级目的地请求。医院陪诊包负责取消
当前 Nav2 任务、前往厕所，再恢复未完成路线。

```bash
ros2 launch restroom_priority_assistance restroom_priority.launch.py
```

地图必须包含配置中的 `restroom_1f` 安全停靠点。真实医院应在厕所门外记录 POI，
不能把机器人导航到厕位内部。
