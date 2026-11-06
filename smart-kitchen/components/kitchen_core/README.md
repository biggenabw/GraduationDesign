# kitchen_core

厨房安全控制核心组件，属于 C1 方案（同一块 ESP32-S3 上 xiaozhi 负责语音/显示/MCP，本组件负责传感器、执行器和安全判定）。

## 边界

- 只依赖 ESP-IDF：`freertos`、`esp_timer`、`log`。
- **不 include** `application.h`、`board.h`、`mcp_server.h`，也不直接调用 xiaozhi 的音频/显示/协议。
- 所有 MCP/OneNET/按键请求通过 `kitchen_core_request()` / `kitchen_core_post_key()` 进入本组件，回调立即返回。
- 安全任务不等待音频、网络或 LLM。

## 目录

```
kitchen_core/
├── CMakeLists.txt
├── include/
│   └── kitchen_core.h          # 公共 C 接口
└── src/
    ├── kitchen_core.cc         # 上下文、队列、任务、快照
    ├── sensor_manager.cc       # SHT30/MLX90614/MQ/雷达（待实现）
    ├── actuator_manager.cc     # 舵机/继电器/风扇/蜂鸣器/LED（待实现）
    ├── safety_monitor.cc       # 燃气/干烧/湿度/故障判定（待实现）
    └── params.cc               # NVS 参数（待实现）
```

## 当前状态

这是**骨架**，只实现：

- `kitchen_core_init()`：创建互斥锁、消息队列、默认快照。
- `kitchen_core_start()`：创建 `kitchen_safety` / `kitchen_ctrl` 两个任务。
- `kitchen_core_get_snapshot()`：线程安全复制快照。
- `kitchen_core_request()`：投递动作请求。
- `kitchen_core_post_key()`：投递按键事件。
- `kitchen_core_set_event_callback()`：注册事件回调。

尚未实现任何真实传感器/执行器驱动，也不控制 GPIO。

## 任务

| 任务 | 优先级 | Core | 周期 | 说明 |
| --- | --- | --- | --- | --- |
| `kitchen_safety` | 24 | 1 | 100 ms | 安全判定；当前只更新 uptime |
| `kitchen_ctrl` | 22 | 1 | 事件驱动 | 处理动作/按键；当前只更新快照并打日志 |

## 下一步

1. 在 `sensor_manager.cc` 实现 SHT30、MQ-2/MQ-4、MLX90614、LD2402。
2. 在 `actuator_manager.cc` 实现 SG90、继电器、L9110、蜂鸣器、LED。
3. 在 `safety_monitor.cc` 实现燃气/干烧/湿度/故障状态机。
4. 在 `params.cc` 实现 NVS 参数读写。
5. 通过 `main` 的 `KITCHEN_SELFTEST` 模式先独立验证，再接入 xiaozhi。
