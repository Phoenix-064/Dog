# Story 1.3 30分钟内存压测报告

- 场景：
  - 运行进程：dog_perception_node
  - 输入负载：/camera/image_raw 与 /livox/lidar 双路 30Hz 持续发布
  - 采样窗口：1800 秒（30 分钟）
  - 采样间隔：10 秒（共 180 个样本）
- 阈值：系统 8GB 的 60% = 5033165 KB
- 结果：PASS
- 峰值 RSS：25656 KB
- 峰值 PSS：20483 KB

## 证据文件
- 原始采样：_bmad-output/implementation-artifacts/1-3-memory-soak-30m.log
- 运行日志：_bmad-output/implementation-artifacts/1-3-perception-runtime-30m.log
