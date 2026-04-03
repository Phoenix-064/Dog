## Deferred from: code review of 1-1-基础工程骨架与接口防腐层搭建 (2026-04-02)

- 通信 Topic 契约是否以代码还是文档为准：故事规格文件定义的规范和 `_bmad-output/implementation-artifacts` 中定义的规范不一致，需要先修订规范后再落地代码对齐。

## Deferred from: code review of 1-1-基础工程骨架与接口防腐层搭建 (2026-04-02)

- 第三方依赖治理（`3rd_party/` 未跟踪）：第三方代码来源与版本锁定策略未在本次变更内收敛，属于既有项目治理问题，暂缓到依赖治理专项处理。

## Deferred from: code review of 1-1-基础工程骨架与接口防腐层搭建 (2026-04-02)

- `/target/pick_tasks` 契约语义与消息类型（`TargetArray[4]` vs `Target3D`）需先完成架构层统一决策后再改代码，避免在骨架阶段引入不稳定迁移。
