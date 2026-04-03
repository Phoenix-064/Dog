---
stepsCompleted: [1]
inputDocuments: []
session_topic: 'ROS 2 包结构规划与位姿转换工作流'
session_goals: '梳理机器狗与箱子坐标位姿的 ROS 2 发布、建立 PnP解算算法集成的包结构'
selected_approach: ''
---

# 头脑风暴：机器狗任务系统的 ROS 2 包结构规划

## 会议目标与背景
基于已与运动控制建立的接口，需开始规划包结构。任务包含：接收 PnP 解算的位姿，把箱子的位姿转换至世界与机器狗坐标系，由 `/localization/boxes` 发布；持续定位并发送 `/localization/dog`；包含任务逻辑（从底边触发、识别场地对角数字题计算等）。
