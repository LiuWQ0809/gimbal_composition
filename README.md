# Cine Camera Control

这是一个 ROS 2 工作空间，用于 cine 相机云台的目标跟踪控制。核心包 `cine_gimbal_control` 订阅 2D 目标框，结合云台状态与相机内参（可选），通过 PID 控制发布云台指令，使目标保持在画面中心。

## 目录结构

- `cine_gimbal_control/`：ROS 2 包（节点、参数、launch、文档）
- `build.sh`：编译脚本（colcon）
- `run.sh`：启动脚本（ros2 launch）
- `stop.sh`：停止脚本（pkill 追踪节点）
- `build/`、`install/`、`log/`：colcon 生成目录

## 依赖环境

- ROS 2 Humble
- `recomo_controller`（消息：`TrackedObject2D`）
- `jc2804_gimbal_driver`（消息：`GimbalCommand`、`GimbalState`）
- `ronin_rs4_driver`（DJI RS4，可选；直接模式时使用）
- `nlohmann-json3-dev`（解析遥测 JSON）

> 注意：脚本默认尝试 `source /home/nvidia/yanbo/gikWBC9DOF/scripts/orin/recomo_env.bash`，若不存在会回退到 `/opt/ros/humble` 及本地工作空间。若你的目录不同，请修改脚本中的 `WORKSPACE_DIR`。

## 快速开始

在仓库根目录执行：

```bash
./build.sh
./run.sh
```

停止节点：

```bash
./stop.sh
```

也可手动运行：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select cine_gimbal_control --symlink-install
source install/setup.bash
ros2 launch cine_gimbal_control tracking.launch.py
```

RS4 direct mode (attitude/world pointing):

```bash
ros2 launch cine_gimbal_control tracking.launch.py gimbal_driver:=rs4
```

## 话题与数据流

- 订阅：
  - `/recomo/subject_tracking`（`recomo_controller/msg/TrackedObject2D`）
    - 使用 `bbox.x_offset`、`bbox.y_offset`、`bbox.width`、`bbox.height` 与 `confidence`
  - `/recomo/rgb/telemetry`（`std_msgs/msg/String`，JSON 字符串）
    - 可选字段：`width`、`height`、`fx`（焦距像素）
  - `/gimbal/state`（`jc2804_gimbal_driver/msg/GimbalState`）
    - 使用 `pitch_position_rad`、`yaw_position_rad`
  - `/ronin_rs4_driver/status/state`（`ronin_rs4_driver/msg/RoninRs4Status`，当 `gimbal_driver=rs4`）
- 发布：
  - `/gimbal/command`（`jc2804_gimbal_driver/msg/GimbalCommand`）
  - `/ronin_rs4_driver/cmd/control`（`ronin_rs4_driver/msg/RoninRs4Control`，当 `gimbal_driver=rs4`）

> RS4 若需保留 `/gimbal/*` 话题，可启动 `ronin_rs4_driver` 的 compat 节点，并保持 `gimbal_driver=jc2804`。
> RS4 直接模式默认使用姿态控制（`rs4_control_mode=attitude`），发送的是相机朝向而非电机角度。

## 参数说明（`cine_gimbal_control/config/params.yaml`）

| 参数名 | 默认值 | 说明 |
| --- | --- | --- |
| `image_width` | 1920 | 图像宽度 |
| `image_height` | 1080 | 图像高度 |
| `gimbal_driver` | `jc2804` | `jc2804` 或 `rs4` |
| `rs4_control_mode` | `attitude` | `attitude`（世界姿态）或 `joint` |
| `topics.gimbal_state` | `/gimbal/state` | jc2804 状态话题 |
| `topics.gimbal_command` | `/gimbal/command` | jc2804 指令话题 |
| `topics.rs4_status` | `/ronin_rs4_driver/status/state` | RS4 状态话题 |
| `topics.rs4_command` | `/ronin_rs4_driver/cmd/control` | RS4 指令话题 |
| `rs4_axis_map_from_gimbal` | `[2,0,1]` | gimbal[roll,pitch,yaw] -> rs4[yaw,roll,pitch] |
| `rs4_axis_sign` | `[1,1,1]` | RS4 轴向符号 |
| `rs4_axis_zero_offset_rad` | `[0,0,0]` | RS4 零位偏移 |
| `kp_yaw` | 0.000005 | Yaw 比例增益（rad/pixel） |
| `ki_yaw` | 0.0000001 | Yaw 积分增益 |
| `kd_yaw` | 0.0 | Yaw 微分增益 |
| `kp_pitch` | 0.000005 | Pitch 比例增益 |
| `ki_pitch` | 0.0000001 | Pitch 积分增益 |
| `kd_pitch` | 0.0 | Pitch 微分增益 |
| `default_focal_length` | 1000.0 | 参考焦距（像素） |
| `max_velocity_rpm` | 2.0 | 最大速度限制（rpm） |
| `max_step_rad` | 0.002 | 每次控制步长上限（rad） |
| `limit_pitch_rad` | 0.78 | Pitch 角度限幅 |
| `limit_yaw_rad` | 1.57 | Yaw 角度限幅 |
| `deadband_x` | 50 | X 轴死区（pixel） |
| `deadband_y` | 50 | Y 轴死区（pixel） |
| `confidence_threshold` | 0.5 | 置信度阈值 |
| `cmd_rate_hz` | 30.0 | 控制频率 |

## 控制逻辑简述

- 计算目标框中心与图像中心的像素误差。
- PID 计算得到 `delta_yaw`、`delta_pitch`。
- 使用 `max_step_rad` 限制单次变化，用 `limit_*_rad` 做绝对限幅。
- 将目标角度发布到 `/gimbal/command`，速度设为 `max_velocity_rpm`。
- 若遥测提供 `fx`，控制增益会按 `default_focal_length / fx` 自动缩放。

## 调参与注意事项

- 如果转向与预期相反，优先调整 `kp_yaw` / `kp_pitch` 的正负号。
- 遥测 `width/height/fx` 会动态更新分辨率和焦距；未提供则使用参数默认值。
- 像素到角度的理论推导与整定建议见：`cine_gimbal_control/docs/PIXEL_TO_ANGLE.md`。
- 更完整的包级说明见：`cine_gimbal_control/README.md`。

## 常见问题

- **节点启动后无动作**：确认 `/gimbal/state` 与 `/recomo/subject_tracking` 正在发布，且跟踪消息 0.5s 内持续更新。
- **抖动或超调**：减小 `kp_*`、`ki_*`，或增大 `deadband_*` 与 `max_step_rad`。
