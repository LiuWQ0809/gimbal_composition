# Cine Gimbal Control

这是一个用于控制云台跟踪相机的 ROS 2 节点。它接收 `/recomo/subject_tracking` 中的 2D 框，并控制云台（发布 `/gimbal/command`）将目标保持在图像中心。
RS4 direct mode publishes to `/ronin_rs4_driver/*` when `gimbal_driver=rs4` (or keep `/gimbal/*` via the compat bridge).
RS4 direct mode defaults to attitude control (`rs4_control_mode=attitude`), so commands are camera pointing in world coordinates.

## 硬件与坐标系说明

根据您的硬件配置（参考 `/home/nvidia/recomoArm/recomo-urdfs/urdf/recomoDemo1.urdf`）：

- **电机映射**:
  - ID 1: Roll (横滚)
  - ID 2: Pitch (俯仰)
  - ID 3: Yaw (偏航)
- **控制逻辑**:
  - **Yaw (偏航)**: 用于水平跟踪。
    - 图像误差 `error_x > 0` (目标在左侧) -> 期望动作: 向左转 (CCW)。
    - 根据驱动说明 "正值(+): 逆时针旋转"，代码默认使用 `+ kp_yaw * error`。
  - **Pitch (俯仰)**: 用于垂直跟踪。
    - 图像误差 `error_y > 0` (目标在上方) -> 期望动作: 向上看。
    - 常见光学坐标系中，向上看通常对应 Pitch 减小 (或负方向)。代码默认使用 `- kp_pitch * error`。
  - **参数调整**: 如果发现跟踪方向相反，请在 `params.yaml` 中将对应的 `kp` 值设为负数。

For recomoProto1 (DJI RS4), verify axis mapping/offsets against
`/home/nvidia/yanbo/gikWBC9DOF/models/recomoProto1/recomoProto1.urdf` and update
`rs4_axis_*` parameters as needed.

## 像素到角度的换算

控制核心是将像素误差乘以系数 $K_p$ 转换为目标弧度。
- **理论基础**: 基于针孔相机模型，理论 $K_p \approx 1 / f$ (焦距像素)。
- **详细指南**: 请参阅 [docs/PIXEL_TO_ANGLE.md](docs/PIXEL_TO_ANGLE.md) 获取详细推导和参数整定方法。

## 依赖

- ROS 2 Humble
- `recomo_controller` (消息定义: `TrackedObject2D`)
- `jc2804_gimbal_driver` (消息定义: `GimbalCommand`, `GimbalState`)
- `ronin_rs4_driver` (DJI RS4 direct mode)

## 编译

请确保已 source 所有依赖的工作空间：

```bash
source /home/nvidia/yanbo/gikWBC9DOF/scripts/orin/recomo_env.bash
# 或者手动 source:
# source /opt/ros/humble/setup.bash
# source /home/nvidia/recomoArm_ws/install/setup.bash
# source /home/nvidia/yanbo/ros2_ws/install/setup.bash
```

然后在工作空间根目录（例如 `/home/nvidia/workspaces/cine_camera/control`）编译：

```bash
cd /home/nvidia/workspaces/cine_camera/control
colcon build --packages-select cine_gimbal_control
source install/setup.bash
```

## 运行

使用 launch 文件启动（加载参数）：

```bash
ros2 launch cine_gimbal_control tracking.launch.py
```

RS4 direct mode:

```bash
ros2 launch cine_gimbal_control tracking.launch.py gimbal_driver:=rs4
```

或者直接运行节点：

```bash
ros2 run cine_gimbal_control tracking_node
```

## 参数说明 (`config/params.yaml`)

| 参数名 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `image_width` | int | 1920 | 图像宽度 (pixel) |
| `image_height` | int | 1080 | 图像高度 (pixel) |
| `gimbal_driver` | string | `jc2804` | `jc2804` or `rs4` |
| `rs4_control_mode` | string | `attitude` | `attitude` (world pointing) or `joint` |
| `topics.gimbal_state` | string | `/gimbal/state` | jc2804 state topic |
| `topics.gimbal_command` | string | `/gimbal/command` | jc2804 command topic |
| `topics.rs4_status` | string | `/ronin_rs4_driver/status/state` | RS4 status topic |
| `topics.rs4_command` | string | `/ronin_rs4_driver/cmd/control` | RS4 control topic |
| `rs4_axis_map_from_gimbal` | int[3] | `[2,0,1]` | gimbal[roll,pitch,yaw] -> rs4[yaw,roll,pitch] |
| `rs4_axis_sign` | double[3] | `[1,1,1]` | RS4 axis sign |
| `rs4_axis_zero_offset_rad` | double[3] | `[0,0,0]` | RS4 axis offset |
| `kp_yaw` | double | 0.0005 | Yaw 轴比例增益 (rad/pixel) |
| `kp_pitch` | double | 0.0005 | Pitch 轴比例增益 (rad/pixel) |
| `deadband_x` | int | 20 | X 轴死区 (pixel)，误差小于此值不调整 |
| `deadband_y` | int | 20 | Y 轴死区 (pixel) |
| `confidence_threshold` | double | 0.5 | 目标置信度阈值，低于此值不跟踪 |
| `cmd_rate_hz` | double | 30.0 | 控制命令发布频率 |

## 控制逻辑

- 读取当前云台状态 (`/gimbal/state`) 获取当前 `yaw` 和 `pitch`。
- 计算图像中心与目标中心 (`/recomo/subject_tracking`) 的像素误差。
- 应用 P 控制器：`Target = Current + Kp * Error`。
- 发布位置指令到 `/gimbal/command`。

## 注意事项

- 确保 `/recomo/subject_tracking` 正在以足够频率发布。
- 确保云台驱动已启动，并发布 `/gimbal/state`。
- 坐标系假设：
  - User looking at image: X is Right, Y is Down.
  - Gimbal: Left Rotation (CCW) increases Yaw? Check `jc2804` documentation.
  - Current code assumes: 
    - Box Left -> Error Positive -> Increase Yaw (Turn Left).
    - Box Top -> Error Positive -> Decrease Pitch (Look Up).
    - 根据实际云台方向定义可能需要调整 +/- 号。
