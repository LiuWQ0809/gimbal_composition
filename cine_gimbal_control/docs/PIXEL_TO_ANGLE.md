# 像素到角度的换算方案

在云台跟踪控制中，我们需要将图像上的**像素误差 (Pixel Error)** 转换为云台需要转动的**物理角度 (Physical Angle)**。主要有两种方法：

1.  **工程估算法 (PID 调试法)**
2.  **几何计算法 (针孔相机模型)**

---

## 1. 工程估算法 (当前默认方案)

目前的控制代码使用的是简单的比例控制器 (P-Controller)：

$$
\text{TargetAngle} = \text{CurrentAngle} + K_p \times \text{PixelError}
$$

其中 $K_p$ (在 `params.yaml` 中为 `kp_yaw` 和 `kp_pitch`) 的单位是 **"弧度/像素"**。

### 如何理解 $K_p$ ?
$K_p$ 代表了：**图像上每偏离 1 个像素，云台应该转动多少弧度**。

*   **默认值**: `0.0005` rad/pixel
    *   这意味着如果目标偏离中心 100 像素，云台将转动 $100 \times 0.0005 = 0.05$ rad ($\approx 2.8^\circ$)。
*   **调试方法**:
    *   如果跟踪**反应太慢**，目标总是在画面边缘晃荡：**增大** $K_p$。
    *   如果云台**震荡**（左右来回摆动）：**减小** $K_p$。

---

## 2. 几何计算法 (理论精确值)

如果您希望根据相机的物理参数计算精确的转换系数，可以使用**针孔相机模型**。

### 基本公式

假设相机焦距为 $f$ (单位：像素)，目标偏离中心的像素数为 $\Delta x$，则目标相对于相机的角度 $\theta$ 为：

$$
\tan(\theta) = \frac{\Delta x}{f}
\implies \theta = \arctan\left(\frac{\Delta x}{f}\right)
$$

在小角度下（目标靠近中心时），$\tan(\theta) \approx \theta$，公式简化为：

$$
\theta \approx \frac{1}{f} \times \Delta x
$$

这就对应了我们的 P 控制器形式，其中理论上的 $K_p$ 就是焦距的倒数：

$$
K_{p\_theoretical} = \frac{1}{f}
$$

### 如何获取焦距 $f$ (像素)?

#### A. 从视场角 (FOV) 计算
如果您知道相机的水平视场角 ($FOV_h$) 和图像宽度 ($W$)：

$$
f \approx \frac{W / 2}{\tan(FOV_h / 2)}
$$

**例如**:
*   图像宽度 $W = 1920$ 像素
*   水平视场角 $FOV_h = 90^\circ$ ($\approx 1.57$ rad)
*   计算:
    $$ f = \frac{960}{\tan(45^\circ)} = \frac{960}{1} = 960 \text{ pixels} $$
*   对应 $K_p = 1 / 960 \approx 0.00104$ rad/pixel

#### B. 从标定文件获取 (最准确)
查看系统中的标定文件 (如 `/home/nvidia/intrin/H120UA-H04230809.json`)：

```json
"intrinsic": [
    1015.6158, 0, ...  // f_x
    ...
]
```

*   这里 $f_x \approx 1015.6$ 像素。
*   理论 $K_p = 1 / 1015.6 \approx 0.00098$ rad/pixel。

### 结论与建议

*   **理论值**: $\approx 0.001$ (即 1 像素对应 0.001 弧度)。
    *   使用这个值作为 $K_p$，云台会尝试**一步到位**修正误差（Deadbeat Control）。
*   **安全值**: $\approx 0.0005$ (理论值的一半)。
    *   为了平滑运动并防止超调震荡，我们通常使用理论值的 **0.5 到 0.8 倍**。
    *   这也是为什么默认配置选用了 `0.0005`。

### 总结换算步骤

1.  **确定焦距 $f$**:
    *   如果不知焦距，估算 $f \approx \text{ImageWidth} / 2$ (对应约 90 度视场角)。
2.  **计算角度**:
    *   $\text{Angle(rad)} = \frac{\text{Pixels}}{f}$
3.  **计算度数 (人类可读)**:
    *   $\text{Angle(deg)} = \text{Angle(rad)} \times \frac{180}{\pi}$

**速查表 ($f=1000$ px)**:

| 像素误差 | 对应弧度 | 对应度数 |
| :--- | :--- | :--- |
| 10 px | 0.01 rad | 0.57° |
| 100 px | 0.10 rad | 5.7° |
| 500 px | 0.50 rad | 28.6° |
| 960 px (边缘) | 0.96 rad | 55.0° |
