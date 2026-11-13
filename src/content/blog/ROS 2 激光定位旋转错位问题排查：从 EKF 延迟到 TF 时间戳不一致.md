---
title: 'ROS 2 激光定位旋转错位问题排查：从 EKF 延迟到 TF 时间戳不一致'
excerpt: '机器人静止时，激光点云与已知地图基本能够贴合；但当机器人原地旋转时，激光与地图出现明显的角度偏移。机器人停止旋转后，这种偏移又逐渐减小。'
publishDate: '2026-11-13'
tags:
  - ROS2 Humble 
  - C++
  - TF2
  - robot_localization
  - Linux
---

## 前言

在移动机器人定位系统中，激光雷达、轮式里程计和 IMU 通常需要协同工作，为机器人提供准确、连续的位姿估计。

然而，在实际开发中，即使各个传感器的数据看起来正常，机器人仍然可能出现定位偏差。

近期，我在调试一台基于 ROS 2 Humble 的室内清洁机器人时，遇到了一个比较典型的问题：

**机器人静止时，激光点云与已知地图基本能够贴合；但当机器人原地旋转时，激光与地图出现明显的角度偏移。机器人停止旋转后，这种偏移又逐渐减小。**

最初怀疑是 IMU、EKF 融合或激光雷达运动畸变造成的。

经过多次实验，最终发现，问题的一个关键原因是：

**定位节点在处理历史激光扫描数据时，使用了最新的里程计 TF，而不是与激光扫描时间戳对应的 TF。**

本文记录完整的排查过程，并总结 ROS 2 TF 时间同步中的几个重要问题。

---

## 1. 系统架构与问题现象

### 1.1 机器人系统

机器人采用差速驱动结构，主要软硬件配置如下：

| 模块 | 配置 |
|---|---|
| 主控 | RK3576 |
| 操作系统 | Ubuntu |
| ROS 版本 | ROS 2 Humble |
| 底盘 | 差速驱动 |
| 激光雷达 | 二维单线激光雷达 |
| 惯性传感器 | IMU |
| 里程计 | CAN 总线反馈的轮速数据 |
| 状态估计 | robot_localization EKF |
| 定位 | 基于已知地图的激光扫描匹配 |
| 导航 | Nav2 |

系统中的主要 TF 结构为：

```text
map
 └── odom
      └── base_footprint
           └── base_link
                └── laser
```

其中：

- `map → odom`：由激光定位节点发布，用于修正全局定位误差。
- `odom → base_footprint`：最初由 EKF 发布，实验期间也尝试由原始轮式里程计直接发布。
- `base_footprint → base_link`：机器人本体坐标变换。
- `base_link → laser`：激光雷达安装外参。

主要 ROS 2 话题包括：

```text
/odom_raw      原始轮式里程计
/odom          EKF 融合后的里程计
/imu_data      IMU 数据
/raw_scan      原始激光扫描
/scan          去畸变后的激光扫描
/tf            动态坐标变换
/tf_static     静态坐标变换
```

### 1.2 问题现象

使用已知地图定位时，可以观察到：

**静止状态：**

机器人不运动，激光点云与地图墙体基本贴合，`map → odom` 的旋转修正较为稳定。

**旋转状态：**

机器人原地旋转时，激光点云相对地图出现角度偏移，同时 `map → odom` 的 Yaw 发生明显变化。

**停止状态：**

机器人停止旋转后，`map → odom` 的 Yaw 逐渐恢复到接近旋转前的数值。

这一现象具有明显的运动相关性，因此需要优先排查：

1. 轮式里程计与 IMU 的动态响应。
2. EKF 是否引入额外的滤波滞后。
3. 激光雷达运动畸变补偿是否准确。
4. 激光扫描与里程计 TF 的时间戳是否对应。

其中，第四点最终成为解决问题的关键。

---

## 2. 第一次排查：怀疑 EKF 存在动态延迟

### 2.1 比较原始里程计与 EKF 里程计

首先录制以下 ROS 2 话题：

```bash
ros2 bag record \
    /odom_raw \
    /odom \
    /imu_data \
    /raw_scan \
    /scan \
    /tf
```

通过 Python 读取 rosbag，提取两个话题中的角速度：

```cpp
odom_msg.twist.twist.angular.z
```

分别绘制：

$$
\omega_{\mathrm{raw}}(t)
$$

和：

$$
\omega_{\mathrm{EKF}}(t)
$$

结果发现，机器人开始原地旋转时：

- `/odom_raw` 的角速度迅速达到目标值。
- `/odom` 的角速度变化明显更加平缓。
- 停止旋转时，`/odom_raw` 很快恢复到零，而 `/odom` 存在一定的衰减过程。

这说明 EKF 角速度估计存在动态响应滞后。

这里需要区分两个概念。

#### 固定时间延迟

例如：

$$
\omega_{\mathrm{EKF}}(t)
=
\omega_{\mathrm{raw}}(t-\tau)
$$

这种情况下，两条信号主要表现为时间轴上的水平错位。

#### 滤波动态滞后

例如一阶低通系统：

$$
\tau\frac{d\omega_{\mathrm{EKF}}}{dt}
+
\omega_{\mathrm{EKF}}
=
\omega_{\mathrm{raw}}
$$

此时，角速度不会立即跟随输入变化，而是逐渐接近新的测量值。

实际曲线更加接近第二种情况，说明不能简单地将全部差异理解为通信延迟。

### 2.2 分析轮式里程计协方差

检查原始里程计发布代码后，发现其角速度测量方差设置为：

```cpp
odom_msg.twist.covariance[35] = 0.1;
```

对于 `nav_msgs/msg/Odometry`，Twist 协方差是一个 $6\times6$ 的矩阵，按照行优先方式存储。

其状态顺序为：

```text
[vx, vy, vz, wx, wy, wz]
```

因此：

```cpp
twist.covariance[35]
```

对应绕 Z 轴角速度的测量方差。

原始设置：

$$
R_{\omega}=0.1
$$

其标准差为：

$$
\sigma_\omega=\sqrt{0.1}
\approx0.316\ \mathrm{rad/s}
$$

对于实际旋转角速度约为：

$$
|\omega|\approx0.6\sim0.7\ \mathrm{rad/s}
$$

的机器人而言，这一测量不确定性可能偏大。

EKF 测量更新公式为：

$$
\hat{x}_k=
\hat{x}_{k|k-1}
+
K_k(z_k-H\hat{x}_{k|k-1})
$$

其中：

$$
K_k=
P_kH^T(HP_kH^T+R)^{-1}
$$

当测量协方差 \(R\) 较大时，滤波器通常会降低对对应测量的信任程度。

为验证这一因素，将角速度方差从：

```cpp
odom_msg.twist.covariance[35] = 0.1;
```

修改为：

```cpp
odom_msg.twist.covariance[35] = 0.01;
```

重新录制数据并绘图。

实验中观察到，EKF 角速度能够更快地跟随原始轮式角速度，启动和停止阶段的平滑滞后有所改善。

但是需要注意：

**减小协方差只能改变 EKF 对测量的信任程度，不能从根本上修复传感器时间戳不一致的问题。**

此外，协方差应根据实际噪声、编码器精度及轮胎滑移情况标定，不能仅以响应速度快慢确定最优值。

---

## 3. 第二次排查：绕过 EKF，直接使用原始轮式 TF

为了进一步确认 EKF 是否为主要原因，进行了第二组实验。

### 3.1 修改 TF 发布方式

原始系统中：

```text
/odom_raw
    ↓
EKF
    ↓
/odom
    ↓
odom → base_footprint
```

实验中改为：

```text
/odom_raw
    ↓
轮式里程计积分
    ↓
odom → base_footprint
```

也就是说，由原始轮式里程计直接发布 TF。

在里程计发布函数中增加：

```cpp
geometry_msgs::msg::TransformStamped odom_tf;

odom_tf.header = odom_msg.header;
odom_tf.child_frame_id = "base_footprint";

odom_tf.transform.translation.x = pose_x_;
odom_tf.transform.translation.y = pose_y_;
odom_tf.transform.translation.z = 0.0;

odom_tf.transform.rotation =
    odom_msg.pose.pose.orientation;

tf_broadcaster_->sendTransform(odom_tf);
```

同时关闭 EKF 的 TF 广播：

```yaml
publish_tf: false
```

避免多个节点同时发布相同的 `odom → base_footprint` 变换。

### 3.2 实验结果

原始轮式里程计直接发布 TF 后：

```text
ODOM -> base_footprint

stamp age    : 0.0100 s
receive age  : 0.0036 s
stamp period : 0.0199 s
```

TF 发布频率约为：

$$
f\approx50\ \mathrm{Hz}
$$

时间戳距离当前时刻大约 10 ms。

这说明原始轮式 TF 的更新频率和消息传输情况基本正常。

但是，**旋转时 `map → odom` 的动态角度修正仍然存在。**

实验日志中，旋转前：

```text
MAP -> ODOM
yaw : 0.3772 rad
```

旋转过程中：

```text
MAP -> ODOM
yaw : 0.5305 rad
```

旋转停止后：

```text
MAP -> ODOM
yaw : 0.3950 rad
```

以旋转前的姿态为参考，旋转过程中的变化约为：

$$
\Delta\theta
=
0.5305-0.3772
$$

$$
\Delta\theta
\approx0.1533\ \mathrm{rad}
$$

换算为角度：

$$
\Delta\theta\approx8.8^\circ
$$

可以看到，即使绕过 EKF，旋转过程中仍然存在明显的定位修正。

这意味着：

**EKF 的动态滤波滞后并不能单独解释全部问题。**

当然，`map → odom` 的变化同时可能包含扫描匹配修正、轮式里程计误差等因素，因此不能直接将 8.8° 全部解释为时间同步误差。

这一实验使排查重点逐渐转向定位节点内部的 TF 使用方式。

---

## 4. 第三次排查：检查激光与 TF 的时间关系

### 4.1 分析激光扫描的时间戳

在 ROS 2 中，每条带有 `std_msgs/Header` 的消息都可以记录时间戳。

例如：

```cpp
scan->header.stamp
```

表示激光扫描消息所对应的时间。

根据 `sensor_msgs/msg/LaserScan` 的消息约定，`header.stamp` 通常表示**第一束激光的采样时间**。

对于一帧扫描：

```text
scan_time ≈ 0.1 s
```

不同激光束实际采集于不同时间。

因此，在机器人运动过程中，如果直接将整帧扫描视为同一时刻采集，就可能产生运动畸变。

去畸变算法通常需要使用里程计或 IMU 提供的运动信息，将不同采样时刻的激光点统一到指定参考时刻。

### 4.2 对比激光与最新 TF

实验日志中：

```text
LASER SCAN
stamp age : 0.1804 s
```

而：

```text
ODOM -> base_footprint
stamp age : 0.0107 s
```

两者时间戳相对于当前时间的差异约为：

$$
\Delta t
=
0.1804-0.0107
$$

$$
\Delta t
\approx0.1697\ \mathrm{s}
$$

也就是约 170 ms。

假设机器人以：

$$
\omega=-0.643\ \mathrm{rad/s}
$$

旋转，则这段时间内的航向变化约为：

$$
\Delta\theta=\omega\Delta t
$$

$$
\Delta\theta
=-0.643\times0.1697
$$

$$
\Delta\theta\approx-0.109\ \mathrm{rad}
$$

约为：

$$
\Delta\theta\approx-6.3^\circ
$$

这是一个不容忽视的角度差。

需要强调的是，这里的 170 ms 是**激光消息时间戳与最新 TF 时间戳之间的差值**，并不等于已经测量出了 170 ms 的物理传感器延迟。

它说明：如果算法错误地把最新 TF 当作历史激光扫描的位姿，就可能引入明显的动态姿态偏差。

---

## 5. 关键问题：定位节点使用了最新 TF

进一步检查定位节点的源码，发现以下代码。

```cpp
std::vector<scan_point_t>
LocalizationOptimize::convert_scan(
    const sensor_msgs::msg::LaserScan::SharedPtr scan,
    const Matrix<double, 4, 4> &odom_to_base)
{
    std::vector<scan_point_t> points;

    tf2::Stamped<tf2::Transform> base_to_odom;
    tf2::Stamped<tf2::Transform> sensor_to_base;

    try
    {
        auto tempTransform = buffer->lookupTransform(
            m_base_frame,
            scan->header.frame_id,
            tf2::TimePointZero);

        tf2::fromMsg(tempTransform, sensor_to_base);
    }
    catch (const std::exception &ex)
    {
        return points;
    }

    try
    {
        auto tempTransform = buffer->lookupTransform(
            m_odom_frame,
            m_base_frame,
            tf2::TimePointZero);

        tf2::fromMsg(tempTransform, base_to_odom);
    }
    catch (const std::exception &ex)
    {
        return points;
    }

    const Matrix<double, 4, 4> S =
        convert_transform_3(sensor_to_base);

    const Matrix<double, 4, 4> L =
        convert_transform_25(base_to_odom);

    const Matrix<double, 4, 4> T =
        odom_to_base * L * S;

    // 后续将激光点转换至目标参考坐标系
}
```

问题集中在：

```cpp
buffer->lookupTransform(
    m_odom_frame,
    m_base_frame,
    tf2::TimePointZero);
```

### 5.1 TimePointZero 的含义

在 TF2 中：

```cpp
tf2::TimePointZero
```

并不表示当前系统时间，也不表示当前处理消息的时间戳。

它表示：

**查询 TF Buffer 中最新可用的变换。**

例如：

```text
激光时间戳：100.050 s

最新 TF 时间戳：100.180 s
```

执行：

```cpp
lookupTransform(
    "odom",
    "base_footprint",
    tf2::TimePointZero);
```

通常得到最新可用时间对应的机器人位姿：

$$
{}^{odom}T_{base}(100.180)
$$

而不是：

$$
{}^{odom}T_{base}(100.050)
$$

当机器人静止时，两者几乎相同，因此问题并不明显。

但在旋转时，两者存在明显的姿态差。

### 5.2 正确的时间查询方式

如果算法需要的是激光扫描时刻的机器人姿态，应使用：

```cpp
const rclcpp::Time scan_time(scan->header.stamp);

auto tempTransform = buffer->lookupTransform(
    m_odom_frame,
    m_base_frame,
    scan_time,
    rclcpp::Duration::from_seconds(0.1));
```

其中：

```cpp
scan_time
```

指定要查询的 TF 时间。

```cpp
rclcpp::Duration::from_seconds(0.1)
```

表示 TF 查询最多等待 100 ms。

**这里的 0.1 秒是查询超时时间，不是时间补偿量。**

它不会自动将激光时间戳向前或向后移动 100 ms。

如果 TF Buffer 中存在请求时刻前后的有效变换，TF2 可以进行时间插值。

例如：

```text
100.040 s：odom → base_footprint

100.060 s：odom → base_footprint
```

请求：

```text
100.050 s
```

TF2 可以根据相邻变换计算对应时刻的机器人姿态。

---

## 6. 从坐标变换角度理解问题

在定位节点中，最终使用：

```cpp
const Matrix<double, 4, 4> T =
    odom_to_base * L * S;
```

假设：

$$
S={} ^{base}T_{laser}
$$

表示激光坐标系到机器人坐标系的外参。

$$
L={} ^{odom}T_{base}(t)
$$

表示机器人在里程计坐标系中的姿态。

如果外部传入：

$$
odom\_to\_base=
{}^{base(t_{ref})}T_{odom}
$$

那么最终变换为：

$$
T=
{}^{base(t_{ref})}T_{odom}
\cdot
{}^{odom}T_{base}(t)
\cdot
{}^{base}T_{laser}
$$

这相当于将扫描点变换到某个参考时刻的机器人坐标系。

这里有一个关键要求：

**各个矩阵必须使用符合算法预期的坐标系方向和时间基准。**

如果 \(L\) 使用最新时间：

$$
t=t_{\mathrm{latest}}
$$

而实际处理的是：

$$
t=t_{\mathrm{scan}}
$$

就可能计算出错误的相对运动。

正确做法不是简单地在所有地方都使用 `TimePointZero`，而是明确：

- 当前扫描对应什么时间？
- 激光点最终需要转换到哪个坐标系？
- 参考位姿对应什么时间？
- 运动补偿使用的是扫描开始、扫描结束还是其他参考时刻？

对于已经完成运动去畸变的 `/scan`，还必须确认去畸变输出实际采用的参考时刻。

否则即使使用 `scan->header.stamp`，仍可能因为消息时间戳与去畸变参考时刻不一致而产生误差。

---

## 7. 修复方法与实验结果

在确认定位节点所需的变换时间后，修改其里程计 TF 查询。

原代码：

```cpp
auto tempTransform = buffer->lookupTransform(
    m_odom_frame,
    m_base_frame,
    tf2::TimePointZero);
```

修改为：

```cpp
const rclcpp::Time scan_time(scan->header.stamp);

auto tempTransform = buffer->lookupTransform(
    m_odom_frame,
    m_base_frame,
    scan_time,
    rclcpp::Duration::from_seconds(0.1));
```

其他矩阵运算保持与算法要求一致。

重新运行定位系统，进行原地旋转测试。

**修复后，之前机器人旋转时 `map → odom` 出现较大角度变化的问题得到解决。**

这说明，在本次系统中，定位节点使用的 TF 时间与激光扫描时间不一致，是导致动态定位修正异常的重要因素。

不过，本次实验并没有单独量化修复后的角度误差，也没有完全排除其他因素。

因此更准确的结论是：

**修复扫描时刻的 TF 查询后，原先明显的动态角度修正现象消失或显著减小，验证了这一修改对当前问题的有效性。**

---

## 8. 几个容易混淆的 TF2 接口

### 8.1 查询最新可用变换

```cpp
auto tf = buffer->lookupTransform(
    "odom",
    "base_footprint",
    tf2::TimePointZero);
```

含义：

获取最新可用的 `odom → base_footprint` 变换。

适用于确实需要最新位姿的场景。

### 8.2 查询指定时刻的变换

```cpp
auto tf = buffer->lookupTransform(
    "odom",
    "base_footprint",
    rclcpp::Time(scan->header.stamp));
```

含义：

获取激光扫描时间戳对应的机器人位姿。

适用于需要与历史传感器数据进行时间对齐的场景。

### 8.3 查询指定时刻并设置超时

```cpp
auto tf = buffer->lookupTransform(
    "odom",
    "base_footprint",
    rclcpp::Time(scan->header.stamp),
    rclcpp::Duration::from_seconds(0.1));
```

含义：

查询扫描时间对应的 TF，最多等待 100 ms。

### 8.4 不要混淆时间戳与时间间隔

例如：

```cpp
tf2::durationFromSec(0.1)
```

得到的是一个时间间隔。

它通常用于指定查询超时，而不是表示一个绝对时间戳。

另外，多个 `lookupTransform()` 调用相互独立。

即使前一次查询：

```cpp
lookupTransform(
    "base_footprint",
    "laser",
    tf2::TimePointZero);
```

后一次查询也不会自动使用激光扫描的 `header.stamp`。

**TF2 不会根据查询涉及的坐标系，自动推断应该使用哪个传感器消息时间戳。**

---

## 9. 工程实践中的几个重要经验

### 9.1 TF 发布频率高，不代表时间同步正确

本次实验中，原始轮式里程计 TF 已经达到约 50 Hz。

但定位节点仍然因为查询时间不一致而出现问题。

因此，检查 TF 时不能只关注：

```bash
ros2 topic hz /tf
```

还需要关注：

- TF 的实际时间戳。
- 传感器数据的时间戳。
- TF 查询时指定的时间。
- 数据进入算法时已经经历了多少处理延迟。

### 9.2 消息传输延迟不等于测量延迟

例如：

```text
/odom_raw receive age ≈ 3 ms
```

只能说明消息进入监控程序的时间较近。

如果轮速数据来自 CAN 总线，而消息时间戳是在 ROS 节点处理时生成的，那么它可能仍然包含电机采样、CAN 传输和线程调度造成的时间误差。

因此，不能简单根据 `header.stamp` 与当前时间的差值，判断传感器实际采样延迟。

### 9.3 EKF 平滑滞后与 TF 时间错配是不同问题

EKF 可能因为测量噪声协方差、过程噪声协方差等原因，对角速度变化产生平滑响应。

但这与定位节点使用错误时刻的 TF 是两个不同问题。

前者属于状态估计器的动态特性。

后者属于时间与坐标变换的一致性。

**不能试图通过不断减小 EKF 协方差，修复定位算法本身的 TF 时间查询错误。**

### 9.4 `map → odom` 的动态变化可以作为诊断线索

当机器人原地旋转时，如果：

```text
map → odom
```

的 Yaw 发生明显变化，而停止旋转后又恢复，那么值得检查：

- 里程计运动模型是否准确。
- 激光是否进行了正确的运动补偿。
- 激光时间戳是否真实对应采样时间。
- 定位节点是否使用了正确时刻的 TF。
- 是否混用了不同来源的里程计位姿。

不过，`map → odom` 不要求绝对静止。定位算法本来就会通过它修正里程计误差，因此应结合地图匹配质量判断变化是否异常。

### 9.5 运动去畸变与定位时间对齐需要分别检查

即使激光已经进行运动去畸变，也不意味着定位节点的所有 TF 查询都已经正确。

可以将问题分为两个层次：

**第一层：扫描内部的时间对齐。**

不同激光束采集于不同时间，需要根据运动轨迹将其变换到统一参考时刻。

**第二层：整帧扫描与定位位姿的时间对齐。**

定位算法使用整帧扫描时，需要查询与扫描参考时刻一致的机器人位姿。

两层处理都正确，才能保证旋转过程中激光点云与地图具有良好的时间一致性。

---

## 10. 总结

本次 ROS 2 激光定位问题的排查过程如下：

```text
机器人旋转时激光与地图错位
              |
              v
      怀疑 IMU / EKF 延迟
              |
              v
   对比 odom_raw 与 odom 角速度
              |
              v
       发现 EKF 动态滞后
              |
              v
       调整角速度测量协方差
              |
              v
       EKF 响应得到改善
              |
              v
    直接使用原始轮式里程计发布 TF
              |
              v
      map→odom 动态修正仍存在
              |
              v
      检查激光与 TF 时间戳
              |
              v
    发现定位节点查询最新可用 TF
              |
              v
      改为查询扫描时刻的 TF
              |
              v
     旋转时较大角度修正问题解决
```

整个过程中，最重要的认识是：

**机器人定位不仅要求坐标变换关系正确，还要求参与计算的各个坐标变换在时间上符合算法预期。**

对于处于静止状态的机器人，使用最新 TF 和历史 TF 可能没有明显区别。

但是在机器人运动时，尤其是原地旋转时，即使只有一百多毫秒的时间差，也可能带来数度的航向偏差。

因此，在 ROS 2 激光定位、运动去畸变和多传感器融合系统中，必须同时关注三个方面：

**坐标系正确、时间戳准确、TF 查询时刻一致。**

这次问题也提醒我，在排查定位误差时，不应一开始就将全部注意力集中在滤波器参数或传感器标定上。

有时真正影响定位精度的，只是代码中一个看似普通的参数：

```cpp
tf2::TimePointZero
```

**当算法需要的是历史传感器时刻的位姿时，最新可用的 TF 并不一定是正确的 TF。**

---
