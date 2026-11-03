---
title: '弓字形路径异常排查：障碍物轮廓方向如何影响 Cell 分解'
excerpt: '从反转障碍物轮廓后恢复正常的案例出发，梳理有向面积、CEILING/FLOOR 事件分类、扫描线排序与 Cell 边界校验之间的关系。'
publishDate: '2026-11-03'
tags:
  - 机器人
  - 路径规划
  - OpenCV
  - C++
---

弓字形覆盖路径出现大量异常点时，问题可能来自更早的区域分解环节。本次 Q5 排障中，用户反馈将 `reverse_contour` 改为 `true` 后恢复正常。这提示我们：输入轮廓的遍历方向，是事件分类器必须明确约定的一部分。

本文根据一次实际排障对话整理。对话中的修复反馈属于案例证据；此前正常地图与补扫地图的轮廓方向为何不同，以及异常路径是否确实由越界导致，仍需要源码、日志或运行检查进一步验证。

## 1. 先区分两种“排序”

| 概念 | 处理对象 | 作用 |
| --- | --- | --- |
| 轮廓遍历顺序 | 一个障碍物的边界点 | 决定沿边界前进的方向，以及相邻点关系 |
| 扫描线事件排序 | 已完成分类的事件 | 按 X、Y 等规则组织扫描切片 |

在本案例的处理流程中，事件分类发生在扫描线排序之前：

```text
ExtractSupplementContours()
    → ConstructObstacles()
    → InitializeEventList()
    → AllocateObstacleEventType()
    → SliceListGenerator()
    → ExecuteCellDecomposition()
    → BoustrophedonCurvePath()
```

因此，即使扫描线坐标顺序正确，也不能保证事件类型正确。对已经分错类型的事件重新排序，无法修复上游语义。

## 2. 反转轮廓改变了什么？

对一个按边界顺序存储的闭合多边形：

```text
原顺序：A → B → C → D → A
反转后：D → C → B → A → D
```

执行：

```cpp
std::reverse(contour.begin(), contour.end());
```

点的坐标集合、形状和无向面积保持不变，遍历方向、有向面积符号以及起点位置会改变。以上结论针对闭合轮廓；如果代码把该容器当成开放折线，或者依赖固定起点，则还需要额外处理。

有向面积可写为：

```text
A = 1/2 × Σ(x_i × y_(i+1) - x_(i+1) × y_i)
```

OpenCV 可以直接计算：

```cpp
double signed_area = cv::contourArea(contour, true);
```

第二个参数为 `true` 时保留方向符号。图像坐标通常是 X 向右、Y 向下，此时上述公式的正面积对应视觉上的顺时针；若换成 Y 向上的坐标系，同一组数值所呈现的视觉方向会不同。

不要只记录“顺时针”三个字，应同时记录坐标系和面积符号。自交轮廓的代数面积也不能直接当作有效障碍物面积。参见 [OpenCV 轮廓面积文档](https://docs.opencv.org/4.13.0/d3/dc0/group__imgproc__shape.html)。

## 3. 为什么会影响 CEILING/FLOOR？

本案例讨论的分类规则包含：

```cpp
// 沿轮廓从 OUT 走向 IN 的区段
event_list[j].event_type = FLOOR;

// 沿轮廓从 IN 走向 OUT 的区段
event_list[j].event_type = CEILING;
```

它依赖沿轮廓遍历的前后关系。反转方向后，同一区段被访问的方向改变，分类结果就可能交换。这是当前实现的输入约束，并非所有弓字形分解算法都必须依赖某一种轮廓方向。

在案例使用的图像坐标和命名约定下，某条普通扫描线自上而下的事件应表示两个自由区间：

```text
CEILING (220, 3)    外墙上边界
FLOOR   (220, 85)   障碍物上边界
CEILING (220, 104)  障碍物下边界
FLOOR   (220, 297)  外墙下边界
```

障碍物顶部是其上方自由空间的 FLOOR；障碍物底部是其下方自由空间的 CEILING。名称描述的是自由空间的边界。

异常排列曾表现为：

```text
CEILING → CEILING → FLOOR → FLOOR
```

恢复后的普通切片为：

```text
CEILING → FLOOR → CEILING → FLOOR
```

这个检查适用于示例中的普通切片。穿过顶点、竖直边或分裂合并事件的临界切片，需要按实现的拓扑规则单独处理。

## 4. 修复应统一方向

无条件反转适合定位问题。正式处理应检查输入是否符合分类器约定，只在不符合时反转，并在一个入口完成。

对话记录了：

```text
[OBSTACLE CONTOUR] index=0 points=46 signed_area=180
```

**只有当 +180 是反转之前的日志，且反转后分类恢复正常时，才能据此选择负面积作为当前分类器的目标方向。** 如果日志是在反转之后打印，结论就不同。发布这篇整理时，这个日志位置尚未得到进一步确认。

下面提供可配置的方向归一化示例，不把目标符号写成通用规则：

```cpp
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <vector>

bool NormalizeObstacleContour(
    std::vector<cv::Point>& contour,
    bool expected_positive,
    double min_abs_area)
{
    if (contour.size() < 3 || min_abs_area < 0.0) {
        return false;
    }

    const double area = cv::contourArea(contour, true);
    if (!std::isfinite(area) ||
        area == 0.0 ||
        std::abs(area) < min_abs_area) {
        return false;
    }

    if ((area > 0.0) != expected_positive) {
        std::reverse(contour.begin(), contour.end());
    }
    return true;
}
```

调用方根据验证结果配置 `expected_positive`，并根据地图分辨率确定最小面积阈值。返回失败时应明确拒绝或过滤该轮廓。这个函数只检查面积和方向，不验证自交等完整几何有效性。

可以在 `ExtractSupplementContours()` 输出障碍物前，或 `ConstructObstacles()` 转换数据时调用。归一化之后，应移除后续无条件反转，避免重复修改。外墙和障碍物可能使用不同分类规则，应分别定义方向约定。

## 5. 轮廓层级与遍历方向要分别检查

轮廓层级说明谁包含谁，面积符号说明如何遍历，两者承担不同职责。

OpenCV 的 `RETR_CCOMP` 生成两级层级，孔洞内部的前景岛会重新出现在顶层；`RETR_TREE` 保留完整的嵌套关系。需要支持多级区域时，应结合二值掩膜语义和层级深度处理，不能仅凭“有父轮廓”就把所有情况都解释成同一类物理障碍物。参见 [OpenCV 轮廓检索模式文档](https://docs.opencv.org/4.13.0/d3/dc0/group__imgproc__shape.html)。

此前地图正常、补扫地图异常，可能与输入掩膜、轮廓来源或后续反转处理有关。仅凭本次面积日志，无法确定具体是哪一环节造成差异。

## 6. 从事件、Cell 到路径逐层验证

本次对话提到异常 Cell：

```text
ceiling.size() = 23
floor.size()   = 12
```

数量不一致是排查线索。只有在实现要求一一配对采样时，才能直接判为无效；即使数量相等，也仍需检查对应 X、上下边界关系、自由空间归属以及分裂合并事件。

建议依次保留这些检查：

1. 在分类器入口记录轮廓编号、点数、归一化前后面积。
2. 输出普通扫描线事件，检查自由区间是否正确。
3. 检查 Cell 边界的非空性、配对规则和几何有效性。
4. 在路径生成前拒绝无效 Cell，并检查数组访问范围。
5. 用原异常地图、已有正常地图、多障碍物及退化轮廓验证修改。

不要通过把循环长度截断到两个边界容器的较小值来掩盖配对错误；那可能消除越界，却生成错误覆盖路径。

## 7. 绘图也可能制造“异常路径”

把所有 CEILING 点连成一条线，会连接不同 Cell 的边界，产生跨越自由空间或障碍物的斜线。

排查事件位置时，先画散点：

```python
plt.scatter(ceiling_x, ceiling_y, c="cyan", label="CEILING")
plt.scatter(floor_x, floor_y, c="orange", label="FLOOR")
plt.gca().invert_yaxis()
plt.legend()
plt.grid(True)
plt.show()
```

需要画边界线时，应按 Cell 和边界归属分组，再按各自的几何顺序连接。调试图出现斜线，不能单独证明实际规划路径穿过障碍物。

本次案例最有价值的经验，是把轮廓方向从隐含假设变为明确的数据约定：入口统一方向，分类后检查事件语义，规划前校验 Cell。这样才能把一次有效的反转实验，转化为可维护的修复。

---

资料说明：本文根据用户提供的 Q5 路径异常排障对话整理；通用 API 行为对照 OpenCV 官方文档。示例代码用于说明处理方式，尚未在 Q5 工程中编译或运行。
