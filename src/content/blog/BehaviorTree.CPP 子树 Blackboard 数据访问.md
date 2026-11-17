---
title: 'ROS 2 Humble：BehaviorTree.CPP 子树 Blackboard 数据访问异常分析与解决'
excerpt: '通过行为树，可以将机器人导航、沿边清扫、区域覆盖、异常恢复等功能组织成不同的子任务，并通过 Sequence、Fallback、RecoveryNode 等控制节点实现任务调度。'
publishDate: '2026-11-17'
tags:
  - Interview
  - CPP
  - Linux
---

## 1. 前言

在基于 ROS 2 Humble 和 Nav2 开发清洁机器人导航系统时，行为树（Behavior Tree，BT）是一种常用的任务组织方式。

通过行为树，可以将机器人导航、沿边清扫、区域覆盖、异常恢复等功能组织成不同的子任务，并通过 `Sequence`、`Fallback`、`RecoveryNode` 等控制节点实现任务调度。

随着系统功能逐渐复杂，将异常恢复逻辑封装成独立子树是一种比较合理的工程设计。

例如，机器人执行沿边清扫时，如果发生卡住、碰撞或局部路径执行失败，可以通过 `RecoveryNode` 调用对应的恢复子树。

然而，在实际开发过程中，遇到了一个比较典型的问题：

**自定义 BT 节点在主树中能够正常获取 Blackboard 中的 ROS Node，但放入 SubTree 后，却出现 `Missing key [node]` 异常。**

本文将结合实际代码和调试过程，分析 BehaviorTree.CPP 3.x 的 Blackboard 作用域机制及其解决方法。

### 开发环境

| 项目 | 配置 |
|---|---|
| 操作系统 | Ubuntu 22.04 |
| ROS 版本 | ROS 2 Humble |
| 导航框架 | Nav2（自定义扩展） |
| 行为树框架 | BehaviorTree.CPP 3.x |
| 编程语言 | C++17 |
| 应用场景 | 室内清洁机器人 |
| 任务类型 | 沿边清扫与异常恢复 |

---

## 2. 问题背景

### 2.1 行为树整体架构

在清洁机器人系统中，沿边任务使用 `WalkControl` 控制器执行。

当控制器检测到异常时，通过 `RecoveryNode` 调用恢复子树。

整体结构如下：

```text
MainTree
    |
    +-- Sequence
          |
          +-- LocateStable
          |
          +-- SegmentRegion
          |
          +-- GetCurrentRegion
          |
          +-- ReactiveSequence
                |
                +-- LocateScoreCondition
                |
                +-- RecoveryNode
                      |
                      +-- WalkControl
                      |     |
                      |     +-- 沿边清扫
                      |
                      +-- WallFollowRecoveryTree
                            |
                            +-- CheckBlackboardInt
                            |
                            +-- CheckLookAhead
                            |
                            +-- Fallback
                                  |
                                  +-- DriveOnHeading
                                  +-- TurnCircle
                                  +-- TurnBack
```

为了提高代码复用性，将异常恢复逻辑单独封装在：

```text
behavior_trees/
    |
    +-- main/
    |     |
    |     +-- segmentation_region_test.xml
    |
    +-- recovery/
          |
          +-- wall_follow_recovery.xml
```

主树通过 XML 的 `include` 机制引入恢复子树。

### 2.2 主树 XML

主树中使用以下代码：

```xml
<root main_tree_to_execute="MainTree">

    <include
        ros_pkg="clean_bt_nav"
        path="behavior_trees/recovery/wall_follow_recovery.xml" />

    <BehaviorTree ID="MainTree">

        <ReactiveSequence name="WallFollowWithRegionCheck">

            <LocateScoreCondition score_th="0.20" />

            <RecoveryNode
                name="WallFollowRecovery"
                number_of_retries="5">

                <Action
                    ID="WalkControl"
                    path="{path}"
                    controller_id="OneKeyWallFollow"
                    goal_checker_id="wall_goal_checker"
                    wall_follow_path="{wall_follow_path}"
                    clean_path="true" />

                <SubTree
                    ID="WallFollowRecoveryTree"
                    node="{node}"
                    server_timeout="{server_timeout}"
                    bt_loop_duration="{bt_loop_duration}"
                    walk_control_error_code="{walk_control_error_code}"
                    escape_direction="{escape_direction}" />

            </RecoveryNode>

        </ReactiveSequence>

    </BehaviorTree>

</root>
```

其中：

```xml
<include
    ros_pkg="clean_bt_nav"
    path="behavior_trees/recovery/wall_follow_recovery.xml" />
```

用于引入恢复子树定义。

而：

```xml
<SubTree ID="WallFollowRecoveryTree" />
```

用于在主树中实例化该子树。

需要注意：

**`include` 和 `SubTree` 并不是同一个概念。**

`include` 负责加载 XML 定义，`SubTree` 负责创建对应的行为树实例。

---

## 3. 问题现象

执行清扫任务时，行为树加载失败。

关键错误日志如下：

```text
[ERROR] [bt_navigator]:
Exception when loading BT:
Blackboard::get() error. Missing key [node]
```

与此同时，系统还出现多个行为树节点的析构日志：

```text
Shutting down GetRegionSegmentation BT node

Shutting down GetCurrentRegion BT node

Shutting down SendBoundaryService BT node

Shutting down CheckCurrentRegionId BT node

Shutting down LocateScoreCondition BT node
```

最初怀疑是：

- XML 文件路径不正确；
- 行为树插件没有注册；
- 恢复子树没有被加载；
- 自定义 BT 节点构造函数存在问题。

但是进一步分析发现，系统已经能够成功创建部分行为树节点。

例如：

```text
WalkControlAction constructed.
```

这意味着 XML 已经进入解析和节点实例化过程。

因此，问题不是简单的 XML 文件不存在，而是行为树节点初始化过程中发生了异常。

---

## 4. 定位异常节点

### 4.1 自定义 CheckLookAheadCondition

恢复子树中使用了一个自定义节点：

```xml
<CheckLookAhead
    escape_direction="{escape_direction}" />
```

该节点主要用于根据超声波和激光雷达数据，判断机器人应该选择哪个方向进行脱困。

例如：

```text
0：短距离前进
1：正常前进
2：右转后前进
3：左转后前进
4：后退
```

其构造函数中需要获取 ROS Node：

```cpp
CheckLookAheadCondition::CheckLookAheadCondition(
    const std::string &condition_name,
    const BT::NodeConfiguration &conf)
    : BT::ConditionNode(condition_name, conf)
{
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

    callback_group_ = node_->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive,
        false);

    callback_group_executor_.add_callback_group(
        callback_group_,
        node_->get_node_base_interface());

    // 创建传感器订阅
}
```

其中关键代码是：

```cpp
node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
```

它直接从 Blackboard 中获取 ROS Node 指针。

正常情况下，这种方式在 Nav2 自定义 BT 节点中是可行的。

### 4.2 增加 Blackboard 调试日志

为了定位问题，在构造函数中增加以下代码：

```cpp
auto bb = config().blackboard;

RCLCPP_INFO(
    rclcpp::get_logger("BT_DEBUG"),
    "CheckLookAhead Blackboard keys:");

for (const auto &key : bb->getKeys())
{
    RCLCPP_INFO(
        rclcpp::get_logger("BT_DEBUG"),
        "Key: %.*s",
        static_cast<int>(key.size()),
        key.data());
}

rclcpp::Node::SharedPtr bt_node;

if (!bb->get("node", bt_node))
{
    RCLCPP_ERROR(
        rclcpp::get_logger("BT_DEBUG"),
        "can't get blackboard node!!!");
}
```

运行后得到：

```text
[CheckLookAhead]:
CheckLookAhead constructor begin

[BT_DEBUG]:
CheckLookAhead Blackboard keys:

[BT_DEBUG]:
Key: escape_direction

[BT_DEBUG]:
can't get blackboard node!!!
```

通过日志可以发现：

1. `CheckLookAhead` 已经成功进入构造函数；
2. 当前 Blackboard 的本地键中存在 `escape_direction`；
3. 无法通过当前 Blackboard 获取 `node`。

需要说明的是，`getKeys()` 通常只显示当前 Blackboard 的本地键，不一定显示父级 Blackboard 中的全部变量或重映射关系。

因此，单纯看到 `escape_direction` 并不足以证明 `node` 没有被映射。

真正有效的证据是：

```cpp
bb->get("node", bt_node)
```

返回了 `false`。

---

## 5. 进一步排查：主树正常，子树异常

为了确认是否与 `CheckLookAheadCondition` 节点本身有关，进行了一个对比实验。

### 实验一：将节点放在主树

```xml
<BehaviorTree ID="MainTree">

    <Sequence>

        <CheckLookAhead
            escape_direction="{escape_direction}" />

    </Sequence>

</BehaviorTree>
```

结果：

```text
CheckLookAhead Get node success
```

节点可以正常获取 ROS Node。

### 实验二：将节点放在 SubTree

```xml
<SubTree
    ID="WallFollowRecoveryTree"
    node="{node}"
    escape_direction="{escape_direction}" />
```

子树内部：

```xml
<BehaviorTree ID="WallFollowRecoveryTree">

    <Sequence>

        <CheckLookAhead
            escape_direction="{escape_direction}" />

    </Sequence>

</BehaviorTree>
```

结果：

```text
can't get blackboard node!!!
```

两个实验使用的是相同的自定义 BT 节点。

唯一的重要区别是：**节点所在的 Blackboard 作用域不同。**

因此，问题范围进一步缩小到 BehaviorTree.CPP 的子树 Blackboard 管理机制。

---

## 6. Blackboard 的作用域机制

### 6.1 什么是 Blackboard？

Blackboard 可以理解为行为树节点之间共享数据的存储区域。

例如：

```cpp
blackboard->set<int>("error_code", 14);
```

其他节点可以通过：

```cpp
int error_code = 0;

blackboard->get("error_code", error_code);
```

读取数据。

Nav2 也会通过 Blackboard 为 BT 节点提供 ROS 运行环境。

常见键包括：

```text
node
tf_buffer
server_timeout
bt_loop_duration
number_recoveries
```

其中：

```cpp
node
```

通常用于创建 ROS 订阅、Service Client、Action Client 等对象。

### 6.2 普通 SubTree 的 Blackboard

在 BehaviorTree.CPP 3.x 中，子树可以使用独立 Blackboard。

假设主树的 Blackboard 为：

```text
Root Blackboard
    |
    +-- node
    +-- tf_buffer
    +-- server_timeout
    +-- bt_loop_duration
    +-- walk_control_error_code
```

子树拥有独立的 Blackboard：

```text
SubTree Blackboard
    |
    +-- escape_direction
```

这两个 Blackboard 并不是同一个数据容器。

其结构可以理解为：

```text
BehaviorTreeEngine
        |
        v
Root Blackboard
        |
        +-- LocateStable
        |
        +-- WalkControl
        |
        +-- SubTree
              |
              v
         Child Blackboard
              |
              +-- CheckLookAhead
              |
              +-- TurnCircle
              |
              +-- TurnBack
```

主树中的节点能够读取：

```cpp
config().blackboard->get<rclcpp::Node::SharedPtr>("node");
```

但子树中的节点并不一定能够直接读取。

是否可以访问父级数据，取决于 Blackboard 的共享方式以及端口映射配置。

### 6.3 为什么 `node="{node}"` 没能解决？

最初尝试通过以下方式传递：

```xml
<SubTree
    ID="WallFollowRecoveryTree"
    node="{node}"
    server_timeout="{server_timeout}"
    bt_loop_duration="{bt_loop_duration}"
    escape_direction="{escape_direction}" />
```

这里的：

```xml
node="{node}"
```

表示尝试将父 Blackboard 的 `node` 映射到子树。

但这种配置与共享 Blackboard 并不等价。

在本项目使用的 BehaviorTree.CPP 3.x 版本中，实际运行时未能通过这一配置让构造函数成功读取 `node`。

随后尝试了：

```xml
<SubTreePlus
    ID="WallFollowRecoveryTree"
    node="{node}"
    escape_direction="{escape_direction}" />
```

依然没有成功。

这说明当前工程中，显式映射方案没有达到预期效果。

不过，不能仅凭这些现象断定 BehaviorTree.CPP 所有 3.x 版本都无法正确映射 `node`。

具体原因可能与所使用的版本、XML 解析实现、子树构造过程以及映射建立时机有关，需要结合对应版本源码进一步确定。

---

## 7. 最终解决方案：共享 Blackboard

最终将主树中的：

```xml
<SubTree
    ID="WallFollowRecoveryTree"
    node="{node}"
    server_timeout="{server_timeout}"
    bt_loop_duration="{bt_loop_duration}"
    walk_control_error_code="{walk_control_error_code}"
    escape_direction="{escape_direction}" />
```

修改为：

```xml
<SubTree
    ID="WallFollowRecoveryTree"
    __shared_blackboard="true" />
```

问题成功解决。

### 7.1 `__shared_blackboard` 的作用

在本项目使用的 BehaviorTree.CPP 3.x 实现中：

```xml
__shared_blackboard="true"
```

用于让子树与父树共享 Blackboard。

原来的结构：

```text
Root Blackboard
      |
      +-- node
      |
      +-- SubTree
             |
             v
       Child Blackboard
             |
             +-- CheckLookAhead
                    |
                    +-- get("node") 失败
```

修改后：

```text
             Root Blackboard
                    |
             +------+------+
             |             |
             v             v
         WalkControl   WallFollowRecoveryTree
             |             |
             v             v
         get("node")   CheckLookAhead
             |             |
           成功         get("node")
                           |
                          成功
```

共享模式下，子树节点可以直接访问主树 Blackboard 中的数据。

因此不再需要逐个映射：

```xml
node="{node}"
server_timeout="{server_timeout}"
bt_loop_duration="{bt_loop_duration}"
```

### 7.2 修改后的主树

最终采用：

```xml
<ReactiveSequence name="WallFollowWithRegionCheck">

    <LocateScoreCondition score_th="0.20" />

    <RecoveryNode
        name="WallFollowRecovery"
        number_of_retries="5">

        <Action
            ID="WalkControl"
            path="{path}"
            controller_id="OneKeyWallFollow"
            goal_checker_id="wall_goal_checker"
            wall_follow_path="{wall_follow_path}"
            clean_path="true" />

        <SubTree
            ID="WallFollowRecoveryTree"
            __shared_blackboard="true" />

    </RecoveryNode>

</ReactiveSequence>
```

恢复子树内部的 `CheckLookAhead` 不需要修改 Blackboard 访问接口：

```cpp
node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
```

即可正常访问 ROS Node。

---

## 8. 另一个问题：为什么还出现段错误？

排查过程中还出现了：

```text
process has died
exit code -11
```

Linux 中，退出码 `-11` 通常对应 `SIGSEGV`，即段错误。

结合当时的调试代码：

```cpp
if (!bb->get("node", node_))
{
    RCLCPP_ERROR(
        rclcpp::get_logger("BT_DEBUG"),
        "can't get blackboard node!!!");
}

callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false);
```

这里存在明显风险。

当 Blackboard 无法读取 `node` 时，`node_` 可能为空。

但程序仍继续执行：

```cpp
node_->create_callback_group(...)
```

这就可能导致空指针解引用，从而触发段错误。

因此，对于构造函数中的 Blackboard 访问，建议增加异常保护。

```cpp
CheckLookAheadCondition::CheckLookAheadCondition(
    const std::string &condition_name,
    const BT::NodeConfiguration &conf)
    : BT::ConditionNode(condition_name, conf)
{
    auto bb = config().blackboard;

    if (!bb)
    {
        throw BT::RuntimeError(
            "[CheckLookAhead] Blackboard is nullptr");
    }

    if (!bb->get("node", node_) || !node_)
    {
        throw BT::RuntimeError(
            "[CheckLookAhead] Missing Blackboard key: node");
    }

    callback_group_ = node_->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive,
        false);

    callback_group_executor_.add_callback_group(
        callback_group_,
        node_->get_node_base_interface());

    // 初始化订阅
}
```

这样即使 Blackboard 中缺少必要变量，也能够给出明确的异常信息。

需要注意，原始日志并没有包含段错误堆栈，因此空指针解引用是根据代码推断出的高度可能原因，而不是已经通过调试器确认的唯一原因。

---

## 9. 工程实践：独立 Blackboard 与共享 Blackboard 如何选择？

对于复杂的机器人任务系统，是否使用共享 Blackboard，需要根据任务特点决定。

| 对比项 | 独立 Blackboard | 共享 Blackboard |
|---|---|---|
| 数据隔离 | 较好 | 较弱 |
| 子树复用 | 接口明确 | 需要约定变量名称 |
| 公共 ROS 对象访问 | 需要正确映射 | 可以直接读取 |
| 变量传递 | 显式端口映射 | 直接共享 |
| 调试难度 | 需要检查映射关系 | 相对简单 |
| 变量冲突风险 | 较低 | 较高 |
| Nav2 自定义恢复树 | 适合严格接口设计 | 适合共享公共运行环境 |

### 9.1 为什么本项目选择共享 Blackboard？

因为清洁机器人的恢复子树需要访问大量公共数据：

```text
node
tf_buffer
server_timeout
bt_loop_duration
walk_control_error_code
escape_direction
```

与此同时，恢复子树并不是完全独立运行的任务，而是主树执行过程中调用的异常处理模块。

因此，共享 Blackboard 可以减少数据传递配置，提高工程实现的便利性。

### 9.2 共享 Blackboard 的潜在问题

共享 Blackboard 虽然方便，但也存在变量名称冲突的风险。

例如，系统中存在三个恢复子树：

```text
WallFollowRecoveryTree
ComputePathRecoveryTree
CollisionRecoveryTree
```

如果它们都使用：

```cpp
blackboard->set<int>("error_code", error_code);
```

不同恢复任务可能互相覆盖错误码。

因此，建议采用更明确的命名方式：

```cpp
walk_control_error_code
compute_path_error_code
collision_recovery_count
wall_follow_escape_direction
```

另外，任务切换时，也应注意重置恢复次数、异常码以及恢复方向等状态，避免上一次任务的数据影响下一次任务。

---

## 10. 总结

本文分析了 ROS 2 Humble + Nav2 + BehaviorTree.CPP 3.x 环境下，一个典型的子树 Blackboard 访问异常问题。

### 问题现象

自定义节点在主树中能够正常获取：

```cpp
config().blackboard->get<rclcpp::Node::SharedPtr>("node");
```

但是放入 `SubTree` 后，出现：

```text
Blackboard::get() error. Missing key [node]
```

### 排查过程

通过对比主树和子树的运行情况，确认异常与 Blackboard 作用域有关。

随后验证发现：

- 主树可以正常读取 `node`；
- 子树无法读取 `node`；
- `include` 已经正确加载恢复树定义；
- `SubTreePlus` 显式映射未能解决问题；
- 使用共享 Blackboard 后，节点可以正常初始化。

### 最终解决方法

```xml
<SubTree
    ID="WallFollowRecoveryTree"
    __shared_blackboard="true" />
```

### 核心结论

**BehaviorTree.CPP 中，子树的 Blackboard 不一定与主树共享。**

当 Nav2 自定义 BT 节点需要直接访问 `node`、`tf_buffer` 等公共运行环境时，必须保证这些数据在子树的 Blackboard 作用域中可访问。

对于 BehaviorTree.CPP 3.x，可以根据具体版本和工程需求选择 Blackboard 映射机制或共享机制。

本项目最终通过：

```xml
__shared_blackboard="true"
```

解决了恢复子树无法访问 ROS Node 的问题。

这一问题也说明，在使用 BehaviorTree.CPP 开发复杂机器人任务时，除了理解 `Sequence`、`Fallback`、`RecoveryNode` 等控制节点，还需要理解 **Blackboard 的作用域、数据映射和子树实例化机制**。

这些机制直接影响行为树模块的复用性、可靠性和可维护性。

---

## 参考资料

1. BehaviorTree.CPP 官方文档：https://www.behaviortree.dev/
2. BehaviorTree.CPP GitHub：https://github.com/BehaviorTree/BehaviorTree.CPP
3. Navigation2 GitHub：https://github.com/ros-navigation/navigation2
4. Nav2 Issue #3640：BehaviorTree issue when SubTrees are used  
   https://github.com/ros-navigation/navigation2/issues/3640
5. BehaviorTree.CPP Issue #563：SubTreePlus Blackboard 映射相关问题  
   https://github.com/BehaviorTree/BehaviorTree.CPP/issues/563