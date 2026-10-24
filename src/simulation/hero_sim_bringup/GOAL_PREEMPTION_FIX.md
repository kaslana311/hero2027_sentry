# 导航目标抢占误判成功：空旷区域低概率停驶的完整复盘

日期：2026-10-24，Asia/Kuching，UTC+8。

算法工作区：`/home/apocalypse_linux_00/hero2026_-sentry (to_compile_not_develop))`。
修复分支：`codex/split-static-planning`，修复前提交：`6495b8a`。
仿真分支：`codex/rmuc-2027-field`，提交：`03489a4`；本轮没有修改仿真仓库。
修复采用当前算法分支上的一次向前提交，不改写历史、不合并main、不新建分支或标签。

## 1. 先说结论，以及“修好”的验证范围

这次不是小车被空旷处的空气墙卡住，也不是A*起点落在障碍物内再次无解。
直接原因是：**新导航任务已经接管，但控制器还在跟随旧任务的路径；旧路径恰好完成，
行为树便把这份“成功”传给了新任务。于是新任务尚未真正跟随自己的路径，就被判定成功结束。**

所以现场同时出现了几个看似矛盾的现象：

- RViz仍能看见一条通往远处的新规划线，但车停下不再走。
- 附近三张代价地图都认为当前位置是自由区域。
- 定位仍在正常更新，没有几十米漂移。
- 终端却出现`Reached the goal!`和`Goal succeeded`，RViz显示`reached`。
- 没有后续持续速度指令，导航桥显示`command_active=false`。

这不是整个系统死锁，而是**本次任务错误地结束了**。本轮没有向现场主动再发新目标，
因此不能根据这次停驶推导出“从此永远拒绝所有新目标”。

本次已经确认：旧版本控制流和真实Nav2 Action客户端能复现故障；加入修复后，
相同触发条件下新任务保持RUNNING，只有新目标自己的跟随完成才SUCCESS。
正式安全编译、安装、动态库加载及231项回归通过。

这不等于“所有物理卡滞、所有定位问题和未来所有导航异常都已解决”。
本轮未重新启动完整Gazebo+Point-LIO+MINCO+MPC做整场运动验收，不能冒充现场已跑完一轮。

## 2. 为什么绝大多数时候中途换目标没有问题

### 2.1 中途换目标，本来就不等于立刻把控制器换成新路径

系统里至少有三种不同的对象：

1. **导航目标**：NavigateToPose action中的目的地，以及行为树黑板上的`goal`。
2. **规划/平滑路径**：黑板上的`path`，先由A*产生，再由MINCO平滑。
3. **跟随任务**：FollowPath action中当前已经接受的路径和它自己的action UUID。

接收新导航目标B时，`goal`可以已经变成B，但`path`仍是A的路径，FollowPath也仍在跟随A。
这本身是原流水线的设计：不要在每次同目标重规划期间都停下，继续跟随已知旧路径，
等新路径准备好后再交接。因此“收到B”“A*接收B”“B路径发布到RViz”“B已经由MPC跟随”
是四件事，不能画等号。

### 2.2 正常成功换目标的时序

通常发生的是：

1. 接收B，更新`goal`。
2. 小车暂时仍沿A的路径运动。
3. B的A*和MINCO较快完成，黑板`path`更新为B的可跟随路径。
4. FollowPath客户端发现路径变化，发送新的跟随请求。
5. 控制器接受B的路径；旧跟随结果即使晚到，也不能替代当前跟随目标的结果。

Nav2的BtActionNode会检查action结果UUID，避免旧action结果污染同一个客户端已切换的新action。
因此只要新路径**先交接给跟随任务**，旧A后续完成或取消一般不会导致本次错误。

本次回归包含了“去掉修复、但让新路径先准备好”的旧版对照，旧版也可以正常换目标。
不是只有错误反例而没有正常对照。

### 2.3 出错需要一个特定的时间窗口

设：

- `T_new_ready`：B的新路径真正准备好并交给FollowPath的时刻。
- `T_old_success`：A的FollowPath成功结果被行为树消费的时刻。

多数正常情况是`T_new_ready < T_old_success`。本次出错是相反顺序：
`T_old_success < T_new_ready`，而导航器当前任务已经是B。

典型高风险条件是：**A已经快到终点，这时突然给了一个相当远的新目标B，
B的规划/平滑又没来得及在A完成前接管。**

如果A离终点还很远，留给新路径准备的时间通常很多；如果B规划很快，新路径也能先接管。
所以以前很多次“旧目标还没到就发新目标”都正常，与本次故障不矛盾。
在临近终点的边界上，早晚几十毫秒、规划负载或通信调度变化，就可能改变先后顺序。
这是一种时序竞态，不需要附近有墙、坡、障碍或定位漂移才能发生。

仿真树原有3Hz重规划限频可能带来一个周期内的等待，A*、平滑和action调度也会占时间。
本次新A*的`planning_time`为0.268秒ROS/仿真时间，旧末段的几个短规划结果为0或不足时间分辨率。
新目标没有对应的新SmoothPath，说明这次**在新A*完成并被整条树消费前就出了问题**，
不能把它归咎于“这次MINCO一定算得很慢”。

墙钟和仿真时钟不同，本文不会把0.268秒直接当作墙钟0.268秒。
也没有足够的重复样本给出发生概率、CPU占比或“split导致故障概率增加多少”的百分比。

## 3. 本次现场的具体证据

### 3.1 版本、运行与位置

本次算法运行约从2026-10-24 22:02:41开始，相关节点PID：
controller_server=149505，planner_server=149509，bt_navigator=149513，真值桥=149493。
输入为RMUC2027、双雷达、split、Point-LIO定位和点云接管。

停下后只读采样：

- Point-LIO/map约`(0.04235, 7.53995, 0.14494)`。
- 真值/map约`(0.04334, 7.60184, 0.16077)`。
- XY差约0.0619米，角度差约0.082°，没有足以解释21米目标误判的定位跳变。
- 三维真值世界XY约`(4.6471, 13.9972)`，不是上一轮东北坡顶接缝卡点。
- 动态全局、局部、静态先验地图当前位置代价均为0。
- 约9秒墙钟采样内定位/地图/状态仍更新，但没有新速度或新路径消息。
- bridge为`ready`，relay开启，`command_active=false`。
- NavigateToPose和FollowPath的活动任务计数均为0，最后状态均是ROS action的4，即SUCCEEDED。

仅凭没有速度不能证明该位置任意方向的物理通行都已验收；但它足以说明，
本次不是“控制器一直要求运动，物理模型却执行不了”这种故障链。

### 3.2 从动作服务器保留结果还原时序

本次不仅看了终端，还只读调用各action的get_result，取回了真实结果。

| 顺序 | 时间 | 实际事件 |
|---|---|---|
| 1 | 仿真52.624s | 最后一个旧SmoothPath接收；结果9点，末端约`(-0.0125,7.4625)` |
| 2 | 仿真52.643s | 最后一个旧FollowPath接收，继续跟随A |
| 3 | 仿真52.747s；墙钟22:04:38.900左右 | 新NavigateToPose B接收，目的地`(16.94,-5.06)` |
| 4 | 仿真52.796s | B的ComputePathToPose接收，开始计算新路径 |
| 5 | 墙钟22:04:39.362 | controller_server打印`Reached the goal!`，完成的是旧路径A |
| 6 | 墙钟22:04:39.500 | halt正在运行的规划时，打印`Failed to get result ... in node halt!` |
| 7 | 墙钟22:04:39.501 | bt_navigator对当前新任务B打印`Goal succeeded` |

B的导航UUID是`41ce3ff141135a3e65dffb87f1cdabc1`，保留结果实际为SUCCEEDED，
不是只看到了旧任务的一行成功日志。

B的A*最终成功返回708点，末端`(16.937500416,-5.062500075)`。
但最后的SmoothPath仍是52.624s的旧任务，最后FollowPath仍是52.643s的旧任务，
都早于B的导航接收时间。**B确实规划出了路，却没有走完“新平滑—新跟随”的接管流程。**

停下位置离旧目标A约9厘米，符合当前SimpleGoalChecker的15厘米容差；
离新目标B约21米。控制器对旧路径完成的判断是合理的，错在成功被上层归到了B。

### 3.3 为什么画面可能仍显示一条远处路线

planner_server可以把新A*结果发布出去，但行为树可能已经提前结束，
没有再消费该结果、调用新平滑或把它交给新的跟随任务。
此外RViz也会保留最后收到的路径。**看到线路不等于MPC仍在执行它。**

## 4. 错误究竟在代码哪一层发生

### 4.1 原来的关键结构

原仿真树的结构可简化为：

```xml
<RecoveryNode number_of_retries="150">
  <PipelineSequence>
    <RateController hz="3.0">
      <Sequence>
        <ComputePathToPose goal="{goal}" path="{path}"/>
        <SmoothPath unsmoothed_path="{path}" smoothed_path="{path}"/>
      </Sequence>
    </RateController>
    <FollowPath path="{path}"/>
  </PipelineSequence>
  <!-- 失败之后才进入的恢复分支 -->
</RecoveryNode>
```

真实文件还有每阶段的恢复包装，本文没有删除它们。

PipelineSequence的语义不是普通Sequence：它允许前面曾经成功过的子节点重新RUNNING时，
继续tick后面已启动的子节点。这让机器人能在重规划期间继续走，属于正常用途。

但同样由于这个语义，**上游规划仍RUNNING、最后一个FollowPath返回SUCCESS时，
PipelineSequence可以返回SUCCESS并halt上游**。本机Nav2头文件的状态表本来就描述了这种行为。

### 4.2 从旧路径成功到新任务成功的逐层传播

1. bt_navigator接受B的抢占，当前导航句柄换成B，黑板`goal=B`。
2. 黑板`path`和FollowPath的已接受目标仍属于A。
3. 新规划开始，但尚未通过完整规划/平滑流程更新跟随任务。
4. 旧FollowPath收到自己UUID对应的SUCCEEDED；这并不是客户端UUID过滤失效。
5. FollowPath BT叶子把旧action的SUCCEEDED映射成BT的SUCCESS。
6. PipelineSequence把最后一个子节点的成功作为整条流水线成功，停止上游规划。
7. 外层RecoveryNode收到主分支SUCCESS，也返回SUCCESS，不进入任何失败恢复。
8. bt_navigator据此结束当前导航句柄；当前句柄已是B，所以B被错误宣布成功。
9. 跟随结束、零速度/原有转发超时机制生效，车辆不再收到导航运动命令。

原树里的`GoalUpdated`在RecoveryFallback内，只在主导航分支失败后才可能处理，
无法在这条走SUCCESS的正常分支中隔离旧结果。
150次总重试、10次跟随重试也帮不上忙：**没有FAILURE，就不会因为这个问题进入重试。**

`Failed to get result ... in node halt!`是树提前结束后停止未完成规划时的伴随报错，
不能颠倒为“它先导致A*失败、然后车卡死”。get_result已证明新A*实际成功。

### 4.3 这是不是刚才改模型或最近split提交造成的

不是刚才的STL修复引入了这个行为树结构。
`git blame`显示该流水线随`ab1116a`于2026-09-26 19:57:29+08创建，
随后`fd84a3b`、`75e858e`主要调整恢复部分；最近split/A*恢复修复没有改这里。
原算法实车树也有相似流水线，但本轮没有拿实车验证，不能直接声称所有实车任务已复现。

split场景、远目标或负载变化可能扩大新路径接管窗口；本次没有频率/模式发生率的A/B统计，
不把“更容易观察到”说成已经量化证明了某个分支的全部责任。

## 5. 本次修复如何工作

### 5.1 只在导航流水线外增加一个小包装

新结构为：

```xml
<RecoveryNode name="NavigateRecovery" number_of_retries="150">
  <ResetOnGoalUpdate goal="{goal}">
    <PipelineSequence name="NavigateWithReplanning">
      <!-- 原规划、平滑、跟随和各自恢复节点，原样保留 -->
    </PipelineSequence>
  </ResetOnGoalUpdate>
  <ReactiveFallback name="RecoveryFallback">
    <!-- 原恢复分支，原样保留 -->
  </ReactiveFallback>
</RecoveryNode>
```

不是把PipelineSequence改成普通Sequence，也不是提高重试次数、改目标容差，
更没有加“遇到不动就自动倒车”、空点云发布、额外速度发布器或姿态锁定。

### 5.2 每个tick的实际处理

`ResetOnGoalUpdate`保存上一次目标的frame和pose。

- 第一次启动：记录目标，正常tick原流水线。
- 目标frame、位置、方向都没有变化：不重置，仍按原频率连续重规划、跟随。
- frame、位置或四元数方向发生变化：先调用BT标准的`resetChild()`，
  halt旧流水线，清除PipelineSequence已走到跟随阶段的进度；然后再tick新目标。
- 外层取消或正常停止：标准halt继续向子树传递，同时清除目标跟踪状态。

重置后，PipelineSequence从第一个子节点开始，不能再跳到旧FollowPath。
新A*或新MINCO尚未成功时，整条流水线保持RUNNING，跟随阶段不会被tick。
只有新目标自己的规划/平滑准备好，才启动新的FollowPath。
旧action晚到的结果由Nav2原有UUID检查处理，不会直接修改新任务成功状态。

一旦只是同一个目标的周期性重规划，包装不会触发，原来的边走边算仍保留。
真正换目标时，原RateController也随子树复位，新任务不必继承旧周期剩余等待。

### 5.3 为什么只改时间戳不重置

当前仿真入口使用固定map坐标目的地。同一frame、同一pose只是重新盖了时间戳，
并没有改变要去哪里。因此本实现不把单独的header.stamp变化当成新目的地，
避免上层重复发送同一点时不必要地停车和重算。

位置或方向比较没有人为加一个“变化阈值”，所以很小的真实目标变化也会重置。
此契约针对固定地图目标；将来若使用移动坐标系中的时间相关目标，应重新审查时间戳语义，
不能不加评估地把它推广为任意相对坐标追踪器。

### 5.4 实际改动位置与安装

改动都在`src/simulation/hero_sim_bringup/`内：

- `include/hero_sim_bringup/reset_on_goal_update.hpp`：目标比较、标准halt和重置。
- `src/reset_on_goal_update.cpp`：注册BT节点名`ResetOnGoalUpdate`。
- `behavior_trees/navigate_to_pose_sim.xml`：包住导航流水线，恢复分支不动。
- `launch/perception_truth_sim.launch.py`：仿真生成参数只追加一次插件库名。
- `CMakeLists.txt`、`package.xml`：构建、安装、导出动态库搜索路径和测试依赖。
- 三个目标切换测试文件，以及原split配置快照测试的明确新增插件兼容断言。

库名是`libhero_goal_update_reset_bt_node.so`；明确链接系统的BehaviorTree.CPP v3，
不是工作区决策树那份v4。readelf已确认依赖`libbehaviortree_cpp_v3.so`。

还验证了不使用绝对路径、只按库名加载的实际启动方式。
构建时补上标准`ament_export_libraries`环境hook；source原install/setup.bash后就能找到，
不要求用户手工改LD_LIBRARY_PATH或换启动入口。

## 6. 做了哪些验证，分别能证明什么

### 6.1 控制流回归

加载本机真实的Nav2 PipelineSequence、RateController、RecoveryNode，以及本次插件。
规划、平滑、跟随叶子用可控制的本地模拟动作隔离时序；不创建ROS通信。

覆盖15种情况：普通到达、限频窗口换目标、规划RUNNING时换目标、平滑RUNNING时换目标、
同目标连续重规划、仅更新时间戳、只改朝向、改变frame、连续A→B→C、取消后重启、
上一次成功后再导航、规划失败，以及三个旧版对照。

旧版对照既包含稳定重现错误，也包含“新路径先准备好时旧版本正常工作”。
这直接验证了“为什么以前大多数操作正常”的时序解释。

### 6.2 真实Nav2 Action客户端的异步回归

仅用本地模拟叶子还不够，因为真实action有确认、取消、结果排队和UUID过滤。
第二层测试直接加载本机ComputePathToPose、SmoothPath、FollowPath的真实Nav2 BT插件，
对接可控的测试action服务器。使用独立ROS域198、localhost和每进程独立命名空间；
不连接现场域0，不运行Gazebo或MPC，不向用户机器人发目标。

六项结果：

| 场景 | 结果 |
|---|---|
| 旧FollowPath成功结果已发出、但客户端尚未消费 | 新目标仍RUNNING；B准备好后正常跟随和完成 |
| 限频窗口内换目标 | 新任务不能被旧成功结束 |
| 旧规划仍RUNNING时换目标 | 旧动作取消/结果处理后，B正确接管 |
| 旧平滑仍RUNNING时换目标 | 旧平滑/跟随被halt，B平滑未好前不启动跟随 |
| 故意延迟旧规划取消结果 | 即使原Nav2 halt等待报超时，新任务也没有误判SUCCESS，释放后B完成 |
| 去掉本次包装的旧版反例 | 仍能重现新任务成功结束、却没有B跟随任务 |

取消延迟测试特意保留并断言了那条`Failed to get result ... in node halt!`日志。
这说明“出现这行日志”不等于修复没生效；必须结合目标UUID、动作状态和路径归属判断。
本补丁没有去改Nav2全局取消超时来消除日志。

### 6.3 参数和旧功能隔离

覆盖2025/2026/2027三个场地、combined/static_only/dynamic_only/split四模式，
确认只追加一个插件，重复生成参数不会追加重复项，实车基础YAML文字没有被修改。

原split的9项全参数哈希最初因新增插件而失败。最终处理不是重录一套新哈希：
先断言列表末尾恰好是本次插件、没有重复，再仅移除这一项，继续对其余全部参数
使用原历史哈希。原DOG分辨率的既有归一化仍保留。
因此代价层、地图尺寸、规划/平滑/MPC、频率和恢复参数的意外变化仍会被发现。

最终合计：**231项通过**，涵盖新控制流/异步测试及原恢复、代价地图、split配置检查。
其中新目标测试34项，原恢复契约3项，其余为既有地图/配置回归。

### 6.4 正式构建与部署检查

用户确认算法已关闭后，使用指定命令正式构建：

```bash
cd "/home/apocalypse_linux_00/hero2026_-sentry (to_compile_not_develop))"
./build_hero2026_safe.sh --jobs 4
```

为补齐动态库导出hook，最终版本又按同一安全脚本完整构建并审计。
39个工作区包完成，43个安装前缀，120个ELF；断链、缺失共享库、临时RPATH均为0。
测试中的临时CMake构建只生成/tmp下的测试程序，不替代安全脚本正式安装。

### 6.5 没有声称做过的验证

本轮没有重新跑完整场地的多目标导航，没有复测所有洞口和坡面物理接触，
也没有验证实车串口、硬急停或高速移动目标追踪。
“确认修好”指已复现的跨目标成功混淆链得到受控回归和真实异步action验证，
不把它扩展成所有导航故障从此永不出现的保证。

## 7. 修复的副作用与边界：具体有什么变化

### 7.1 真正换目标时，可能出现短暂的运动交接停顿

旧行为倾向于继续走A，等B准备好后无缝接管；新行为在识别目的地改变后，
先按标准halt结束旧流水线，再等B的路径准备好。
因此频繁换到不同目标，可能比以前多一些“停一下再走”的观感。
这是为消除跨任务成功混淆所作的明确保守选择，不是把速度上限或PID调小了。

**这不是硬急停。**标准halt会按原Nav2逻辑依次取消子动作，后端忙时有等待；
规划分支的halt也可能先于跟随分支执行。旧命令处理、转发超时和机器人惯性仍存在，
不能承诺一收到新目标就瞬间把物理速度归零。本轮没有新增强制速度写入或改变减速参数。

### 7.2 取消可能有等待，也可能保留原Nav2的超时日志

当前`default_server_timeout=100ms`保持不变。Nav2 halt分别等待取消响应和结果，
后端计算不及时响应时可能出现额外延迟或那条halt错误日志。
延迟取消测试已验证这种情况下不会再把A的成功直接算给B；
但本补丁不保证任意失联action服务器都能立刻恢复，也没有重写Nav2客户端内部实现。

### 7.3 高频改变目的地，可能反复取消而来不及起步

只重新盖时间戳的同点消息不会重置；但真实位置、方向或frame反复变化会重置。
如果将来决策节点每帧发送略有变化的新目标，应在任务层设计去重/更新策略，
或专门实现移动目标追踪，而不是期待离散NavigateToPose不断取消仍完全平滑。
本轮没有私自给决策消息增加距离阈值，也没有吞掉小幅但真实的目标变化。

### 7.4 只改朝向也会重置，但不会自动获得新的朝向控制功能

朝向属于目标pose，改变四元数会触发重置。当前仿真MPC的yaw tracking仍为原设置，
目标检查器也仍使用原来的yaw容差。包装只保证任务隔离，不会擅自打开转向跟踪。

### 7.5 同一目标下的连续重规划没有改成走走停停

同目标的`path`更新不会触发包装，原PipelineSequence仍继续tick旧跟随，
规划/平滑完成后按原流程更新路径。测试对这一点有明确断言。
3Hz重规划、100Hz控制、150次总恢复、10次跟随恢复、原5秒进度检查等保持不变。

恢复阶段仍由原RecoveryFallback/GoalUpdated处理；本补丁没有重新设计恢复任务抢占，
也没有把整棵树的重试预算、清图、倒车和Wait逻辑全部重置成另一套机制。

### 7.6 仿真四模式都受保护，实车原入口暂未改变

四种代价地图模式共用仿真导航树，所以都加载此保护，不只split。
但是它们的代价层、PGM含义、地图范围和先验规划器选择保持各自原样。
原实车树和基础nav2_params.yaml没有修改；若实车也要采用，应单独接入并验收。

### 7.7 增加了一个小型动态库，但没有新增运行节点或线程

新增状态是一份目标pose和一个有效标志，每个BT tick做一次目标比较。
没有新增ROS节点、定时器、订阅、速度发布器或后台线程。
没有做CPU耗时基准，不能写成“开销绝对为零”；只是代码路径和状态量很小。

若只复制新XML却没有安装新库/加载配置，会启动失败或找不到BT节点。
本轮已用安全脚本把库、XML、参数工厂和环境hook一起安装，并验证按库名加载。
切换代码分支后仍需重编译对应安装目录，不能把旧install与新source随意混用。

### 7.8 保持未改的部分与仍需区分的其他问题

没有改Point-LIO、DOG滤波、TF关系、地图原点、A*代价/恢复、MINCO优化或MPC权重；
没有改PGM、STL、机器人尺寸、驱动力、轮子摩擦、镜像或启动入口。

原日志中的真值桥TF dedicated-thread报错仍是另一项问题，本轮没有把它夹带修改。
本次C2导航由Point-LIO提供定位/点云，且已经有完整的错误SUCCESS证据链，
不能用那条TF报错替代本次原因，也不能说保留它就意味着其潜在影响为零。

## 8. 如何应用、如何复测

正式代码和插件已经编译安装；不需要新镜像或新启动参数。

由于算法本轮已经关闭，而车辆可能停在远离出生点的位置，建议把仿真也正常退出，
再从出生点成对启动。原因是原入口`zero_at_first_odometry=true`，
只在车辆已移动后重启算法可能重新取map初始原点，让PGM错位；这与本次插件不是同一问题。

仿真终端：

```bash
cd /home/apocalypse_linux_00/build_sim_docker
SIM_ROBOT_PROFILE=tunnel ./scripts/start_sim_rmuc_2027_dual_lidar.sh
```

车辆落稳后，算法终端使用原命令：

```bash
cd "/home/apocalypse_linux_00/hero2026_-sentry (to_compile_not_develop))"
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch hero_sim_bringup navigation_point_lio_sim.launch.py \
  field:=rmuc_2027 \
  lidar_mode:=dual \
  costmap_mode:=split \
  relay_cmd_vel:=true \
  publish_debug_visualizations:=false \
  point_lio_rviz:=true \
  filter_high_columns:=false \
  high_column_min_z:=0.20
```

建议人工验收：

1. 普通单目标正常到达；到达后发下一点。
2. 走向A，离A还远时切B；确认B确实接管。
3. 快到A时切到远处B，重复几次；允许交接停顿，但不能在原处报告B已到达。
4. 快速A→B→C，最后只以C为当前任务。
5. 导航途中取消，再发新目标。

遇到异常先对比当前NavigateToPose UUID、FollowPath接受时间/路径终点、速度和真值；
不要把所有“不动”一律归为物理卡滞，也不要仅凭一条残留路径判断任务仍在执行。

## 9. 版本、证据和复现文件

算法内有同内容的`src/simulation/hero_sim_bringup/GOAL_PREEMPTION_FIX.md`，随修复提交保存。
本文件放在用户指定的combined_sp目录；最终代码提交号在交付时另行记录，避免仓库文档
为写入自身哈希而反复制造提交。

本次现场只读证据：

```text
/home/apocalypse_linux_00/.codex/visualizations/2026/09/24/01a0d39c-2651-74c3-88fa-9206c221d0f1/
  open_area_stuck_20261024_2206/
    snapshot.json
    action_results.json
    排查结论.md
    repro/
```

测试源码保留在算法包test目录：

- `test_goal_update_reset_control_flow.cpp`：真实Nav2控制节点，确定性叶子时序。
- `test_goal_update_reset_actions.cpp`：真实Nav2异步客户端与独立测试服务器。
- `test_goal_update_reset.py`：旧/新对照、配置矩阵和测试运行管理。

安全构建日志在算法工作区`log/build_v2/`；最终231项回归的JUnit结果已保存至诊断证据目录。
这类运行输出不加入源码提交，也不通过删除日志掩盖曾经失败的配置快照检查。

版本查询命令：

```bash
git -C "/home/apocalypse_linux_00/hero2026_-sentry (to_compile_not_develop))" \
  log -5 --date=iso-strict --format='%h %ad %s'
git -C "/home/apocalypse_linux_00/hero2026_-sentry (to_compile_not_develop))" status --short --branch
git -C /home/apocalypse_linux_00/build_sim_docker status --short --branch
```

测试命令可在安全编译完成、source对应环境后运行；异步部分会自行使用独立ROS域198，
不要在真正使用该测试域的其他实验同时运行同名测试。

```bash
cd "/home/apocalypse_linux_00/hero2026_-sentry (to_compile_not_develop))"
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 -m pytest -q -p no:cacheprovider \
  src/simulation/hero_sim_bringup/test/test_goal_update_reset.py \
  src/simulation/hero_sim_bringup/test/test_recovery_contract.py \
  src/simulation/hero_sim_bringup/test/test_costmap_modes.py \
  src/simulation/hero_sim_bringup/test/test_split_mode.py
```

结语：这次修的是“旧任务完成结果跨目标传播”的边界，而不是靠修改环境或增大重试
让错误不容易被看见。它的代价是离散换目标时采用更保守的停止/重规划交接；
它保留同目标连续规划，但不承诺任意高频新目标流都无停顿。
