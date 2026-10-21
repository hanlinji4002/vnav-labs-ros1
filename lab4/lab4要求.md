# LAB4 要求｜无人机控制和竞速（ROS 1 版）

> 记录于 2026-10-19，10-21 补充。来源：学校作业平台截图（最高优先级）+ MIT 页面 `https://vnav.mit.edu/labs_2023/lab4/exercises.html` + GitHub 标签 `VNAV2021`（commit `2443ec7`）。
> 标记：`[学校]` 平台原文；`[MIT]` MIT 页面；`[推断]` 根据资料推断；`[未确认]` 需要老师确认。

## 0. 一句话

两道轨迹优化的数学题，加上在 TESSE 仿真器的赛道里，让 LAB3 的四旋翼按最小 snap 轨迹依次穿过 12 个门；最后交一份能看懂结果的实验报告。截止 **2026-11-10 09:02**。

## 1. 学校要求（最高优先级）

| # | 平台原文 | 意思 |
|---|---|---|
| S1 | 标题「LAB4」 | = MIT Lab 4 |
| S2 | 「26/11/10 09:02截止」 | 2026-11-10 09:02 截止 |
| S3 | 「个人作业」 | 一个人做完全部（MIT 的 Team 部分也自己做） |
| S4 | 「100分」 | 满分 100，平台没分项 |
| S5 | 「LAB4：无人机控制和竞速」 | 中文名称 |
| S6 | 「LAB4 mit实验要求：…/labs_2023/lab4/exercises.html」 | 任务内容以 MIT 2023（ROS 1）页面为准 |
| S7 | 「LAB4实验代码：…/VNAV-labs/releases/tag/VNAV2021」 | 起始代码 = MIT 仓库标签 `VNAV2021` 的 `lab4/`（ROS 1 catkin） |

平台没写报告要求，`[推断]` 和 LAB1–3 一样：可用 ICRA 模板，要能清晰看懂实验结果。没写的（`[未确认]`）：要不要交源码、视频、rosbag；D1、D2 是否必做；评分细则。

## 2. 本次的额外约定

1. 所有新建的文件都放在虚拟机桌面 `~/Desktop/LAB4/`（MIT 写的 `~/labs`、`~/vnav_ws`、`~/vnav-builds` 都改到这个文件夹下面）。
2. 用 **ROS 1（Noetic）+ MIT 仓库 `VNAV2021`**；之前 ROS 2 的那次不用理会。
3. 控制器用 LAB3 调好的（MIT 也是这样要求的）。

## 3. 要交什么（默认做法）

| # | 交付物 | 依据 |
|---|---|---|
| 1 | 实验报告 PDF（ICRA 模板），含 D1、D2、D3 | `[学校]` 的落地 `[推断]` |
| 2 | 补全的 Part 0、1.1–1.3 代码 | `[MIT]` D3 |
| 3 | 完整飞完赛道的视频 | `[MIT]` D3 |
| 4 | 最快一次的 rosbag（`/current_state` 和 `/desired_state`） | `[MIT]` D3 |

MIT 的提交方式（git 仓库、视频和 rosbag 放网盘、链接写进 `rviz_drone_race.txt`）学校没有，不适用；内容写进报告。

## 4. 任务清单（来自 MIT 页面）

### D1 单段轨迹优化（MIT 20 分，个人，数学题要排版成 PDF）

最小速度问题 $\min \int_0^1 (P'(t))^2 dt$，约束 $P(0)=0$、$P(1)=1$，$P$ 是多项式（没有常数项）。

1. $P = p_1 t$：最优解和最优代价。
2. $P = p_2 t^2 + p_1 t$：(a) 把代价写成 $\boldsymbol p^\top Q \boldsymbol p$；(b) 把 $P(1)=1$ 写成 $A\boldsymbol p=b$；(c) 解 QP，比较代价有没有变小。
3. $P = p_3 t^3 + p_2 t^2 + p_1 t$：(a) 写出 $Q, A, b$；(b) 解 QP，和课上 Euler–Lagrange 方程的结论对照。
4. 再加约束 $P'(1) = -2$，分别用 2 次和 3 次多项式求最优解和最优代价。

### D2 多段轨迹优化（MIT 15 分，个人）

1. 最小 snap（$r=4$）、$k$ 段：需要多少个、哪几类约束（中间点和起终点上的 waypoint 约束、free derivative 约束、fixed derivative 约束）。提示：每段多项式几次、有多少未知数、每类约束提供几个方程，参考讲义里 $k=3$ 的图。
2. 推广到最小化第 $r$ 阶导数、$k$ 段。

### D3 无人机竞速（MIT 65 分，团队→个人）

1. Part 0（`planner_pkg/src/simple_traj_planner.cpp`）：取第一个门的位姿，速度、加速度置零，发布期望状态，并发布期望位姿的 TF——飞到第一个门悬停。测试：`roslaunch planner_pkg static_point_test.launch`。
2. Part 1.1–1.3（`trajectory_generation_pkg/src/trajectory_generation_node.cpp`）：读当前位置；用 `mav_trajectory_generation` 把门转成顶点（位置维度 `D`、偏航单独一维，注意偏航角 2π 绕回，最后一个门用 `makeStartOrEnd`），解最小 snap；按时间采样，发布期望状态。
3. 把 LAB3 调好的增益抄进 `traj_following.launch` 和 `static_point_test.launch`。
4. 交一段飞完赛道的视频和一份最快那次的 rosbag：`rosbag record /current_state /desired_state`。
5. 选做（Extra credit 15 分）：飞得更快，MIT 纪录 10.3 s。

运行顺序（MIT 页面）：仿真器 `./lab4.x86_64` → `roslaunch tesse_ros_bridge tesse_quadrotor_bridge.launch` → `roslaunch trajectory_generation traj_following.launch` → `roslaunch planner_pkg traj_gen.launch`。仿真器窗口里按 `R` 重生。

## 5. 技术要点

| Topic | 类型 | 方向 |
|---|---|---|
| `/desired_traj_vertices` | `geometry_msgs/PoseArray` | `traj_vertices_publisher` → 轨迹节点：12 个门的位姿，从仿真器的 `vnav-2020-lab4_static_tfs.csv` 读出并从 Unity 坐标转到 ROS 坐标 |
| `/current_state` | `nav_msgs/Odometry` | bridge → 轨迹节点、控制器（50 Hz） |
| `/desired_state` | `trajectory_msgs/MultiDOFJointTrajectoryPoint` | 轨迹节点 → 控制器 |

- 起始代码的分段时间来自 `estimateSegmentTimes(vertices, v_max, a_max)`，模板值 `v_max 8`、`a_max 4`，期望状态 5 Hz 发布。
- 出生点 (0.40, 8.00, 0.22) m，偏航 −166°；赛道约 96 m。

## 6. 这台虚拟机的情况和路线

- 和 LAB3 同一台虚拟机（UTM「Ubuntu20」，Ubuntu 20.04.6 arm64，4 核，无 3D 加速），ROS Noetic、box64 已经装好。
- 仿真器 `lab4.x86_64` 只有 x86_64 版 → 和 LAB3 一样用 box64 转译。下载地址 `https://vnav.mit.edu/material/lab4.zip`（54.5 MB），单连接很慢，分段并行下载。
- `dependencies/` 里 `glog_catkin`、`nlopt`、`yaml_cpp_catkin` 编译时要联网下载源码（`glog_catkin` 还缺 `gflags_catkin`）→ 换成导出系统库的空壳包，系统库用 apt 装（`libgoogle-glog-dev`、`libnlopt-dev`、`libnlopt-cxx-dev`）。
- 虚拟机没有可用的 GPU：UTM 4.7.5 开 GPU 加速后虚拟机里只有 OpenGL 2.1，而 Unity 2019.3 要 OpenGL 3.2 以上，所以仿真器窗口只能用 CPU 渲染。

桌面文件夹规划：

```
~/Desktop/LAB4/
├── env.sh              source 它：ROS + 工作区 + PYTHONPATH，ROS 日志也写到桌面
├── vnav_ws/            catkin 工作区
├── vnav-builds/lab4/   仿真器
├── scripts/            启动、测试、录屏脚本
├── results/            飞行日志、rosbag、视频
└── report/             报告
```

## 7. 未确认的问题（默认做法写在后面）

1. D1、D2 书面题是否必做？→ 默认做，写进报告。
2. 要不要交源码、视频、rosbag？→ 默认都备好。
3. 报告格式是否和 LAB1–3 一样？→ 默认 ICRA 模板。
