# LAB3 要求｜3D trajectory following（ROS 1 版）

> 记录于 2026-10-19。来源：学校作业平台截图（最高优先级）+ MIT 页面 `https://vnav.mit.edu/labs_2023/lab3/exercises.html`（当天重新读过）+ GitHub 标签 `VNAV2021`（commit `2443ec7`）。
> 标记：`[学校]` 平台原文；`[MIT]` MIT 页面；`[推断]` 根据资料推断；`[未确认]` 需要老师确认。

## 0. 一句话

给四旋翼写几何控制器（Lee et al. 2010），在 TESSE 仿真器里稳定跟踪半径 5 m 的圆轨迹；再做两道书面题；最后交一份能看懂结果的实验报告。截止 **2026-11-10 00:00**，也就是 **11-09 晚上必须交完**。

## 1. 学校要求（最高优先级）

| # | 平台原文 | 意思 |
|---|---|---|
| S1 | 标题「LaB3：3D trajectory following」 | 三维轨迹跟踪 = MIT Lab 3 |
| S2 | 「26/11/10 00:00截止」 | 2026-11-10 00:00 截止 |
| S3 | 「个人作业」 | 一个人做完全部（MIT 的 Team 部分也自己做） |
| S4 | 「100分」 | 满分 100，平台没分项 |
| S5 | 「LAB3：无人机路径规划，轨迹生成。」 | 中文名称。字面像 MIT Lab 4，和标题、链接不完全吻合 `[未确认]`，默认按标题做 Lab 3 |
| S6 | 「附件1：简版实验报告」 | 交一份实验报告（附件本体截图里看不到 `[未确认]`） |
| S7 | 「实验报告也可以参考ICRA论文模板等」 | 格式不强制，可用 ICRA 双栏模板 |
| S8 | 「要求能清晰明了看懂实验结果」 | **唯一的评分标准**：只看报告就能看懂结果。要有图、数据、说明 |
| S9 | 「LAB3 mit实验要求：…/labs_2023/lab3/exercises.html」 | 任务内容以 MIT 2023（ROS 1）页面为准 |
| S10 | 「LAB3实验代码：…/VNAV-labs/releases/tag/VNAV2021」 | 起始代码 = MIT 仓库标签 `VNAV2021` 的 `lab3/`（ROS 1 catkin） |

平台没写的（全部 `[未确认]`）：要不要交源码 / 视频；报告语言、页数；D1、D2 书面题是否必做；评分细则；迟交政策。

## 2. 本次的额外约定（高于学校和 MIT）

1. **所有新建的文件都放在虚拟机桌面**：统一放 `~/Desktop/LAB3/`（MIT 写的 `~/labs`、`~/vnav_ws`、`~/vnav-builds` 都改到这个文件夹下面）。
2. 这次用 **ROS 1（Noetic）+ MIT 仓库 `VNAV2021`**。之前那次 ROS 2 的工作不用理会。

## 3. 要交什么（默认做法）

| # | 交付物 | 依据 | 状态 |
|---|---|---|---|
| 1 | 实验报告 PDF（ICRA 模板），含 D1、D2、D3 全部内容 | `[学校]` S6–S8 | 必交 |
| 2 | 报告里的结果：rviz 截图、期望 vs 实际轨迹图、跟踪误差曲线、调好的增益表 | `[学校]` S8 的落地 `[推断]` | 必须有 |
| 3 | 补全的 `controller_node.cpp`（打包成 `Lab3_code.zip`） | `[MIT]` D3；学校没写 | 备好，平台能传就传 |
| 4 | rviz 里飞完一圈的录屏 | `[MIT]` D3；学校没写 | 备好，平台能传就传 |

MIT 的提交方式（git 仓库、`TEAM_<N>`、`deliverable_1.txt`、`rviz_drone_flight.txt`）学校没有，不适用；内容写进报告。

## 4. 任务清单（来自 MIT 页面，学校没另写任务书）

### D1 Transformations in Practice（MIT 10 分，个人，问答）

- A1：`geometry_msgs::Quaternion quat_msg` 存进已定义的 `tf2::Quaternion quat_tf`，一行 C++。
- A2：`tf2::Quaternion quat_tf` 转成 `geometry_msgs::Quaternion`，新变量名 `quat_msg`，一行 C++。
- A3：只要 `tf2::Quaternion` 的标量部分，用哪个成员函数。
- B1：`tf2::Quaternion quat_t` 一次函数调用取出 yaw。
- B2：`geometry_msgs::Quaternion quat_msg` 转 Eigen 3×3 矩阵（约两次函数调用）。
- 答案要在 Noetic 上编译验证。

### D2 Modelling and control of UAVs（MIT 20 分，个人，数学题要排版成 PDF）

- A：图中 (a) 正常四旋翼；(b) 把 3、4 号桨换了并反转。用 lecture notes eq. (6.9) 的 $F$ 矩阵比较两者的秩，证明 (b) 跟踪不了 $[x,y,z]$ + yaw $\Psi$。
- B：机体系阻力 $F^b=\mathrm{diag}(0.1,0.1,0.2)\,(v^b)^2$，$(v^b)^2=[-v_x|v_x|,-v_y|v_y|,-v_z|v_z|]^T$。说明在 Lee 控制器里怎么用它改善跟踪（提示：作为动力学里多出的一项显式补偿）。

### D3 Geometric controller（MIT 50 分，团队→个人）

1. 补全 `controller_pkg/src/controller_node.cpp` 所有缺失部分，能编译。
2. 调 `kx kv kr komega`（在 `launch/traj_tracking.launch`，默认全 1），让四旋翼稳定跟踪圆轨迹。
3. 录一段 rviz 里飞完一整圈的视频。
- 建议顺序：先把 `traj_publisher.cpp` 的 `STATIC_POSE` 改 1 测悬停（目标 (0,0,2)，yaw π/4），稳了再改回 0 跑圆。
- 圆轨迹：$(5\sin(t/2),\ 5\cos(t/2),\ 2)$，yaw $=-t/2$，速度 2.5 m/s，一圈约 12.6 s，`VNAV2021` 版 500 Hz 发布。

## 5. 技术要点（ROS 1 版，来自 MIT 页面和模板）

| Topic | 类型 | 方向 | 坐标系 |
|---|---|---|---|
| `/desired_state` | `trajectory_msgs/MultiDOFJointTrajectoryPoint` | `traj_publisher` → 控制器 | 世界系；rotation 只有 yaw |
| `/current_state` | `nav_msgs/Odometry` | bridge → 控制器 | 位置、速度世界系；**角速度也是世界系，要转机体系** |
| `/rotor_speed_cmds` | `mav_msgs/Actuators` | 控制器 → bridge | `angular_velocities` 4 个，**rad/s**，可为负 |

仿真器约定（MIT 页面）：

1. ROS 的世界系、机体系 z 轴都朝上，论文朝下：影响 $b_{3d}$、重力、气动力的符号。
2. 机体 x 轴在两臂中间 45°，论文沿桨臂：桨速分配矩阵 `F2W` 要按 lecture notes eq. (6.9) 重写（ROS 1 模板里 `F2W` 没填，要自己写）。
3. $\omega_i>0$ 时所有桨都产生向上推力；桨 1 顺时针转，产生逆时针（正）偏航阻力矩，与论文 $(-1)^i$ 相反。
4. $c_{\tau f}=c_d/c_f$。
5. 可以忽略 eq. (11) 第二项、eq. (16) 里含 $\Omega_d,\dot\Omega_d$ 的项。

运行顺序（MIT 页面）：仿真器 `./lab3.x86_64` → `roslaunch tesse_ros_bridge tesse_quadrotor_bridge.launch` → `rviz -d rviz/lab3.rviz` → `roslaunch controller_pkg traj_tracking.launch`。仿真器窗口里 `W` 开全部桨，`R` 重生。

## 6. 这台虚拟机的情况和路线

- 虚拟机：UTM「Ubuntu20」，Ubuntu 20.04.6 arm64，4 核 / 3.8 GB / 剩 26 GB；Xorg 桌面 1280×800；显卡 virtio（无 3D 加速 `[推断]`）；apt 用清华 `ubuntu-ports` 源。
- 2026-10-19 检查时**什么都没装**：没有 ROS、gcc、cmake、pip。
- 障碍：仿真器 `lab3.x86_64` 只有 x86_64 版，arm64 不能直接跑 → 用 `box64` 转译。20.04 的 apt 里没有 box64，要从源码编译（新版要 cmake 3.19，20.04 只有 3.16，所以用 v0.4.0）。
- ROS Noetic：清华镜像 `mirrors.tuna.tsinghua.edu.cn/ros/ubuntu` 有 focal arm64 包（已查到 `ros-noetic-desktop-full`、`ackermann-msgs`、`cv-bridge`、`tf2-eigen`、`tf2-geometry-msgs`、`python3-catkin-tools`）。
- `mav_comm` 没有 apt 包，要 clone `ethz-asl/mav_comm`（MIT 页面原文）。
- MIT 页面要 `pip install` tesse-interface 的依赖，其中 `enum34` 会在 Python 3.8 上覆盖标准库 `enum`、弄坏 pip `[推断]`。改用 apt 装 numpy / defusedxml / Pillow，tesse-interface 通过 `PYTHONPATH` 引用，不用 pip（也保证文件都在桌面）。

桌面文件夹规划：

```
~/Desktop/LAB3/
├── LAB3_要求.md        本文件
├── env.sh              source 它：ROS + 工作区 + PYTHONPATH，ROS 日志也写到桌面
├── labs/               MIT VNAV-labs，标签 VNAV2021
├── vnav_ws/            catkin 工作区（src/: catkin_simple, controller_pkg, tesse-ros-bridge, mav_comm）
├── vnav-builds/lab3/   仿真器
├── tools/box64/        box64 源码和编译结果
├── scripts/            启动、测试脚本
├── results/            飞行日志、图
└── report/             报告
```

## 7. 未确认的问题（默认做法写在后面）

1. S5「路径规划，轨迹生成」是否还要做 MIT Lab 4 的内容？→ 默认只做 Lab 3（LAB4 另有作业）。
2. D1、D2 书面题是否必做？→ 默认做，写进报告。
3. 附件1「简版实验报告」是模板还是要求？→ 默认用 ICRA 模板写一份短报告。
4. 要不要交源码、视频？→ 默认都备好。
