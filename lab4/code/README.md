# LAB4 代码：最小 snap 轨迹和无人机竞速（ROS 1 Noetic）

环境：Ubuntu 20.04 arm64（UTM 虚拟机）+ ROS 1 Noetic。起始代码是 MIT `VNAV-labs` 标签 `VNAV2021` 的 `lab4/`。控制器沿用 LAB3。仿真器 TESSE `lab4.x86_64` 只有 x86_64 版，用 LAB3 编译好的 [box64](https://github.com/ptitSeb/box64) v0.4.0 转译运行。

## 目录

| 路径 | 内容 |
|---|---|
| `planner_pkg/` | `src/simple_traj_planner.cpp` 补全了 Part 0（飞到第一个门悬停）。两个 launch 的仿真器数据路径改成 `~/Desktop/LAB4/...`，`static_point_test.launch` 加了 LAB3 的增益 |
| `trajectory_generation_pkg/` | `src/trajectory_generation_node.cpp` 补全了 Part 1.1–1.3（门 → 轨迹顶点、最小 snap、按时间采样发布期望状态）。`launch/traj_following.launch` 是最终参数 |
| `trajectory_generation_via_pkg/` | 上面这个包的副本，多一个 `via_points` 参数：在门 9 和门 10 之间加一个过渡点，这个弯离西墙从 1.6 m 变成 3.0 m。**最终演示用的是它** |
| `controller_pkg/` | LAB3 的控制器。控制频率改成参数 `control_hz`（默认 1 kHz，和 LAB3 一样） |
| `dependencies/` | MIT 原版依赖。`glog_catkin`、`nlopt`、`yaml_cpp_catkin` 换成只导出 Ubuntu 系统库的同名包（见下） |
| `tesse-ros-bridge/` | 同 LAB3（两处兼容修改） |
| `env.sh` | `source` 它：Noetic + 工作区 + `PYTHONPATH`（tesse-interface） |
| `scripts/` | 安装、下载、启动仿真器、试飞、记录、录屏、碰撞检测、量墙、离线规划 |
| `rviz/` | 并排录屏用的 RViz 配置（门框、期望/实际路径），配合 `scripts/lab4_viz.py` |
| `checks/verify_math.py` | D1 / D2 的数值验证（SymPy 精确分数 + KKT） |

竞速版（选做，10.2 s）在旁边的 [`../codeforRacing`](../codeforRacing)。

## 相对 MIT 原版的改动

1. `simple_traj_planner.cpp`（Part 0）：取第一个门的位姿作为期望状态，速度、加速度置零，发布到 `/desired_state`，并广播 TF `av-desired`。
2. `trajectory_generation_node.cpp`（Part 1.1–1.3）：
   - 1.1 存当前位置，另外存当前偏航作为偏航轨迹的起点（模板从 0 开始，而无人机出生时偏航是 −166°）；等到第一条 `/current_state` 才规划。
   - 1.2 `D = 3`；中间的门只约束位置和偏航，最后一个门用 `makeStartOrEnd` 停住；偏航解缠绕：取和上一个角最接近的等价角。
   - 1.3 采样时间 = 轨迹开始后经过的时间，超过总时长就停在终点；填位置、偏航四元数、速度（偏航角速度放 `angular.z`）、加速度。
   - `v_max`、`a_max`、`publish_rate` 做成节点参数（默认是模板值 8 / 4 / 5 Hz），并打印规划出的峰值速度、加速度、倾角。
3. `traj_following.launch`：默认 `v_max 10`、`a_max 12`、`publish_rate 100`（约 40 s 一圈）。增益 `kx 10, kv 5, kr 16, komega 2.5`：`kr` 从 LAB3 的 30 降到 16，因为带仿真器窗口时 CPU 渲染拖慢控制，`kr 30` 会抖。所有值都能在命令行覆盖，如 `kr:=30`。
4. 5 Hz 发布期望状态时，参考轨迹是阶梯状的，姿态会出现 5 Hz 的抖动；改成 100 Hz 后，模板轨迹的跟踪误差从 8.8 cm 降到 0.4 cm。
5. `trajectory_generation_via_pkg`：`via_points` 为 `[门序号, x, y, z, ...]`，在该门之后插入只约束位置的顶点（偏航取前后两个门的平均）。launch 里设为 `[9, -45.5, 6.5, 2.51]`。
6. `dependencies/`：原版 `glog_catkin`、`nlopt`、`yaml_cpp_catkin` 在编译时联网下载源码，`glog_catkin` 还依赖仓库里没有的 `gflags_catkin`。换成同名的空壳包，只导出 `libgoogle-glog-dev`、`libnlopt-cxx-dev`、`libyaml-cpp-dev`。`eigen_catkin` 直接用系统 Eigen 3.3.7。

## 搭建

脚本里的路径按虚拟机上的布局 `~/Desktop/LAB4/` 写，box64 用 LAB3 那份（`~/Desktop/LAB3/tools/box64`，见 [`../../lab3/code`](../../lab3/code)）：

```
~/Desktop/LAB4/
├── env.sh
├── scripts/
├── vnav_ws/src/          planner_pkg, trajectory_generation_pkg, trajectory_generation_via_pkg,
│                         controller_pkg, tesse-ros-bridge, dependencies
├── vnav-builds/lab4/     仿真器（scripts/download_sim.sh）
└── results/rviz/         rviz/lab4_sbs.rviz
```

1. `scripts/install_deps.sh`：glog、nlopt（需要 sudo）。ROS Noetic、box64 按 LAB3 装。
2. `scripts/download_sim.sh`：仿真器 `lab4.zip`。单连接很慢，脚本分 16 段并行下载。
3. 编译：`cd vnav_ws && catkin init && catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release && catkin build`（20 个包）。

## 运行

```
source ~/Desktop/LAB4/env.sh
~/Desktop/LAB4/scripts/sim_ctl.sh start                     # 仿真器（无窗口）+ bridge，约 30 s；加 window 带 Unity 窗口
roslaunch trajectory_generation_via traj_following.launch    # 轨迹节点 + 控制器（最终版，带过渡点）
roslaunch planner_pkg traj_gen.launch                        # 发布 12 个门，轨迹从这时开始
~/Desktop/LAB4/scripts/sim_ctl.sh stop
```

- 不带过渡点的原版：`roslaunch trajectory_generation traj_following.launch`。模板参数：`v_max:=8 a_max:=4 publish_rate:=5`。
- Part 0：`roslaunch planner_pkg static_point_test.launch`。
- `scripts/reset_drone.py` 停桨并把无人机放回出生点。带窗口时仿真器的 Respawn 不起作用，所以脚本再用 Reposition 直接放回去。
- 碰撞上报：`BRIDGE_ARGS="publish_collisions:=true" sim_ctl.sh start`，话题 `/tesse/collision`；起飞前停在地面上会报 `Floor_01`。

## 测试脚本

| 脚本 | 作用 |
|---|---|
| `fly_part0.sh [秒]` | Part 0：飞到门 0 悬停，记录并算误差（`hover_log.py`） |
| `fly_race.sh <名字> [v_max a_max 发布频率] [bag] [控制频率]` | 无窗口飞一圈并记录（`race_log.py`：过门偏差、跟踪误差、碰撞、实时因子）。`TRAJ_PKG=trajectory_generation_via` 选包，`LAUNCH_ARGS="kr:=30"` 改增益 |
| `record_sbs.sh <名字> [v_max a_max 发布频率 控制频率]` | RViz + 仿真器窗口并排录屏，同时记录和录 rosbag。仿真器以 `nice 10` 运行，RViz 15 fps |
| `roll_spectrum.py <bag>...` | 姿态 2–10 Hz 抖动（滚转 / 俯仰 RMS）和最大倾角 |
| `collision_log.py <csv>` | 记录 `/tesse/collision` |
| `probe_wall.py x0 y z [方向]` | 量墙的位置：无人机放到 (x0, y, z) 悬停，再沿 x 慢慢移动，直到仿真器报碰撞 |
| `plan_path.sh <前缀> v_max a_max [via_points]` | 只规划不飞：独立 ROS master（端口 11313）、10 倍速假时钟，导出整条期望轨迹，算门 9→10 弯离西墙的距离 |
| `export_attitude.py`、`lab4_viz.py`、`move_window.py`、`xkey.py`、`sim_ok.py`、`run_sim.sh`、`sim_ctl.sh` | 辅助：导出姿态、RViz 门框与路径、摆窗口、按键、检查仿真器、启动仿真器 |
