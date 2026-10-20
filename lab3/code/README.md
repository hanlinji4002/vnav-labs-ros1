# LAB3 代码：四旋翼几何控制器（ROS 1 Noetic）

环境：Ubuntu 20.04 arm64（UTM 虚拟机）+ ROS 1 Noetic。起始代码是 MIT `VNAV-labs` 标签 `VNAV2021` 的 `lab3/`。仿真器 TESSE `lab3.x86_64` 只有 x86_64 版，用 [box64](https://github.com/ptitSeb/box64) v0.4.0 转译运行。

## 目录

| 路径 | 内容 |
|---|---|
| `controller_pkg/` | 控制器包。`src/controller_node.cpp` 补全了 PART 1–5，`launch/traj_tracking.launch` 里是调好的增益 |
| `tesse-ros-bridge/`、`catkin_simple/` | MIT 原版，bridge 有两处兼容修改（见下） |
| `env.sh` | `source` 它：Noetic + 工作区 + `PYTHONPATH`（tesse-interface） |
| `scripts/` | 安装、启动仿真器、试飞、记录、调参、录屏脚本 |
| `rviz/` | 录屏用的 RViz 配置（`lab3.rviz` 的副本，镜头和显示项有调整） |
| `variants/` | 对照实验用的控制器变体：仿真时间定时器版、保留 Ω_d 项版 |
| `checks/` | D1 答案的编译验证、D2-A 秩的数值验证 |

## 相对 MIT 原版的改动

1. `controller_node.cpp`：补全 PART 1–5（订阅、发布、期望/当前状态、误差、期望姿态、力和力矩、桨速分配 `F2W`）。控制循环用 `ros::WallTimer`（1 kHz）：bridge 打开了 `use_sim_time`，而 `/clock` 只有 50 Hz，仿真时间的 `ros::Timer` 实际只能约 100 Hz 成对发指令，仿真器在指令稀疏时推力偏小（悬停低约 6 cm）。
2. `traj_tracking.launch`：增益 `kx=10, kv=5, kr=30, komega=2.5`（原为全 1）；新增参数 `smooth_start`。
3. `traj_publisher.cpp`：新增平稳起飞（`smooth_start:=true`，默认关闭，关闭时与原版完全一样）。参考轨迹从重生点出发，3 s 爬升到 2 m，8 s 内螺旋展开到半径 5 m 的圆上、速度平滑加到 2.5 m/s，起始偏航等于重生偏航；起飞时最大倾角从 89.5° 降到 9.5°，最大机体角速度从 401°/s 降到 29°/s。录屏脚本用它。
4. `tesse-ros-bridge`：两个节点脚本的 shebang 改为 `python3`；在 `cv_bridge` 之前先 `import cv2`（aarch64 上否则报 `initialization of cv_bridge_boost raised unreported exception`）。
5. tesse-interface 不用 `pip install`，通过 `env.sh` 的 `PYTHONPATH` 引用（它依赖的 `enum34` 会覆盖 Python 3 的 `enum`）。

## 搭建

脚本里的路径按虚拟机上的布局 `~/Desktop/LAB3/` 写：

```
~/Desktop/LAB3/
├── env.sh
├── scripts/
├── labs/                 git clone -b VNAV2021 https://github.com/MIT-SPARK/VNAV-labs
├── vnav_ws/src/          controller_pkg, tesse-ros-bridge, catkin_simple, mav_comm
├── vnav-builds/lab3/     仿真器（https://vnav.mit.edu/material/lab3.zip 解压）
├── tools/box64/          box64 v0.4.0 源码，在 build/ 里编译
└── results/rviz/         rviz/ 里的两个配置；results/variants/ 放 variants/
```

1. `scripts/install_deps.sh`：ROS Noetic（清华镜像）和依赖，需要 sudo。
2. `scripts/download_src.sh`：仿真器 zip 和 box64 源码；另外 `git clone https://github.com/ethz-asl/mav_comm.git` 到 `vnav_ws/src/`。
3. 编译 box64：`cd tools/box64 && mkdir build && cd build && cmake .. -DARM_DYNAREC=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo && make -j4`（不安装，脚本直接用 `build/box64`）。
4. 编译工作区：`cd vnav_ws && catkin init && catkin build`。

## 运行

```
source ~/Desktop/LAB3/env.sh
~/Desktop/LAB3/scripts/sim_ctl.sh start          # 仿真器（无窗口）+ bridge，约 30 s；加 window 带 Unity 窗口
roslaunch controller_pkg traj_tracking.launch    # 跟踪圆轨迹（悬停测试：traj_publisher.cpp 里 STATIC_POSE 改 1）
# 平稳起飞：roslaunch controller_pkg traj_tracking.launch smooth_start:=true
rviz -d ~/Desktop/LAB3/vnav_ws/src/controller_pkg/rviz/lab3.rviz
~/Desktop/LAB3/scripts/sim_ctl.sh stop
```

- box64 需要 `BOX64_DYNAREC_STRONGMEM=1`（`run_sim.sh` 已设置），否则 Unity 运行十几分钟后会因内存错误崩溃。
- 无窗口模式下没有键盘，`scripts/reset_drone.py` 代替按 R（停桨并重生）。

## 测试脚本

| 脚本 | 作用 |
|---|---|
| `fly_test.sh <hover\|circle> kx kv kr komega [秒] [名字]` | 用给定增益试飞一次，记录并算误差 |
| `fly_launch.sh <hover\|circle> [秒] [名字]` | 用 launch 文件里的增益试飞 |
| `flight_log.py` | 记录 `/current_state` 和 `/desired_state`，圆轨迹按时间戳解析计算期望位置，输出 CSV 和指标 |
| `gain_sweep.sh` | 13 组增益扫描 |
| `compare_runs.sh` | 指令频率与推力的开环测试；仿真时间定时器对照 |
| `omegad_run.sh` | 保留 Ω_d 项的变体对照 |
| `record_rviz.sh`、`record_sbs.sh` | RViz 录屏；RViz + 仿真器窗口并排录屏（平稳起飞） |
| `takeoff_test.sh [秒] [launch 参数]` | 量起飞的最大倾角、机体角速度、桨速（如 `takeoff_test.sh 20 smooth_start:=true`） |
