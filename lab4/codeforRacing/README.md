# LAB4 竞速版代码（选做：Faster, faster）

**最快一次：10.2 s**。录像里无人机在起飞后 10.24 s 穿过第 12 个门，12 个门全部通过，除了起飞离地时碰到地面，没有碰到任何东西。MIT 的纪录是 10.3 s。

录像：[`../results/lab4_racing_rviz_sim.mp4`](../results/lab4_racing_rviz_sim.mp4)（左 RViz，右仿真器窗口，按真实速度播放）。

这里只放和 [`../code`](../code) 不同的部分：一个轨迹生成包和两个脚本。环境、仿真器、控制器、其余包和脚本都用 `../code` 里的。

## 目录

| 路径 | 内容 |
|---|---|
| `trajectory_generation_fast_pkg/` | 包名 `trajectory_generation_fast`。`trajectory_generation_node.cpp` 的副本，改了分段时间的分配方法，并在最后一个门之后加了 5 m 的冲出段 |
| `scripts/plan_test.sh` | 只规划不飞（独立 ROS master，端口 11312）：打印轨迹总时长、峰值速度 / 加速度 / 倾角、各段时间 |
| `scripts/with_lock.sh` | 先拿仿真器锁 `/tmp/tesse_sim.lock` 再执行命令，避免两个进程同时用一个仿真器 |

## 和 `code` 里的版本有什么不同

`code` 用起始代码的分段时间估计（只看两个顶点之间的直线距离）。这个估计给急转弯的时间太少、给直道的时间太多，所以单纯调大 `v_max` / `a_max` 只能快到约 17 s：门 0 到门 1 那个急弯的倾角会超过 100°，再快就会掉到地上。

这个版本自己选分段时间：

1. **时间分配**：给定各段时间，照常解最小 snap。然后对各段时间的比例做坐标搜索，让总时长最短，约束是峰值加速度 ≤ `a_lim`、向下加速度 ≤ `down_lim`（推力不需要朝下）。最后整体缩放一次，让负荷最大的那一段正好达到上限。
2. **冲出段**（`runout`）：终点放在最后一个门沿门朝向往前 5 m 处，无人机高速穿过最后一个门，而不是在门里停下。用时按穿过第 12 个门的时刻算。

所有新参数默认为 0 时关闭，行为和原版相同。`dump_file` 可以把规划出的整条轨迹写成 CSV。

## 录像那次的设置和结果

| 项 | 值 |
|---|---|
| 规划参数 | `a_lim 20`、`down_lim 6`、`runout 5`、`v_max 10`、`a_max 12`、发布 100 Hz |
| 控制器 | `kx 10, kv 5, kr 16, komega 2.5`，1 kHz |
| 规划时长 | 11.33 s（峰值 13.8 m/s，规划倾角最大 69°） |
| 穿过第 12 个门 | **10.24 s** |
| 最高速度 / 最大倾角 | 15.0 m/s / 84° |
| 离门中心最远 | 60 cm（门 5）；门框约 2 m 宽 |
| 碰撞 | 只有起飞离地时（0.5–0.6 s）碰到地面 |
| 门 9→10 的弯离西墙 | 3.4 m（不用加过渡点） |

## 用法

先按 [`../code/README.md`](../code/README.md) 搭好工作区，再把 `trajectory_generation_fast_pkg/` 放进 `vnav_ws/src/`，`scripts/` 里的两个脚本放进 `~/Desktop/LAB4/scripts/`：

```
cd ~/Desktop/LAB4/vnav_ws && catkin build trajectory_generation_fast
source ~/Desktop/LAB4/env.sh
~/Desktop/LAB4/scripts/sim_ctl.sh start
roslaunch trajectory_generation_fast traj_following.launch kr:=16   # launch 默认 kr 是 30
roslaunch planner_pkg traj_gen.launch
```

录像（RViz + 仿真器窗口并排，同时录 rosbag）：

```
TRAJ_PKG=trajectory_generation_fast LAUNCH_ARGS="kr:=16" ~/Desktop/LAB4/scripts/with_lock.sh lab4-fast ~/Desktop/LAB4/scripts/record_sbs.sh fast10_sbs 10 12 100 1000
```

只看规划结果、不飞：

```
TRAJ_PKG=trajectory_generation_fast ~/Desktop/LAB4/scripts/plan_test.sh 10 12 _a_lim:=20 _down_lim:=6 _runout:=5
```
