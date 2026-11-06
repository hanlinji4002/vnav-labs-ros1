# LAB5 代码：特征检测、匹配与跟踪（ROS 1 Noetic）

环境：Ubuntu 20.04 arm64（UTM 虚拟机）+ ROS 1 Noetic。起始代码是 MIT `VNAV-labs` 标签 `VNAV2021` 的 `lab5/feature_tracking`（包名 `lab_5`）。

## 目录

| 路径 | 内容 |
|---|---|
| `lab_5/` | 补全的 D3–D8：SIFT、SURF、ORB、FAST+BRIEF、Harris+LK 跟踪器，Farneback 稠密光流 |
| `dependencies/opencv3_catkin/` | 换成系统 OpenCV 4.2，并补编 Ubuntu 包里没有的 SIFT、SURF、BRIEF（见下） |
| `dependencies/glog_catkin/` | 同 LAB4：只导出系统的 `libgoogle-glog-dev` |
| `env.sh` | `source` 它：Noetic + 工作区，ROS 日志写到桌面 |
| `scripts/` | 下载数据、两帧统计、视频统计、汇总表格 |
| `checks/verify_math.py` | D1 / D2 的数值验证 |

## 相对 MIT 原版的改动

1. `feature_tracker.cpp`：补全 `trackFeatures`，依次是检测、画关键点、算描述子、匹配、画匹配和 good matches、RANSAC 内点、画内外点（红色是全部 good matches，绿色是内点）、统计。另外：
   - `setVisualization`：两帧模式保存所有图，然后等按键；视频模式只显示内外点图，并保存第 100 帧。
   - 统计里加了每对图的耗时（检测、描述、匹配）。
   - 没有 good matches 时内点率按 0 计，原版会除以 0，算出的 NaN 会让整个平均值失效。
   - `CV_FM_RANSAC` 改成 `cv::FM_RANSAC`（OpenCV 4 里没有前者）。
2. SIFT、SURF：`detect`、`compute`，FLANN `knnMatch`（k = 2）+ 比值测试 0.8。描述子为空时直接返回，否则 FLANN 会报错。
3. ORB：5000 个特征、8 层金字塔（模板是 500 个、1 层，在 box 上只有 7 个内点），用 Hamming 距离暴力匹配。
4. FAST+BRIEF：FAST 阈值 10，BRIEF 32 字节（头文件里加了 `extractor`），同样用 Hamming 距离暴力匹配。
5. Harris+LK：`goodFeaturesToTrack`（Harris，最多 1000 个，间距至少 10 px）+ `calcOpticalFlowPyrLK`。RANSAC 判为外点的轨迹丢掉，再在已有轨迹以外补新角点。显示渐隐的轨迹线和 `track count`，加了 `printStats`。
6. `track_features.cpp`：
   - 补全 LK 的回调。
   - 第一帧不统计：原版拿第一帧和它自己比。
   - 上一帧图像改为 `clone`：原版保存的指针指向回调结束后就释放的消息。
   - LK 模式结束时也打印统计：原版在这里对空指针调用 `printStats`。
7. `self_flow.cpp`（D8）：Farneback 稠密光流，色调表示方向、亮度表示大小（10 px 为最亮），保存第 100 帧和它的光流并排图。
8. launch：
   - 加了 `save_prefix`，以及 `params`（YAML 格式，如 `params:="{nfeatures: 1500, matcher: flann}"`）。roslaunch 会悄悄丢掉值里含 `:=` 的命令行参数，所以不能用 `_nfeatures:=1500`。
   - `video_tracking.launch` 加了 `rate`（播放倍率），rosbag 节点设为 `required`：bag 播完 launch 就结束，跟踪器打印统计。
   - 新增 `optical_flow.launch`。
9. `dependencies/opencv3_catkin`：原版下载并编译 OpenCV 3.4.2，和 Noetic 的 `cv_bridge`（链接 OpenCV 4.2）冲突。Ubuntu 的 OpenCV 4.2 又删掉了 `xfeatures2d`（SIFT、SURF、BRIEF 都在里面）。所以这个同名包直接用系统 OpenCV 4.2，只把 opencv_contrib 4.2.0 的 `sift.cpp`、`surf.cpp`、`brief.cpp` 带 `OPENCV_ENABLE_NONFREE` 编成 `libopencv_xfeatures2d.so`（源码已放在包里，`precomp.hpp` 是精简版）。编译只要几秒。

可调参数（`params` 里的键）：SIFT `contrast`（默认 0.04），SURF `hessian`（100），ORB `nfeatures`（5000）、`nlevels`（8），FAST `threshold`（10），ORB/FAST `matcher`（`bf` 或 `flann`），LK `max_corners`（1000）。

## 搭建

脚本里的路径按虚拟机上的布局 `~/Desktop/LAB5/` 写：

```
~/Desktop/LAB5/
├── env.sh
├── scripts/
├── vnav_ws/src/      lab_5, dependencies
├── downloads/        两个 rosbag
└── results/          figures/, logs/, stats/
```

1. 依赖：ROS Noetic 自带 OpenCV 4.2、`cv_bridge`、`image_transport`；另外要 `libgoogle-glog-dev`（LAB4 已装）。
2. 数据：`scripts/download_lab5.sh` 在 Mac 上运行，下载两个 bag（0.8 GB + 1.4 GB）后 `scp` 到虚拟机。bag 放在 Dropbox 上，国内需要代理。
3. 编译：`cd vnav_ws && catkin init && catkin config --extend /opt/ros/noetic --cmake-args -DCMAKE_BUILD_TYPE=Release && catkin build`（3 个包）。

## 运行

```
source ~/Desktop/LAB5/env.sh
roslaunch lab_5 two_frames_tracking.launch descriptor:=SIFT        # SIFT / SURF / ORB / FAST，在图窗口里按任意键退出
roslaunch lab_5 video_tracking.launch descriptor:=ORB path_to_dataset:=$HOME/Desktop/LAB5/downloads/30fps_424x240_2018-10-01-18-35-06.bag
roslaunch lab_5 video_tracking.launch descriptor:=LK path_to_dataset:=$HOME/Desktop/LAB5/downloads/vnav-lab5-smooth-trajectory.bag
roslaunch lab_5 optical_flow.launch path_to_dataset:=$HOME/Desktop/LAB5/downloads/vnav-lab5-smooth-trajectory.bag
```

跟踪器处理不过来时会跳帧，统计就变了：SIFT、SURF 在 smooth bag 上用 `rate:=0.25`，其余用 `rate:=0.5`。结束时打印的 `Num. of samples` 应等于帧数减 1（1583 和 1385）。

## 脚本

| 脚本 | 作用 |
|---|---|
| `download_lab5.sh` | Mac 上下载 OpenCV contrib 源码和两个 bag，传到虚拟机（可重复运行，断点续传） |
| `run_pair.sh <方法> ["{参数}"] [名字]` | box 两帧，约 8 s 后自动结束，打印统计，图存到 `results/figures/box_<名字>_*.png` |
| `run_video.sh <30fps\|smooth> <方法> <倍率> ["{参数}"] [名字]` | 视频统计，图存第 100 帧 |
| `run_all_video.sh` | 两个 bag × 5 种方法，约 26 分钟 |
| `make_tables.py` | 从 `results/logs` 生成 `results/stats/tables.md` |
| `common.sh` | 公共环境：独立 ROS master（端口 11315），显示到虚拟机桌面 |
