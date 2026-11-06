# LAB5 要求｜无人机视觉跟踪（ROS 1 版）

> 记录于 2026-11-02。来源：学校作业平台截图（最高优先级）+ MIT 页面 `https://vnav.mit.edu/labs_2023/lab5/exercises.html` + GitHub 标签 `VNAV2021`（commit `2443ec7`）。
> 标记：`[学校]` 平台原文；`[MIT]` MIT 页面；`[推断]` 根据资料推断；`[未确认]` 需要老师确认。

## 0. 一句话

两道透视投影的数学题，加上用 SIFT、SURF、ORB、FAST+BRIEF 做特征检测和匹配，用 Harris + Lucas–Kanade 做特征跟踪，在一对图片和两个 rosbag 上比较；最后交一份能看懂结果的实验报告。截止 **2026-12-30 00:00**。

## 1. 学校要求（最高优先级）

| # | 平台原文 | 意思 |
|---|---|---|
| S1 | 标题「LAB5」 | = MIT Lab 5 |
| S2 | 「26/12/30 00:00截止」 | 2026-12-30 00:00 截止 |
| S3 | 「个人作业」 | 一个人做完全部（MIT 的 Team 部分也自己做） |
| S4 | 「100分」 | 满分 100，平台没分项 |
| S5 | 「LAB5: 无人机视觉跟踪」 | 中文名称 |
| S6 | 「LAB5: mit实验要求…/labs_2023/lab5/exercises.html」 | 任务内容以 MIT 2023（ROS 1）页面为准 |
| S7 | 「LAB5: 实验代码：…/VNAV-labs/releases/tag/VNAV2021」 | 起始代码 = MIT 仓库标签 `VNAV2021` 的 `lab5/`（ROS 1 catkin） |
| S8 | 「由于本实验稍复杂，故提前开放供大家学习提交」 | 提前开放，截止时间较晚 |

平台没写报告要求，`[推断]` 和 LAB1–4 一样：可用 ICRA 模板，要能清晰看懂实验结果。

## 2. 本次的额外约定

1. 所有新建的文件都放在虚拟机桌面 `~/Desktop/LAB5/`。
2. 用 **ROS 1（Noetic）+ MIT 仓库 `VNAV2021`**。

## 3. 要交什么（默认做法）

| # | 交付物 | 依据 |
|---|---|---|
| 1 | 实验报告 PDF（ICRA 模板），含 D1–D8 的推导、图、表和讨论 | `[学校]` 的落地 `[推断]` |
| 2 | 补全的 `lab5/feature_tracking` 代码 | `[MIT]` Team |

## 4. 任务清单（来自 MIT 页面）

| # | 内容 | MIT 分值 | MIT 分工 |
|---|---|---|---|
| D1 | 球心在 (0, 0, d)、半径 r 的球投影到图像上的方程；球心在任意位置时投影是什么形状 | 10 | 个人（数学题，排版成 PDF） |
| D2 | 两条平行 3D 直线的消失点；平行线在图像中仍然平行的条件（要证明） | 10 | 个人 |
| D3 | SIFT 检测关键点、计算描述子，画出 box 两张图的关键点 | 20 | 团队→个人 |
| D4 | FLANN 匹配；SIFT 描述子的维数；画比值测试（0.8）前后的匹配 | 10 | 团队→个人 |
| D5 | 用给的 RANSAC 函数分内外点，画图，填 SIFT 的统计表 | 10 | 团队→个人 |
| D6.a | 在 box 两帧上加 SURF、ORB（选做 FAST+BRIEF，+10），讨论试了哪些参数 | 20（+10） | 团队→个人 |
| D6.b | 在 `30fps_424x240_2018-10-01-18-35-06.bag` 和 `vnav-lab5-smooth-trajectory.bag` 上逐帧匹配，每个 bag 一张平均统计表，并写结论 | 同上 | 团队→个人 |
| D7 | Harris 角点 + LK 跟踪，画某一帧的轨迹，加进 D6 的表；LK 依赖什么假设；和描述子方法比较 | 20 | 团队→个人 |
| D8 | 选做：Farneback 稠密光流 | +10 | 团队→个人 |

统计表的行：图 1 关键点数、图 2 关键点数、匹配数、good matches 数、内点数、内点率。

## 5. 技术要点

| 数据 | 话题 | 图像 | 帧数 |
|---|---|---|---|
| `30fps_424x240_2018-10-01-18-35-06.bag`（真实相机） | `/camera/color/image_raw` | 424 × 240，rgb8 | 1584 帧，59.7 s |
| `vnav-lab5-smooth-trajectory.bag`（TESSE 仿真） | `/tesse/left_cam/rgb/image_raw` | 720 × 480，rgb8 | 1386 帧，69 s |

- launch 把两个话题都重映射到节点订阅的 `/images_topic`。
- bag 在 MIT 的 Dropbox 文件夹 `VNAV-lab-data/lab5`（GitHub 上的 `VNAV-lab-data` 仓库只有一个链接），国内需要代理。
- MIT 的 `opencv3_catkin` 编译 OpenCV 3.4.2，和 Noetic `cv_bridge` 用的 OpenCV 4.2 冲突；而 Ubuntu 的 OpenCV 4.2 没有 `xfeatures2d`（SIFT、SURF、BRIEF）→ 用系统 4.2，只补编这三个算法。

## 6. 这台虚拟机的情况和路线

和 LAB3、LAB4 同一台虚拟机（UTM「Ubuntu20」，Ubuntu 20.04.6 arm64，4 核）。

```
~/Desktop/LAB5/
├── env.sh                 source 它：ROS + 工作区，ROS 日志也写到桌面
├── vnav_ws/               catkin 工作区：lab_5、dependencies
├── downloads/             两个 bag
├── scripts/               统计脚本
├── deliverable12answer/   D1、D2
└── results/               图、日志、表格
```

## 7. 未确认的问题（默认做法写在后面）

1. D1、D2 书面题是否必做？→ 默认做，写进报告。
2. 选做的 FAST+BRIEF 和 D8 算不算分？→ 默认做。
3. 报告格式是否和 LAB1–4 一样？→ 默认 ICRA 模板。
