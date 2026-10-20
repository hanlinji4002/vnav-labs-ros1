# LAB3 代码（Windows 版）：在 Windows 电脑上用 WSL2 跑仿真

和 `../code/` 是同一套控制器，区别只在运行环境：

| | `code/` | `codeforWindows/` |
|---|---|---|
| 平台 | Apple 芯片 Mac 上的 Ubuntu 20.04 arm64 虚拟机 | Windows（Intel/AMD）上的 WSL2 Ubuntu 20.04 |
| 仿真器 | 用 box64 转译运行，软件渲染 | 直接运行，WSLg 用显卡渲染，帧率更高 |
| 工作目录 | `~/Desktop/LAB3` | `~/LAB3` |
| 录屏 | 虚拟机里 ffmpeg 录 | Windows 自带录屏工具 |

控制器部分（`controller_pkg/`、`tesse-ros-bridge/`、`catkin_simple/`）两边完全相同，改动说明见 `../code/README.md`。

## 1. 装 WSL2 和 Ubuntu 20.04

需要 Windows 11（或 Windows 10 22H2 + 最新版 WSL），显卡驱动更新到最新。

在 **PowerShell（管理员）** 里：

```
wsl --install -d Ubuntu-20.04
```

重启后打开开始菜单里的「Ubuntu 20.04」，按提示设置用户名和密码。以后的命令都在这个 Ubuntu 终端里运行。

（可选）国内下载慢可以先换清华源：

```
sudo sed -i 's|http://archive.ubuntu.com|https://mirrors.tuna.tsinghua.edu.cn|g; s|http://security.ubuntu.com|https://mirrors.tuna.tsinghua.edu.cn|g' /etc/apt/sources.list
```

## 2. 下代码、装依赖、搭工作区

仓库要 clone 在 Ubuntu 自己的目录里（`~`），不要放在 `/mnt/c/` 下。

```
cd ~
git clone https://github.com/hanlinji4002/vnav-labs-ros1.git
bash vnav-labs-ros1/lab3/codeforWindows/scripts/install_deps.sh     # ROS Noetic 等，约 1 GB，要输密码
bash vnav-labs-ros1/lab3/codeforWindows/scripts/setup_lab3.sh       # 建 ~/LAB3，下载 mav_comm 和仿真器，编译
```

- `setup_lab3.sh` 会下载仿真器 `lab3.zip`（31.8 MB）。如果太慢，可以在 Windows 浏览器里下载 `https://vnav.mit.edu/material/lab3.zip`，放到 `\\wsl$\Ubuntu-20.04\home\<用户名>\LAB3\vnav-builds\lab3.zip`，再运行一次 `setup_lab3.sh`。
- 以后改了 `codeforWindows` 里的代码，重新运行 `setup_lab3.sh` 即可同步到 `~/LAB3` 并重新编译。

## 3. 确认用的是显卡

```
glxinfo -B | grep -i renderer
```

- 显示 `D3D12 (你的显卡型号)`：正在用显卡。
- 显示 `llvmpipe`：是软件渲染，在 PowerShell 里运行 `wsl --update`，并更新显卡驱动。
- 笔记本有独显和核显时，想指定独显（以 NVIDIA 为例）：`echo 'export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA' >> ~/.bashrc`，然后重开终端。

## 4. 运行演示

```
~/LAB3/scripts/run_demo.sh            # 跟踪圆轨迹；hover = 悬停测试
```

会依次打开 Unity 仿真器窗口（左上角显示帧率）和 RViz，然后平稳起飞：从无人机当前的位置和朝向出发，3 s 爬升到 2 m，8 s 内螺旋展开到圆上（`smooth_start:=true`）。按 `Ctrl-C` 结束，所有程序会一起关掉。

- 仿真窗口更大：`SIM_W=1280 SIM_H=720 ~/LAB3/scripts/run_demo.sh`
- 画面有小幅抖动（电脑忙、仿真器渲染占满 CPU 时会出现）：`~/LAB3/scripts/run_demo.sh circle kr:=12`
- RViz 配置 `~/LAB3/rviz/lab3_demo.rviz`：无面板、60 fps。窗口可以用鼠标拖动、缩放，两个窗口左右并排即可。

## 5. 录屏

在 Windows 上录：

- `Win + Shift + R`：截图工具录屏，可以框选两个窗口所在的区域（Windows 11）。
- `Win + Alt + R`：Xbox Game Bar，录当前窗口。
- 或者用 OBS。

## 6. 测跟踪误差

不开窗口，自动起飞、记录 40 s 并输出误差：

```
source ~/LAB3/env.sh
~/LAB3/scripts/fly_launch.sh circle 40      # 用 launch 文件里的增益
~/LAB3/scripts/fly_launch.sh hover 15       # 悬停阶跃
~/LAB3/scripts/sim_ctl.sh stop
```

结果在 `~/LAB3/results/flight/`。增益在 `~/LAB3/vnav_ws/src/controller_pkg/launch/traj_tracking.launch`（`kx=10, kv=5, kr=30, komega=2.5`）；换了电脑合适的增益可能不同，可以用 `fly_test.sh <circle|hover> kx kv kr komega` 试别的增益。

## 常见问题

- **`/current_state` 没有数据**：`~/LAB3/scripts/sim_ctl.sh status` 看仿真器和 bridge 是否在跑；日志在 `~/LAB3/results/logs/`（`sim.log`、`bridge.log`）。
- **Unity 窗口黑屏或闪退**：先试软件渲染 `LIBGL_ALWAYS_SOFTWARE=1 ~/LAB3/scripts/run_demo.sh`，能跑就是显卡驱动或 WSL 版本的问题。
- **RViz 打不开**：`echo $DISPLAY` 应该有输出（WSLg 自动设置）；没有的话在 PowerShell 里 `wsl --update` 后 `wsl --shutdown`，再重开 Ubuntu。
