# VNAV Labs (ROS 1)

Labs of MIT 16.485 *Visual Navigation for Autonomous Vehicles*, done with ROS 1 Noetic on Ubuntu 20.04 and the MIT starter code (tag `VNAV2021`).

## LAB3: 3D Trajectory Following

Geometric tracking controller on SE(3) (Lee et al., 2010) for a simulated quadrotor that follows a circle of radius 5 m at 2.5 m/s in the TESSE simulator. Left: RViz; right: simulator window (preview at 2× speed, click for the full video).

[![LAB3 flight: RViz and the TESSE simulator](lab3/results/lab3_preview.gif)](lab3/results/lab3_rviz_sim_side_by_side.mp4)

Full video (40 s): [`lab3/results/lab3_rviz_sim_side_by_side.mp4`](lab3/results/lab3_rviz_sim_side_by_side.mp4)

With the final gains (k<sub>x</sub> = 10, k<sub>v</sub> = 5, k<sub>R</sub> = 30, k<sub>ω</sub> = 2.5) the drone settles on the circle 1.8 s after take-off and then tracks it with an 8.6 mm position error and a 2.35° heading lag; a hover step overshoots by 1 %.

| Folder | Content |
|---|---|
| [`lab3/lab3要求.md`](lab3/lab3要求.md) | Requirements of the lab |
| [`lab3/code`](lab3/code) | Code for an Apple-silicon Mac (Ubuntu 20.04 arm64 virtual machine, simulator through box64) |
| [`lab3/codeforWindows`](lab3/codeforWindows) | Code for Windows (WSL2 Ubuntu 20.04, x86-64) |
| [`lab3/deliverable12answer`](lab3/deliverable12answer) | Answers to Deliverables 1 and 2 |
| [`lab3/results`](lab3/results) | Flight video |

## LAB4

In progress.
