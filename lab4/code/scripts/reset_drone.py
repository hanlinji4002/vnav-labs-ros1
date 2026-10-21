#!/usr/bin/env python3
# 停桨并让无人机回到 LAB4 出生点。带窗口时 Respawn 不起作用，所以再用 Reposition 直接放回去
import time
from tesse.env import Env
from tesse.msgs import PropSpeeds, Respawn, Reposition
env = Env()
for _ in range(3):
    env.send(PropSpeeds(0, 0, 0, 0))
    time.sleep(0.05)
env.send(Respawn())
time.sleep(0.5)
# spawn (0.40, 8.00, 0.22), yaw -166.3 deg; Unity frame: y up, q = (-qx, -qz, -qy, qw) of ROS
env.send(Reposition(0.40, 0.22, 8.00, 0.0, 0.9929119, 0.0, 0.1188524))
time.sleep(0.5)
env.send(PropSpeeds(0, 0, 0, 0))
print("reset done")
