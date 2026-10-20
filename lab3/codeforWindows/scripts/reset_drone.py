#!/usr/bin/env python3
# 停桨并让无人机回到初始位置（无窗口模式下代替按 R）
import time
from tesse.env import Env
from tesse.msgs import PropSpeeds, Respawn
env = Env()
for _ in range(3):
    env.send(PropSpeeds(0, 0, 0, 0))
    time.sleep(0.05)
env.send(Respawn())
time.sleep(0.5)
env.send(PropSpeeds(0, 0, 0, 0))
print("reset done")
