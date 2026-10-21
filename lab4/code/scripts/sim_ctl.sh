#!/bin/bash
# LAB4 仿真器 + bridge 管理：sim_ctl.sh start|stop|restart|status|ensure [window]
source ~/Desktop/LAB4/env.sh
L=~/Desktop/LAB4; G=$L/results/logs; mkdir -p $G
alive() { python3 $L/scripts/sim_ok.py ${1:-3}; }
master() { rostopic list > /dev/null 2>&1 || { setsid nohup roscore > $G/roscore.log 2>&1 < /dev/null & sleep 4; }; }
stop() {
  [ -f $G/bridge.pid ] && kill -INT -$(cat $G/bridge.pid) 2>/dev/null
  [ -f $G/sim.pid ] && kill $(cat $G/sim.pid) 2>/dev/null
  sleep 3
  [ -f $G/bridge.pid ] && kill -9 -$(cat $G/bridge.pid) 2>/dev/null
  [ -f $G/sim.pid ] && kill -9 $(cat $G/sim.pid) 2>/dev/null
  # bridge nodes left over from a crashed run
  B="src/tesse_ros_bridge/(tesse_ros_node|quadrotor_control_interface)[.]py"
  pkill -f "$B" 2>/dev/null; sleep 2; pkill -9 -f "$B" 2>/dev/null
  rm -f $G/bridge.pid $G/sim.pid
}
start() {
  # LAB3 的仿真器也用 9000 端口，同时只能开一个
  pgrep -f "lab3[.]x86_64" > /dev/null && { echo "LAB3 simulator is running, stop it first"; return 1; }
  master
  [ "$MODE" = window ] && export DISPLAY=:0 XAUTHORITY=/run/user/1000/gdm/Xauthority
  setsid nohup $L/scripts/run_sim.sh $MODE > $G/sim.log 2>&1 < /dev/null & echo $! > $G/sim.pid
  for i in $(seq 60); do ss -lun | grep -q ":9000 " && break; sleep 1; done
  sleep 8   # scene loading
  setsid nohup roslaunch tesse_ros_bridge tesse_quadrotor_bridge.launch $BRIDGE_ARGS > $G/bridge.log 2>&1 < /dev/null & echo $! > $G/bridge.pid
  alive 25 && echo "sim ok" || { echo "sim FAILED"; return 1; }
}
MODE=$2
case $1 in
  start) start ;;
  stop) stop; echo stopped ;;
  restart) stop; start ;;
  status) alive && echo "sim ok" || echo "sim down" ;;
  ensure) alive || { echo "sim down -> restart"; stop; start; } ;;
esac
