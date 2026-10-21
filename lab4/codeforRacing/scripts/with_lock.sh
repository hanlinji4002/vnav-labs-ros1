#!/bin/bash
# 拿仿真器锁再执行命令：with_lock.sh <owner> <命令...>；锁被别的活进程占着就退出
LOCK=/tmp/tesse_sim.lock
if [ -f $LOCK ]; then
  read o p < $LOCK
  if kill -0 "$p" 2>/dev/null; then echo "simulator busy: $o $p"; exit 1; fi
fi
echo "$1 $$" > $LOCK
trap 'rm -f $LOCK' EXIT
shift
"$@"
