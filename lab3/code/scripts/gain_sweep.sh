#!/bin/bash
# 圆轨迹增益扫描，结果表写到 results/gain_sweep.txt
L=~/Desktop/LAB3
OUT=$L/results/gain_sweep.txt
[ -f $OUT ] || echo "label kx kv kr komega pos_err_mean pos_err_max radial height along yaw_deg speed" > $OUT
sed -i "/^done$/d" $OUT
run() {  # label kx kv kr komega
  n=sweep_$1
  grep -q "^$1 " $OUT && return   # already done (resume)
  $L/scripts/fly_test.sh circle $2 $3 $4 $5 40 $n > /dev/null 2>> $L/results/logs/gain_sweep_err.log || { echo "$1 FAILED" >> $OUT; return; }
  f=$L/results/flight/${n}_metrics.txt
  g() { grep "^$1:" $f | awk '{print $2}'; }
  echo "$1 $2 $3 $4 $5 $(g ss_pos_err_mean) $(g ss_pos_err_max) $(g ss_radial_err_mean) $(g ss_height_err_mean) $(g ss_along_err_mean) $(g ss_yaw_err_mean_deg) $(g ss_speed_mean)" >> $OUT
}
run base        6   4   16  2.5
run kx_low      3   4   16  2.5
run kx_high     10  4   16  2.5
run kv_low      6   2   16  2.5
run kv_high     6   6   16  2.5
run kr_low      6   4   8   2.5
run kr_high     6   4   30  2.5
run ko_low      6   4   16  1.5
run ko_high     6   4   16  4
run template    1   1   1   1
run fail_ko_hi  6   4   16  10
run fail_kr_lo  6   4   2   2.5
run fail_kv_lo  6   0.5 16  2.5
echo done >> $OUT
