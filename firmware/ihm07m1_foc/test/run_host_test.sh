#!/usr/bin/env bash
# foc.c 主机回归：重新生成黄金值 → 原生编译 → 跑比对。
# 用法：bash firmware/test/run_host_test.sh
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
cd "$HERE"
python3 gen_golden.py >/dev/null
cc -std=c11 -O2 -Wall -Wextra -I../include -I. \
   test_foc_host.c ../src/foc.c ../src/foc_sensorless.c ../src/param_id.c \
   -lm -o /tmp/test_foc_host
/tmp/test_foc_host
