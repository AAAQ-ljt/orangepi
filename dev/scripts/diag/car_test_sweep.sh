#!/usr/bin/env bash
# 车端逐测扫描：找出套件里失败的用例，并把它的**完整输出**留在 /tmp/one_<名字>.log
# 用法（root）：sudo bash /root/dev/scripts/diag/car_test_sweep.sh
cd /root/dev || exit 1
export PYTHONPATH=/root/dev
fail=0
for f in tests/test_*.py; do
  if python3 "$f" > "/tmp/one_$(basename "$f").log" 2>&1; then
    :
  else
    fail=$((fail+1)); echo "FAIL $f -> /tmp/one_$(basename "$f").log"
  fi
done
echo "SWEEP-DONE fail=$fail"
