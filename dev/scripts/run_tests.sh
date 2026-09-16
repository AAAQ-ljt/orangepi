#!/usr/bin/env bash
# 在车上跑全部单元测试，输出简洁结果（车端一键自检）
#
# 用法： sudo bash /root/dev/scripts/run_tests.sh
#       sudo bash /root/dev/scripts/run_tests.sh test_start_gate   # 只跑某一个
set -uo pipefail

cd /root/dev || { echo "找不到 /root/dev"; exit 1; }
export PYTHONPATH=/root/dev

if [[ $# -gt 0 ]]; then
  files=()
  for name in "$@"; do
    files+=("tests/${name%.py}.py")
  done
else
  files=(tests/test_*.py)
fi

pass=0; fail=0
for f in "${files[@]}"; do
  [[ -f "$f" ]] || { echo "跳过（不存在）：$f"; continue; }
  if python3 "$f" >/tmp/_car_test_one.log 2>&1; then
    printf 'PASS  %s\n' "$f"
    pass=$((pass+1))
  else
    printf 'FAIL  %s\n' "$f"
    tail -4 /tmp/_car_test_one.log | sed 's/^/      /'
    fail=$((fail+1))
  fi
done
printf '==== 通过 %d / 失败 %d ====\n' "$pass" "$fail"
[[ $fail -eq 0 ]]
