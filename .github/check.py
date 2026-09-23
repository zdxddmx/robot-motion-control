# -*- coding: utf-8 -*-
"""
CI 检查：安全行为与平滑性不能被改坏。

单独写成文件（而不是内联 heredoc），因为 GitHub Actions 里
`run: python3 - <<'PY'` 的 heredoc 内容一旦缩进就会报 IndentationError。
"""
import pathlib
import re
import sys

failures = []


def check(cond, msg):
    if cond:
        print("  [PASS] " + msg)
    else:
        failures.append(msg)
        print("  [FAIL] " + msg)


# ---------------------------------------------------------------- 1. 单测
print("== 1. 单元测试 ==")
txt = pathlib.Path("build/test_output.txt").read_text(encoding="utf-8", errors="replace")
m = re.search(r"结果：(\d+) passed, (\d+) failed", txt)
check(m is not None, "能解析出测试结果")
if m:
    passed, failed = int(m.group(1)), int(m.group(2))
    print("      passed=%d failed=%d" % (passed, failed))
    check(failed == 0, "单元测试全部通过")
    check(passed >= 120, "用例数不少于 120（实际 %d）" % passed)

# 关键用例必须存在
print("\n== 2. 关键用例存在性 ==")
for key in [
    "步间最大位置跳变",          # 序列平滑性（修过的真 bug）
    "每拍最大位移",              # 关节运动非阶跃
    "S 曲线：起点加速度严格为 0",
    "梯形曲线：中段速度 = v_max",
    "v_max 不可行时仍能到位",    # 梯形退化边界（修过的算法问题）
    "急停是锁存的",
    "第一次超限仍在去抖",        # 去抖
    "指令含 NaN -> 拦截",        # 指令在下发前被拦
    "通信丢失 -> 切安全态",
    "CRC 过多 -> 保护",
]:
    check(key in txt, "关键用例：" + key)

# ---------------------------------------------------------------- 3. 演示
print("\n== 3. 安全演示行为 ==")
safety = pathlib.Path("build/demo_safety.txt").read_text(encoding="utf-8",
                                                        errors="replace")
for key, desc in [
    ("初始姿态异常（roll=1.40 rad）：启动被拒绝", "初始姿态异常必须拒绝启动"),
    ("初始电量 5%：启动被拒绝", "初始低电必须拒绝启动"),
    ("已检出（COMM_LOST）", "运行中通信丢失必须检出"),
    ("已检出（CRC）", "CRC 错误过多必须检出"),
    ("已检出（ATTITUDE）", "姿态异常必须检出"),
    ("跑 500 拍后仍拦截=是", "急停必须锁存"),
    ("去抖中，不立刻切安全态", "单次毛刺不应立刻切安全态"),
]:
    check(key in safety, desc)

print("\n== 4. 曲线演示 ==")
curves = pathlib.Path("build/demo_curves.txt").read_text(encoding="utf-8",
                                                         errors="replace")
for key in ("线性：最快", "梯形：加速-匀速-减速", "S 曲线：加速度连续"):
    check(key in curves, "曲线演示包含：" + key)

# ---------------------------------------------------------------- 汇总
print()
if failures:
    print("共 %d 项检查失败：" % len(failures))
    for f in failures:
        print("  - " + f)
    sys.exit(1)
print("全部检查通过。")
