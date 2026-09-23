/* main.cpp — 端到端演示：站立 / 关节插值 / 动作序列 / 安全保护
 *
 *   ./motiondemo curves    三种插值曲线对比
 *   ./motiondemo stand     启动 + 站立 + 关节平滑运动
 *   ./motiondemo seq       动作序列播放（蹲起-摆臂-复位）
 *   ./motiondemo safety    安全保护（姿态异常 / 通信丢失 / CRC / 急停）
 *   ./motiondemo all       全部
 */
#include "motion.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

using namespace motion;

static void banner(const char *t) {
    std::printf("\n==============================================================\n");
    std::printf("%s\n", t);
    std::printf("==============================================================\n");
}

/* ------------------------------------------------------------ 曲线对比 */
static void demo_curves() {
    banner("场景一：三种插值曲线对比（位移 0 -> 1 rad，耗时 1 s）");

    struct Item { Curve c; const char *desc; };
    const Item items[3] = {
        { Curve::LINEAR,    "线性：最快，但起停速度突跳（冲击大）" },
        { Curve::TRAPEZOID, "梯形：加速-匀速-减速，工程最常用" },
        { Curve::SCURVE,    "S 曲线：加速度连续，冲击最小（人形/灵巧手首选）" },
    };

    for (const auto &it : items) {
        TrajectorySegment s;
        s.q_start = 0.0;
        s.q_end = 1.0;
        s.duration = 1.0;
        s.curve = it.c;
        if (it.c == Curve::TRAPEZOID) s.v_max = 1.5;

        std::printf("\n-- %s --\n", it.desc);
        std::printf("  %6s %9s %10s %10s\n", "t(s)", "q(rad)", "d q/d t", "d2q/dt2");
        double vmax = 0.0, amax = 0.0;
        for (int i = 0; i <= 10; ++i) {
            const double t = i / 10.0;
            const TrajectoryPoint p = eval_segment(s, t);
            vmax = std::max(vmax, std::fabs(p.dq));
            amax = std::max(amax, std::fabs(p.ddq));
            std::printf("  %6.1f %9.4f %10.4f %10.4f\n", t, p.q, p.dq, p.ddq);
        }
        std::printf("  峰值速度 %.3f rad/s   峰值加速度 %.3f rad/s^2\n", vmax, amax);
    }
}

/* ------------------------------------------------------------ 站立 */
static void demo_stand() {
    banner("场景二：启动 -> 初始状态检查 -> 站立 -> 关节平滑运动");

    SimPort port(12, 0.002, 1234);
    ControlConfig cfg;
    cfg.dt = 0.002;                 /* 500 Hz */
    RobotController rc(port, cfg);

    if (!rc.start()) {
        std::printf("启动失败（初始状态检查未通过）\n");
        return;
    }
    std::printf("启动成功：服务=%s，关节数=%zu，控制周期=%.1f ms\n",
                rc.service().c_str(), rc.state().joints.size(), cfg.dt * 1000.0);

    rc.run(50);
    std::printf("站立 0.1 s 后：\n");
    for (size_t i = 0; i < 4 && i < rc.state().joints.size(); ++i) {
        std::printf("  关节%zu  q=%+.4f rad  dq=%+.4f rad/s  tau=%+.3f N·m  %.0fC\n",
                    i, rc.state().joints[i].q, rc.state().joints[i].dq,
                    rc.state().joints[i].tau, rc.state().joints[i].temperature);
    }

    /* 让 12 个关节各自走到不同目标（模拟摆姿势） */
    std::vector<double> target(12);
    for (size_t i = 0; i < target.size(); ++i)
        target[i] = 0.25 * std::sin(static_cast<double>(i) * 0.5);

    rc.move_joints(target, 0.6, Curve::SCURVE);
    std::printf("\n下发关节目标（S 曲线插值 0.6 s），观察平滑性：\n");
    std::printf("  %8s %12s %12s\n", "t(ms)", "joint0.q", "本拍位移");
    double prev = rc.state().joints[0].q;
    double max_step = 0.0;
    for (int i = 1; i <= 300; ++i) {
        const ControlResult r = rc.tick();
        if (!r.ok) { std::printf("  第 %d 拍故障：%s\n", i, r.detail.c_str()); break; }
        const double q = rc.state().joints[0].q;
        max_step = std::max(max_step, std::fabs(q - prev));
        if (i % 30 == 0)
            std::printf("  %8.0f %12.5f %12.5f\n", i * cfg.dt * 1000.0, q, q - prev);
        prev = q;
    }
    std::printf("每拍最大位移 %.5f rad（阶跃会远大于此值）\n", max_step);
    std::printf("最终 joint0.q = %.5f（目标 %.5f）\n",
                rc.state().joints[0].q, target[0]);
    std::printf("监控通道数：%zu\n", rc.monitor().all_stats().size());
}

/* ------------------------------------------------------------ 序列 */
static void demo_sequence() {
    banner("场景三：动作序列播放（下蹲 -> 摆臂 -> 复位）");

    SimPort port(6, 0.002, 99);
    ControlConfig cfg;
    RobotController rc(port, cfg);
    if (!rc.start()) { std::printf("启动失败\n"); return; }
    rc.run(20);

    Sequence seq;
    struct Act { const char *name; double knee; double arm; double dur; };
    const Act acts[3] = {
        { "下蹲", -0.6,  0.3, 0.8 },
        { "摆臂", -0.2, -0.5, 0.6 },
        { "复位",  0.0,  0.0, 0.8 },
    };
    for (const auto &a : acts) {
        SequenceStep st;
        st.name = a.name;
        st.duration = a.dur;
        st.curve = Curve::SCURVE;
        st.targets.assign(6, JointCommand{});
        for (size_t j = 0; j < 6; ++j) {
            st.targets[j].kp = cfg.kp_default;
            st.targets[j].kd = cfg.kd_default;
            st.targets[j].q = (j % 2 == 0) ? a.knee : a.arm;
        }
        st.on_enter = [&a]() { std::printf("  [进入] %s（%.1f s）\n", a.name, a.dur); };
        seq.add(st);
    }

    rc.play(seq);
    std::printf("开始播放 %zu 步序列：\n", seq.size());

    int tick = 0;
    double last_step = -1.0;
    while (true) {
        const ControlResult r = rc.tick();
        if (!r.ok) { std::printf("  故障中止：%s\n", r.detail.c_str()); break; }
        tick++;
        const double prog = rc.monitor().has("joint0.q") ? 0.0 : 0.0;
        (void)prog;
        if (tick % 150 == 0) {
            std::printf("  t=%.2fs  joint0=%+.4f  joint1=%+.4f\n",
                        tick * cfg.dt, rc.state().joints[0].q, rc.state().joints[1].q);
        }
        if (tick > 3000) break;
        last_step = rc.state().joints[0].q;
    }
    std::printf("序列结束，joint0 最终 = %+.4f（目标 0.0）\n", last_step);
}

/* ------------------------------------------------------------ 安全 */
static void demo_safety() {
    banner("场景四：安全保护（这是实机上最要紧的部分）");

    auto fresh = [](SimPort &p) {
        ControlConfig cfg;
        RobotController *rc = new RobotController(p, cfg);
        return rc;
    };

    /* 1) 初始姿态异常 -> 拒绝启动 */
    {
        SimPort p(6, 0.002, 7);
        p.set_attitude(Vec3(1.4, 0.0, 0.0), Vec3(0, 0, 0));   /* 横滚 80° */
        ControlConfig cfg;
        RobotController rc(p, cfg);
        std::printf("① 初始姿态异常（roll=1.40 rad）：启动%s\n",
                    rc.start() ? "成功（不该！）" : "被拒绝");
    }
    /* 2) 初始低电 -> 拒绝启动 */
    {
        SimPort p(6, 0.002, 7);
        p.set_battery(0.05);
        ControlConfig cfg;
        RobotController rc(p, cfg);
        std::printf("② 初始电量 5%%：启动%s\n",
                    rc.start() ? "成功（不该！）" : "被拒绝");
    }
    /* 3) 运行中通信丢失 */
    {
        SimPort p(6, 0.002, 7);
        ControlConfig cfg;
        RobotController rc(p, cfg);
        rc.start();
        rc.run(50);
        p.set_state_timeout(true);
        const ControlResult r = rc.run(300);
        std::printf("③ 运行中通信丢失：%s（%s）-> 服务切到 %s\n",
                    r.ok ? "未检出（不该！）" : "已检出",
                    safety_event_name(r.fault), rc.service().c_str());
    }
    /* 4) CRC 错误过多 */
    {
        SimPort p(6, 0.002, 7);
        ControlConfig cfg;
        RobotController rc(p, cfg);
        rc.start();
        rc.run(20);
        p.set_crc_error_rate(1.0);
        const ControlResult r = rc.run(400);
        std::printf("④ CRC 错误累积：%s（%s）\n",
                    r.ok ? "未检出（不该！）" : "已检出", safety_event_name(r.fault));
    }
    /* 5) 姿态异常（摔倒前兆） */
    {
        SimPort p(6, 0.002, 7);
        ControlConfig cfg;
        RobotController rc(p, cfg);
        rc.start();
        rc.run(20);
        p.set_attitude(Vec3(0.0, 1.2, 0.0), Vec3(0, 0, 20.0));
        const ControlResult r = rc.run(200);
        std::printf("⑤ 姿态异常（pitch=1.20 rad）：%s（%s）\n",
                    r.ok ? "未检出（不该！）" : "已检出", safety_event_name(r.fault));
    }
    /* 6) 急停是锁存的 */
    {
        SimPort p(6, 0.002, 7);
        ControlConfig cfg;
        RobotController rc(p, cfg);
        rc.start();
        rc.run(20);
        rc.estop();
        const ControlResult a = rc.tick();
        rc.run(500);
        const ControlResult b = rc.tick();
        rc.clear_estop();
        const ControlResult c = rc.tick();
        std::printf("⑥ 急停：立即拦截=%s，跑 500 拍后仍拦截=%s（锁存），显式解除后恢复=%s\n",
                    (!a.ok && a.fault == SafetyEvent::ESTOP) ? "是" : "否",
                    (!b.ok && b.fault == SafetyEvent::ESTOP) ? "是" : "否",
                    c.ok ? "是" : "否");
    }
    /* 7) 去抖：单次毛刺不触发保护 */
    {
        SimPort p(6, 0.002, 7);
        ControlConfig cfg;
        RobotController rc(p, cfg);
        rc.start();
        rc.run(20);
        p.set_attitude(Vec3(0.0, 1.2, 0.0), Vec3(0, 0, 0));
        const ControlResult first = rc.tick();
        std::printf("⑦ 姿态异常第 1 拍：%s（去抖中，不立刻切安全态）\n",
                    first.ok ? "放行" : "已保护");
    }
    std::printf("\n说明：故障时控制器会先切到 damp（阻尼保护态）再返回错误，\n");
    std::printf("      而不是把错误往上抛了事 —— 现场要的是「先保人保机」再说。\n");
}

int main(int argc, char **argv) {
    const std::string which = (argc > 1) ? argv[1] : "all";
    std::printf("机器人运动控制与安全内核 — 演示\n");
    if (which == "all" || which == "curves") demo_curves();
    if (which == "all" || which == "stand") demo_stand();
    if (which == "all" || which == "seq") demo_sequence();
    if (which == "all" || which == "safety") demo_safety();
    std::printf("\n演示结束。\n");
    return 0;
}
