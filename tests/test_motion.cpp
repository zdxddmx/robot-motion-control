/* test_motion.cpp — 运动控制与安全内核单元测试（纯 C++17，无第三方框架）
 *
 *  T1 curve      三种插值曲线：端点、单调性、速度/加速度连续性
 *  T2 traj       多段轨迹拼接
 *  T3 seq        动作序列：步进、进度、回调、中途打断
 *  T4 safety     初始检查 + 每拍检查 + 去抖 + 各类保护 + 急停锁存
 *  T5 monitor    通道统计、超时、跳变
 *  T6 sim        仿真端口：跟随、丢包、CRC、状态超时
 *  T7 ctrl       控制器：启动、站立、行走、序列播放、故障切安全态
 */
#include "motion.h"

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

using namespace motion;

static int passed = 0, failed = 0;

#define CHECK(cond, ...) do {                          \
    if (cond) { passed++; std::printf("  [PASS] "); }   \
    else      { failed++; std::printf("  [FAIL] "); }   \
    std::printf(__VA_ARGS__); std::printf("\n");        \
} while (0)

static bool near(double a, double b, double tol = 1e-9) {
    return std::fabs(a - b) <= tol;
}

/* ------------------------------------------------------------------ T1 */
static void t1_curve() {
    std::printf("T1 插值曲线\n");

    const Curve cs[3] = { Curve::LINEAR, Curve::TRAPEZOID, Curve::SCURVE };
    for (Curve c : cs) {
        TrajectorySegment s;
        s.q_start = 0.0;
        s.q_end = 1.0;
        s.duration = 2.0;
        s.curve = c;

        TrajectoryPoint p0 = eval_segment(s, 0.0);
        TrajectoryPoint pe = eval_segment(s, 2.0);
        CHECK(near(p0.q, 0.0), "%s: 起点为 0", curve_name(c));
        CHECK(near(pe.q, 1.0), "%s: 终点为 1", curve_name(c));
        CHECK(near(p0.dq, 0.0) && near(pe.dq, 0.0), "%s: 两端速度为 0", curve_name(c));

        /* 越界应夹住 */
        CHECK(near(eval_segment(s, -5.0).q, 0.0), "%s: t<0 夹到起点", curve_name(c));
        CHECK(near(eval_segment(s, 99.0).q, 1.0), "%s: t>D 夹到终点", curve_name(c));

        /* 单调递增 */
        double last = -1e9;
        bool mono = true;
        for (int i = 0; i <= 200; ++i) {
            const double q = eval_segment(s, 2.0 * i / 200.0).q;
            if (q < last - 1e-12) mono = false;
            last = q;
        }
        CHECK(mono, "%s: 位置单调不减", curve_name(c));
    }

    /* S 曲线：加速度在两端为 0、在中点也为 0，峰值出现在 u≈0.211 / 0.789。
     * 所以不能拿中点当"中段峰值"，要扫一遍取最大值。 */
    TrajectorySegment sc;
    sc.q_start = 0.0; sc.q_end = 1.0; sc.duration = 1.0; sc.curve = Curve::SCURVE;
    double a_edge = 0.0, a_peak = 0.0;
    for (int i = 0; i <= 1000; ++i) {
        const double t = i / 1000.0;
        const double a = std::fabs(eval_segment(sc, t).ddq);
        if (t <= 0.002 || t >= 0.998) a_edge = std::max(a_edge, a);
        a_peak = std::max(a_peak, a);
    }
    /* S 曲线加速度的性质是"**从 0 严格起、连续上升**"，而不是"两端都接近 0"。
     * 它按 u 的一次方起（dds 在 u=0 处为 0），所以离开端点很近的地方就已有
     * 相当的值 —— 用 2% 的位置去比会显得很大。这里查端点本身。 */
    CHECK(near(eval_segment(sc, 0.0).ddq, 0.0, 1e-12),
          "S 曲线：起点加速度严格为 0");
    CHECK(near(eval_segment(sc, 1.0).ddq, 0.0, 1e-12),
          "S 曲线：终点加速度严格为 0");
    CHECK(a_edge < a_peak * 0.05,
          "S 曲线：最靠近两端 0.2%% 范围内加速度远小于峰值（%.4f vs %.4f）",
          a_edge, a_peak);
    CHECK(a_peak > 1.0, "S 曲线峰值加速度合理（%.3f）", a_peak);
    CHECK(std::fabs(eval_segment(sc, 0.5).ddq) < 1e-9,
          "S 曲线：中点加速度为 0（对称性）");

    /* 对照：线性曲线在起点**速度就突跳**到满值 —— 这正是它冲击大的原因 */
    TrajectorySegment lin0 = sc;
    lin0.curve = Curve::LINEAR;
    CHECK(near(eval_segment(lin0, 0.0).dq, 0.0),
          "线性曲线：t=0 速度为 0（端点定义）");
    CHECK(near(eval_segment(lin0, 1e-9).dq, 1.0, 1e-6),
          "线性曲线：刚离开起点速度即突跳到满值（%.4f）",
          eval_segment(lin0, 1e-9).dq);

    /* 线性曲线加速度为 0（但速度会突跳，这是它的缺点） */
    TrajectorySegment li = sc;
    li.curve = Curve::LINEAR;
    CHECK(near(eval_segment(li, 0.5).ddq, 0.0), "线性曲线：加速度恒为 0");
    CHECK(near(eval_segment(li, 0.5).dq, 1.0), "线性曲线：速度恒为 1（位移 1/时长 1）");

    /* 梯形：给一个**可行**的 v_max（要大于平均速度 1.0、小于三角峰值 2.0），
     * 中段速度应恰好等于 v_max。 */
    TrajectorySegment tr = sc;
    tr.curve = Curve::TRAPEZOID;
    tr.v_max = 1.5;
    const double v_mid = eval_segment(tr, 0.5).dq;
    CHECK(near(v_mid, 1.5, 1e-6), "梯形曲线：中段速度 = v_max (%.4f)", v_mid);

    /* v_max 太小（连平均速度都达不到）应退化为三角且仍然走完 */
    TrajectorySegment slow = sc;
    slow.curve = Curve::TRAPEZOID;
    slow.v_max = 0.5;
    CHECK(near(eval_segment(slow, 1.0).q, 1.0, 1e-9),
          "v_max 不可行时仍能到位（退化处理），终点 = %.6f",
          eval_segment(slow, 1.0).q);

    /* v_max 很大（超过三角峰值）也应正确走完 */
    TrajectorySegment fast = sc;
    fast.curve = Curve::TRAPEZOID;
    fast.v_max = 100.0;
    CHECK(near(eval_segment(fast, 1.0).q, 1.0, 1e-9), "v_max 过大时仍能到位");

    /* 负位移方向也要正确 */
    TrajectorySegment neg = sc;
    neg.q_start = 2.0; neg.q_end = -1.0;
    CHECK(near(eval_segment(neg, 0.5).q, 0.5, 1e-6), "反向运动：中点位置正确");

    /* 零位移不应出 NaN */
    TrajectorySegment zero = sc;
    zero.q_end = zero.q_start;
    const TrajectoryPoint zp = eval_segment(zero, 0.5);
    CHECK(std::isfinite(zp.q) && std::isfinite(zp.dq), "零位移不出 NaN");

    /* 时长为 0 不应除零 */
    TrajectorySegment instant = sc;
    instant.duration = 0.0;
    const TrajectoryPoint ip = eval_segment(instant, 0.0);
    CHECK(std::isfinite(ip.q) && std::isfinite(ip.dq), "零时长不出 NaN/Inf");
}

/* ------------------------------------------------------------------ T2 */
static void t2_traj() {
    std::printf("T2 多段轨迹\n");
    Trajectory t;
    t.start_from(0.0, 1.0, 1.0, Curve::SCURVE);
    t.add(2.0, 1.0, Curve::SCURVE);
    t.add(0.0, 2.0, Curve::TRAPEZOID);

    CHECK(t.duration() == 4.0, "总时长 = %.1f s", t.duration());
    CHECK(near(t.target(), 0.0), "最终目标 = %.3f", t.target());

    /* 采样：段边界处应连续（不能跳变） */
    const double at1m = t.sample(1.0 - 1e-6).q;
    const double at1p = t.sample(1.0 + 1e-6).q;
    CHECK(std::fabs(at1m - at1p) < 1e-4, "段边界连续（%.6f vs %.6f）", at1m, at1p);
    CHECK(near(t.sample(1.0).q, 1.0, 1e-6), "第一段末到目标 1.0");

    /* 超出总时长停在终点 */
    CHECK(near(t.sample(100.0).q, 0.0), "超出总时长停在终点");

    /* 空轨迹不应崩 */
    Trajectory e;
    CHECK(e.empty(), "空轨迹 empty() 为真");
    CHECK(near(e.sample(1.0).q, 0.0), "空轨迹采样返回 0");
    CHECK(near(e.target(), 0.0), "空轨迹 target 为 0");
}

/* ------------------------------------------------------------------ T3 */
static void t3_seq() {
    std::printf("T3 动作序列\n");

    Sequence seq;
    int enters = 0;
    for (int k = 0; k < 3; ++k) {
        SequenceStep st;
        st.name = "step" + std::to_string(k);
        st.duration = 0.5;
        st.targets.assign(2, JointCommand{});
        st.targets[0].q = 0.2 * (k + 1);
        st.targets[1].q = -0.2 * (k + 1);
        st.targets[0].kp = 50.0;
        st.on_enter = [&enters]() { enters++; };
        seq.add(st);
    }
    CHECK(seq.size() == 3, "序列步数 = %zu", seq.size());
    CHECK(!seq.finished(), "初始未结束");

    std::vector<JointCommand> out;
    int ticks = 0;
    bool alive = true;
    while (alive && ticks < 10000) {
        alive = seq.tick(0.002, out);
        ticks++;
    }
    CHECK(enters == 3, "每步 on_enter 各调用一次（%d）", enters);
    CHECK(seq.finished(), "走完后 finished() 为真");
    CHECK(ticks >= 750 && ticks <= 760, "1.5 s / 2 ms ≈ 750 拍（实际 %d）", ticks);

    /* 关键：步与步之间位置要连续（不能跳） */
    Sequence s2;
    for (int k = 0; k < 2; ++k) {
        SequenceStep st;
        st.duration = 0.2;
        st.targets.assign(1, JointCommand{});
        st.targets[0].q = (k == 0) ? 1.0 : -1.0;
        s2.add(st);
    }
    /* 必须先注入"当前状态"，否则首步起点无从得知（这是实机必须做的事） */
    std::vector<JointCommand> cur(1);
    cur[0].q = 0.0;
    s2.set_initial(cur);

    double prev_q = 0.0, max_jump = 0.0;
    std::vector<JointCommand> o;
    while (s2.tick(0.002, o)) {
        max_jump = std::max(max_jump, std::fabs(o[0].q - prev_q));
        prev_q = o[0].q;
    }
    CHECK(max_jump < 0.05, "步间最大位置跳变 = %.4f rad（应平滑）", max_jump);

    /* 空序列 */
    Sequence empty;
    CHECK(empty.empty(), "空序列 empty() 为真");
    CHECK(!empty.tick(0.002, o), "空序列 tick 返回 false");

    /* 复位后可重放 */
    seq.reset();
    CHECK(!seq.finished() && seq.index() == 0, "reset 后回到开头");
}

/* ------------------------------------------------------------------ T4 */
static void t4_safety() {
    std::printf("T4 安全机制\n");

    /* 造一个有 2 个关节的状态 */
    RobotState st;
    st.joints.resize(2);
    for (auto &j : st.joints) { j.valid = true; j.q = 0.0; j.dq = 0.0; j.tau = 0.0; }
    st.imu_rpy = Vec3(0, 0, 0);
    st.battery = 0.9;

    std::vector<JointLimit> jl(2);
    jl[0].pos_min = -1.0; jl[0].pos_max = 1.0; jl[0].vel_max = 5.0; jl[0].torque_max = 20.0;
    jl[1] = jl[0];

    SafetyLimits lim;
    lim.fault_debounce = 3;
    lim.comm_timeout_s = 0.05;

    SafetyChecker sc(lim);
    sc.set_joint_limits(jl);
    sc.on_state_received(0);

    /* 初始检查通过 */
    SafetyReport r = sc.check_initial(st);
    CHECK(r.ok, "初始状态检查通过");

    /* 关节无效 / NaN / 姿态异常 / 低电 */
    RobotState bad = st;
    bad.joints[1].valid = false;
    CHECK(!sc.check_initial(bad).ok, "关节数据无效 -> 不通过");
    bad = st; bad.joints[0].q = std::nan("");
    CHECK(!sc.check_initial(bad).ok, "关节 NaN -> 不通过");
    bad = st; bad.imu_rpy = Vec3(1.2, 0, 0);
    CHECK(sc.check_initial(bad).event == SafetyEvent::ATTITUDE, "初始姿态异常 -> ATTITUDE");
    bad = st; bad.battery = 0.05;
    CHECK(sc.check_initial(bad).event == SafetyEvent::BATTERY, "初始低电 -> BATTERY");
    bad = st; bad.joints.clear();
    CHECK(!sc.check_initial(bad).ok, "无关节 -> 不通过");

    /* 每拍检查：正常 */
    std::vector<JointCommand> cmd(2);
    cmd[0].kp = 50; cmd[1].kp = 50;
    CHECK(sc.tick(st, 0.002, cmd).ok, "正常一拍检查通过");

    /* 去抖：单次超限不应立刻判故障 */
    SafetyChecker s2(lim);
    s2.set_joint_limits(jl);
    s2.on_state_received(0);
    RobotState hit = st;
    hit.joints[0].q = 5.0;                 /* 远超限位 */
    SafetyReport a = s2.tick(hit, 0.002, cmd);
    CHECK(a.ok, "第一次超限仍在去抖（不切安全态）");
    s2.tick(hit, 0.002, cmd);
    SafetyReport c = s2.tick(hit, 0.002, cmd);
    CHECK(!c.ok && c.event == SafetyEvent::JOINT_LIMIT,
          "连续 3 次超限 -> 确认 JOINT_LIMIT（%s）", safety_event_name(c.event));

    /* 各类保护 */
    auto trip = [&](const RobotState &bad_st, const std::vector<JointCommand> &cm,
                    SafetyEvent want, const char *what) {
        SafetyChecker s(lim);
        s.set_joint_limits(jl);
        s.on_state_received(0);
        SafetyReport rr;
        for (int i = 0; i < 5; ++i) rr = s.tick(bad_st, 0.002, cm);
        CHECK(!rr.ok && rr.event == want, "%s -> %s（实际 %s）", what,
              safety_event_name(want), safety_event_name(rr.event));
    };

    RobotState t1 = st; t1.joints[0].dq = 99.0;
    trip(t1, cmd, SafetyEvent::VELOCITY_LIMIT, "速度超限");
    RobotState t2 = st; t2.joints[0].tau = 500.0;
    trip(t2, cmd, SafetyEvent::TORQUE_LIMIT, "力矩超限");
    RobotState t3 = st; t3.joints[0].temperature = 99.0;
    trip(t3, cmd, SafetyEvent::TEMPERATURE, "关节过温");
    RobotState t4 = st; t4.imu_rpy = Vec3(0, 1.5, 0);
    trip(t4, cmd, SafetyEvent::ATTITUDE, "姿态异常");
    RobotState t5 = st; t5.imu_gyro = Vec3(0, 0, 30.0);
    trip(t5, cmd, SafetyEvent::ATTITUDE, "角速度过大");
    RobotState t6 = st; t6.battery = 0.02;
    trip(t6, cmd, SafetyEvent::BATTERY, "低电量");

    /* 通信丢失：不喂新数据，让 comm_age 累积 */
    {
        SafetyChecker s(lim);
        s.set_joint_limits(jl);
        s.on_state_received(0);
        SafetyReport rr;
        for (int i = 0; i < 60; ++i) rr = s.tick(st, 0.002, cmd);   /* 120 ms > 50 ms */
        CHECK(!rr.ok && rr.event == SafetyEvent::COMM_LOST,
              "心跳超时 -> COMM_LOST（%s）", rr.detail.c_str());
    }

    /* CRC 错误过多 */
    {
        SafetyChecker s(lim);
        s.set_joint_limits(jl);
        s.on_state_received(0);
        for (int i = 0; i < 11; ++i) s.on_crc_error();
        SafetyReport rr;
        for (int i = 0; i < 5; ++i) rr = s.tick(st, 0.002, cmd);
        CHECK(!rr.ok && rr.event == SafetyEvent::CRC, "CRC 错误过多 -> CRC");
    }

    /* 指令非法（NaN / 目标超限 / 增益过大）—— 必须在**下发前**被拦住 */
    {
        SafetyChecker s(lim);
        s.set_joint_limits(jl);
        s.on_state_received(0);
        auto bad_cmd = cmd;
        bad_cmd[0].q = std::nan("");
        SafetyReport rr;
        for (int i = 0; i < 5; ++i) rr = s.tick(st, 0.002, bad_cmd);
        CHECK(!rr.ok, "指令含 NaN -> 拦截");

        SafetyChecker s3(lim);
        s3.set_joint_limits(jl);
        s3.on_state_received(0);
        auto over = cmd;
        over[0].q = 99.0;
        for (int i = 0; i < 5; ++i) rr = s3.tick(st, 0.002, over);
        CHECK(!rr.ok && rr.event == SafetyEvent::JOINT_LIMIT, "指令目标超限 -> 拦截");

        SafetyChecker s4(lim);
        s4.set_joint_limits(jl);
        s4.on_state_received(0);
        auto gain = cmd;
        gain[0].kp = 1e6;
        for (int i = 0; i < 5; ++i) rr = s4.tick(st, 0.002, gain);
        CHECK(!rr.ok, "增益超上限 -> 拦截");
    }

    /* 急停是锁存的：即使状态完全正常也不放行，必须显式解除 */
    {
        SafetyChecker s(lim);
        s.set_joint_limits(jl);
        s.on_state_received(0);
        CHECK(s.tick(st, 0.002, cmd).ok, "急停前正常");
        s.estop();
        SafetyReport rr = s.tick(st, 0.002, cmd);
        CHECK(!rr.ok && rr.event == SafetyEvent::ESTOP, "急停后立刻拦截");
        for (int i = 0; i < 100; ++i) s.tick(st, 0.002, cmd);
        CHECK(!s.tick(st, 0.002, cmd).ok, "急停是锁存的（不会自己恢复）");
        s.clear_estop();
        CHECK(s.tick(st, 0.002, cmd).ok, "显式解除后恢复");
    }

    /* 从未收到状态 */
    {
        SafetyChecker s(lim);
        SafetyReport rr = s.tick(st, 0.002, cmd);
        CHECK(!rr.ok && rr.event == SafetyEvent::COMM_LOST, "从未收到状态 -> COMM_LOST");
    }
}

/* ------------------------------------------------------------------ T5 */
static void t5_monitor() {
    std::printf("T5 运行监控\n");
    Monitor m(0.3);
    m.record("a", 1.0, 0.0);
    m.record("a", 3.0, 0.1);
    m.record("a", 2.0, 0.2);

    const ChannelStats *s = m.stats("a");
    CHECK(s != nullptr, "通道已注册");
    CHECK(s->samples == 3, "采样数 = %llu", (unsigned long long)s->samples);
    CHECK(near(s->min_v, 1.0) && near(s->max_v, 3.0), "min/max 正确 (%.2f/%.2f)",
          s->min_v, s->max_v);
    CHECK(near(s->mean, 2.0), "均值 = %.3f", s->mean);
    CHECK(near(s->last, 2.0), "最新值 = %.3f", s->last);

    CHECK(m.has("a") && !m.has("nope"), "has() 正确");
    CHECK(m.stats("nope") == nullptr, "未知通道返回 nullptr");

    /* 超时：0.35 - 0.2 = 0.15 < 0.3 未超时；0.6 - 0.2 = 0.4 > 0.3 超时 */
    CHECK(m.timed_out(0.35).empty(), "未超时");
    auto to = m.timed_out(0.6);
    CHECK(to.size() == 1 && to[0] == "a", "超时通道：%zu 个", to.size());

    /* gap 统计 */
    m.mark_gap("a");
    m.mark_gap("a");
    CHECK(m.stats("a")->gaps == 2, "缺失计数 = %llu",
          (unsigned long long)m.stats("a")->gaps);
    m.mark_gap("nope");       /* 未知通道不应崩 */
    CHECK(true, "对未知通道 mark_gap 不崩");

    /* 记录整机状态 */
    Monitor m2;
    RobotState st;
    st.joints.resize(4);
    for (auto &j : st.joints) { j.q = 0.1; j.tau = 1.0; }
    st.imu_rpy = Vec3(0.1, 0.2, 0.0);
    st.battery = 0.8;
    m2.record_state(st, 0.0);
    CHECK(m2.has("joint0.q") && m2.has("joint3.tau"), "关节通道已建");
    CHECK(m2.has("imu.roll") && m2.has("battery"), "IMU 与电量通道已建");
    CHECK(m2.all_stats().size() == 4 * 2 + 5, "通道总数 = %zu", m2.all_stats().size());

    /* 自适应注册 */
    Monitor m3;
    m3.record("dynamic", 1.0, 0.0);
    CHECK(m3.has("dynamic"), "record 未注册的通道会自动注册");
}

/* ------------------------------------------------------------------ T6 */
static void t6_sim() {
    std::printf("T6 仿真端口\n");
    SimPort p(4, 0.002);
    CHECK(!p.connected(), "初始未连接");
    CHECK(p.connect(), "connect 成功");
    CHECK(p.connected(), "connected() 为真");

    RobotState st;
    CHECK(p.read_state(st), "读到状态");
    CHECK(st.joints.size() == 4, "关节数 = %zu", st.joints.size());

    /* 下发指令 -> 物理推进 -> 关节应朝目标移动 */
    std::vector<JointCommand> cmd(4);
    for (auto &c : cmd) { c.q = 0.5; c.kp = 60.0; c.kd = 2.0; }
    p.write_command(cmd);
    const double q0 = p.state().joints[0].q;
    p.step_physics(0.002);
    CHECK(p.state().joints[0].q > q0, "关节朝目标移动（%.5f -> %.5f）",
          q0, p.state().joints[0].q);
    CHECK(p.commands_sent() == 1, "指令计数 = %llu",
          (unsigned long long)p.commands_sent());

    /* 尺寸不匹配的指令应被忽略 */
    std::vector<JointCommand> wrong(2);
    p.write_command(wrong);
    CHECK(p.commands_sent() == 1, "尺寸不匹配的指令被忽略");

    /* 丢包 */
    SimPort pd(4, 0.002, 7);
    pd.connect();
    pd.set_drop_rate(1.0);
    CHECK(!pd.read_state(st), "丢包率 100% -> 读不到");

    /* 状态超时 */
    SimPort pt(4, 0.002, 7);
    pt.connect();
    pt.set_state_timeout(true);
    CHECK(!pt.read_state(st), "状态超时 -> 读不到");

    /* CRC 错累积 */
    SimPort pc(4, 0.002, 7);
    pc.connect();
    pc.set_crc_error_rate(1.0);
    for (int i = 0; i < 5; ++i) pc.read_state(st);
    CHECK(pc.state().crc_errors >= 4, "CRC 错误累积 = %u", pc.state().crc_errors);

    /* 服务切换记录 */
    SimPort ps(4, 0.002);
    ps.connect();
    ps.switch_service("stand");
    ps.switch_service("walk");
    CHECK(ps.service_log().size() == 2, "服务切换记录 %zu 条",
          ps.service_log().size());

    /* 未连接时读不到 */
    SimPort pu(4, 0.002);
    CHECK(!pu.read_state(st), "未连接 -> 读不到");
}

/* ------------------------------------------------------------------ T7 */
static void t7_ctrl() {
    std::printf("T7 控制器\n");

    SimPort port(6, 0.002, 42);
    ControlConfig cfg;
    cfg.dt = 0.002;
    RobotController c(port, cfg);

    CHECK(c.start(), "启动成功（连接 + 初始检查 + 切 stand）");
    CHECK(port.connected(), "端口已连接");
    CHECK(c.service() == "stand", "服务 = %s", c.service().c_str());
    CHECK(c.stand_by(), "处于站姿");

    /* 正常跑一段 */
    ControlResult r = c.run(200);
    CHECK(r.ok, "跑 200 拍无故障");
    CHECK(r.ticks == 200, "拍数 = %llu", (unsigned long long)r.ticks);

    /* 关节运动：应走插值而不是阶跃 */
    std::vector<double> target(6, 0.4);
    c.move_joints(target, 0.4, Curve::SCURVE);
    double max_step = 0.0, prev = c.state().joints[0].q;
    for (int i = 0; i < 250; ++i) {
        c.tick();
        const double q = c.state().joints[0].q;
        max_step = std::max(max_step, std::fabs(q - prev));
        prev = q;
    }
    CHECK(max_step < 0.02, "每拍最大位移 %.5f rad（平滑，非阶跃）", max_step);
    CHECK(std::fabs(prev - 0.4) < 0.05, "最终接近目标（%.4f）", prev);

    /* 高层动作 */
    CHECK(c.stand(0.5), "stand() 成功");
    CHECK(c.walk(0.3, 0.0, 0.0), "walk() 成功");
    CHECK(c.service() == "walk", "服务切到 walk");

    std::vector<Vec3> path = { Vec3(0, 0, 0), Vec3(1, 0, 0), Vec3(2, 0, 0) };
    CHECK(c.follow_trajectory(path, 0.5), "follow_trajectory() 成功");
    CHECK(c.damp(), "damp() 成功");
    CHECK(c.service() == "damp", "服务切到 damp");

    /* 单元素路径应被拒 */
    std::vector<Vec3> short_path = { Vec3(0, 0, 0) };
    CHECK(!c.follow_trajectory(short_path, 0.5), "路径点不足 -> 拒绝");

    /* 急停：应切到 damp 并持续拦截 */
    RobotController c2(port, cfg);
    port.disconnect();
    CHECK(c2.start(), "控制器 2 启动");
    c2.run(10);
    c2.estop();
    ControlResult er = c2.tick();
    CHECK(!er.ok && er.fault == SafetyEvent::ESTOP, "急停后 tick 报 ESTOP");
    CHECK(c2.service() == "damp", "急停后切到安全态 damp");
    c2.clear_estop();
    CHECK(c2.tick().ok, "解除急停后恢复");

    /* 启动时初始检查不过 -> start 失败 */
    {
        SimPort p2(4, 0.002, 3);
        p2.set_attitude(Vec3(1.5, 0, 0), Vec3(0, 0, 0));   /* 姿态异常 */
        RobotController c3(p2, cfg);
        CHECK(!c3.start(), "初始姿态异常 -> 启动失败");
    }
    {
        SimPort p3(4, 0.002, 3);
        p3.set_battery(0.01);
        RobotController c4(p3, cfg);
        CHECK(!c4.start(), "初始低电 -> 启动失败");
    }

    /* 运行中通信丢失 -> 切安全态 */
    {
        SimPort p4(4, 0.002, 5);
        RobotController c5(p4, cfg);
        CHECK(c5.start(), "启动");
        CHECK(c5.run(50).ok, "正常跑");
        p4.set_state_timeout(true);          /* 通信丢失 */
        ControlResult lr = c5.run(300);
        CHECK(!lr.ok && lr.fault == SafetyEvent::COMM_LOST,
              "通信丢失 -> 切安全态（%s）", safety_event_name(lr.fault));
        CHECK(c5.service() == "damp", "已切到 damp");
        CHECK(lr.service_switched, "标记了服务切换");
    }

    /* 运行中 CRC 错误过多 -> 保护 */
    {
        SimPort p5(4, 0.002, 9);
        RobotController c6(p5, cfg);
        c6.start();
        c6.run(10);
        p5.set_crc_error_rate(1.0);
        ControlResult lr = c6.run(400);
        CHECK(!lr.ok && lr.fault == SafetyEvent::CRC, "CRC 过多 -> 保护（%s）",
              safety_event_name(lr.fault));
    }

    /* 运行中姿态异常 -> 保护 */
    {
        SimPort p6(4, 0.002, 11);
        RobotController c7(p6, cfg);
        c7.start();
        c7.run(10);
        p6.set_attitude(Vec3(0, 1.2, 0), Vec3(0, 0, 0));
        ControlResult lr = c7.run(100);
        CHECK(!lr.ok && lr.fault == SafetyEvent::ATTITUDE, "姿态异常 -> 保护");
    }

    /* 监控在跑：应记录到通道 */
    CHECK(c2.monitor().has("battery"), "监控记录了 battery 通道");
    CHECK(c2.monitor().has("joint0.q"), "监控记录了 joint0.q 通道");
    CHECK(c2.monitor().all_stats().size() > 5, "监控通道数 = %zu",
          c2.monitor().all_stats().size());

    /* 停止 */
    c.stop();
    CHECK(!port.connected(), "stop() 后断开");
}

int main() {
    std::printf("==== 机器人运动控制与安全内核 单元测试 ====\n\n");
    t1_curve();
    t2_traj();
    t3_seq();
    t4_safety();
    t5_monitor();
    t6_sim();
    t7_ctrl();
    std::printf("\n==== 结果：%d passed, %d failed ====\n", passed, failed);
    return failed ? 1 : 0;
}
