/* motion.cpp — 插值、动作序列、安全、监控、仿真端口、控制器 */
#include "motion.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <sstream>

namespace motion {

/* =============================================================== Vec3 */
double Vec3::norm() const { return std::sqrt(x * x + y * y + z * z); }

bool Vec3::is_finite() const {
    return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
}

/* =============================================================== 插值
 * 三种曲线的取舍（现场说法）：
 *   LINEAR    —— 最快到位，但起停瞬间加速度无穷大，减速器会"咯噔"
 *   TRAPEZOID —— 加速-匀速-减速，工程上最常用；加速度有跳变但可接受
 *   SCURVE    —— 加速度连续（加加速度有界），冲击最小，适合人形/灵巧手
 */
static double clamp01(double v) { return v < 0.0 ? 0.0 : (v > 1.0 ? 1.0 : v); }
static double smoothstep(double s) { return s * s * (3.0 - 2.0 * s); }

TrajectoryPoint eval_segment(const TrajectorySegment &seg, double t) {
    TrajectoryPoint p;
    const double D = seg.duration > 1e-9 ? seg.duration : 1e-9;
    const double dq = seg.q_end - seg.q_start;

    if (t <= 0.0) {
        p.q = seg.q_start;
        p.dq = 0.0;
        p.ddq = 0.0;
        return p;
    }
    if (t >= D) {
        p.q = seg.q_end;
        p.dq = 0.0;
        p.ddq = 0.0;
        return p;
    }

    switch (seg.curve) {
    case Curve::LINEAR: {
        const double s = t / D;
        p.q = seg.q_start + dq * s;
        p.dq = dq / D;
        p.ddq = 0.0;
        break;
    }
    case Curve::TRAPEZOID: {
        /* 在**归一化**坐标下算：位移 s∈[0,1]，时间 u=t/D∈[0,1]，
         * 于是归一化速度 v_norm = v_real · D / |dq|。
         * 三角速度曲线的归一化峰值 = 2（中点达到）。
         *
         * 分三段（这是原实现漏掉的部分）：
         *   v_norm >= 2      速度上限不起作用 -> 三角曲线
         *   1 < v_norm < 2   真梯形：加速段 ta = 1 - 1/v_norm
         *   v_norm <= 1      不可行（连平均速度都达不到）-> 退化为三角并尽快走完
         */
        const double absdq = std::max(1e-12, std::fabs(dq));
        double v_norm = 0.0;
        if (seg.v_max > 0.0) v_norm = seg.v_max * D / absdq * std::fabs(dq) / absdq;
        /* 上式里 dq 已同时出现在分子分母，简化成： */
        v_norm = (seg.v_max > 0.0) ? (seg.v_max * D / absdq) : 0.0;

        if (v_norm <= 1.0 || v_norm >= 2.0) {
            /* 三角曲线（含不可行退化） */
            const double a = 4.0 / (D * D);
            const double s_n = t / D;
            if (s_n < 0.5) {
                p.q = seg.q_start + dq * (0.5 * a * t * t);
                p.dq = dq * (a * t);
                p.ddq = dq * a;
            } else {
                const double td = D - t;
                p.q = seg.q_end - dq * (0.5 * a * td * td);
                p.dq = dq * (a * td);
                p.ddq = -dq * a;
            }
            break;
        }
        const double u = t / D;
        const double ta = 1.0 - 1.0 / v_norm;      /* 归一化加速段时长 */
        const double a = v_norm / ta;              /* 归一化加速度 */
        double s_n, v_n, a_n;
        if (u < ta) {
            s_n = 0.5 * a * u * u;
            v_n = a * u;
            a_n = a;
        } else if (u < 1.0 - ta) {
            s_n = 0.5 * a * ta * ta + v_norm * (u - ta);
            v_n = v_norm;
            a_n = 0.0;
        } else {
            const double td = 1.0 - u;
            s_n = 1.0 - 0.5 * a * td * td;
            v_n = a * td;
            a_n = -a;
        }
        p.q = seg.q_start + dq * s_n;
        p.dq = dq * v_n / D;
        p.ddq = dq * a_n / (D * D);
        break;
    }
    case Curve::SCURVE: {
        /* 用五次多项式：s(0)=0,s(1)=1, 且两端一阶二阶导为 0
         *   s(u) = 10u³ - 15u⁴ + 6u⁵
         * 这是最简单且冲击最小的做法（加加速度连续）。 */
        const double u = t / D;
        const double u2 = u * u, u3 = u2 * u, u4 = u3 * u, u5 = u4 * u;
        const double s = 10.0 * u3 - 15.0 * u4 + 6.0 * u5;
        const double ds = (30.0 * u2 - 60.0 * u3 + 30.0 * u4) / D;
        const double dds = (60.0 * u - 180.0 * u2 + 120.0 * u3) / (D * D);
        p.q = seg.q_start + dq * s;
        p.dq = dq * ds;
        p.ddq = dq * dds;
        break;
    }
    }
    (void)clamp01;
    (void)smoothstep;
    return p;
}

const char *curve_name(Curve c) {
    switch (c) {
    case Curve::LINEAR: return "linear";
    case Curve::TRAPEZOID: return "trapezoid";
    case Curve::SCURVE: return "s-curve";
    }
    return "?";
}

bool is_valid_curve(int v) { return v >= 0 && v <= 2; }

/* =============================================================== Trajectory */
void Trajectory::clear() { segs_.clear(); }

void Trajectory::add(double q_target, double duration, Curve curve,
                     double v_max, double a_max) {
    TrajectorySegment s;
    s.q_start = segs_.empty() ? q_target : segs_.back().q_end;
    /* 第一段没有起点信息时，起点=终点（零位移），调用方应改用 start_from */
    s.q_end = q_target;
    s.duration = duration;
    s.curve = curve;
    s.v_max = v_max;
    s.a_max = a_max;
    segs_.push_back(s);
}

void Trajectory::start_from(double q0, double q_target, double duration,
                            Curve curve) {
    segs_.clear();
    TrajectorySegment s;
    s.q_start = q0;
    s.q_end = q_target;
    s.duration = duration;
    s.curve = curve;
    segs_.push_back(s);
}

double Trajectory::duration() const {
    double d = 0.0;
    for (const auto &s : segs_) d += s.duration;
    return d;
}

TrajectoryPoint Trajectory::sample(double t) const {
    if (segs_.empty()) return {};
    double acc = 0.0;
    for (const auto &s : segs_) {
        if (t <= acc + s.duration) return eval_segment(s, t - acc);
        acc += s.duration;
    }
    TrajectoryPoint p;
    p.q = segs_.back().q_end;
    return p;
}

double Trajectory::target() const {
    return segs_.empty() ? 0.0 : segs_.back().q_end;
}

/* =============================================================== Sequence */
void Sequence::clear() {
    steps_.clear();
    traj_.clear();
    prev_.clear();
    idx_ = 0;
    t_ = 0.0;
}

void Sequence::add(const SequenceStep &step) {
    steps_.push_back(step);
    traj_.emplace_back();
}

const SequenceStep *Sequence::current() const {
    return idx_ < steps_.size() ? &steps_[idx_] : nullptr;
}

double Sequence::step_progress() const {
    if (idx_ >= steps_.size()) return 1.0;
    const double d = steps_[idx_].duration;
    return d > 1e-9 ? std::min(1.0, t_ / d) : 1.0;
}

void Sequence::reset() {
    idx_ = 0;
    t_ = 0.0;
}

void Sequence::set_initial(const std::vector<JointCommand> &cur) {
    prev_ = cur;
}

bool Sequence::tick(double dt, std::vector<JointCommand> &out) {
    if (idx_ >= steps_.size()) return false;

    const SequenceStep &st = steps_[idx_];

    /* 进入本步：以**上一拍的实际输出**为起点建轨迹。
     * 用"实际输出"而不是"目标值"当起点很重要 —— 中途改目标或被打断时，
     * 从当前实际位置平滑接续，不会出现位置跳变（跳变会冲击减速器）。 */
    if (t_ <= 0.0) {
        if (st.on_enter) st.on_enter();
        traj_[idx_].assign(st.targets.size(), Trajectory{});
        for (size_t j = 0; j < st.targets.size(); ++j) {
            /* 没注入当前状态时，退化为"从目标出发"（零位移），
             * 而不是"从 0 跳到目标" —— 后者会产生阶跃。 */
            const double q0 = (j < prev_.size()) ? prev_[j].q : st.targets[j].q;
            traj_[idx_][j].start_from(q0, st.targets[j].q, st.duration, st.curve);
        }
    }

    const double t = t_;
    t_ += dt;

    out = st.targets;
    for (size_t j = 0; j < st.targets.size(); ++j) {
        const TrajectoryPoint p = traj_[idx_][j].sample(t);
        out[j].q = p.q;
        out[j].dq = p.dq;      /* 速度给下层做前馈，跟随更准 */
    }

    if (t_ >= st.duration) {
        prev_ = out;
        t_ = 0.0;
        idx_++;
        return idx_ < steps_.size();
    }
    return true;
}

/* =============================================================== 安全 */
const char *safety_event_name(SafetyEvent e) {
    switch (e) {
    case SafetyEvent::NONE: return "NONE";
    case SafetyEvent::JOINT_LIMIT: return "JOINT_LIMIT";
    case SafetyEvent::VELOCITY_LIMIT: return "VELOCITY_LIMIT";
    case SafetyEvent::TORQUE_LIMIT: return "TORQUE_LIMIT";
    case SafetyEvent::TEMPERATURE: return "TEMPERATURE";
    case SafetyEvent::ATTITUDE: return "ATTITUDE";
    case SafetyEvent::COMM_LOST: return "COMM_LOST";
    case SafetyEvent::CRC: return "CRC";
    case SafetyEvent::BATTERY: return "BATTERY";
    case SafetyEvent::ESTOP: return "ESTOP";
    case SafetyEvent::NOT_INITIALIZED: return "NOT_INITIALIZED";
    }
    return "?";
}

SafetyChecker::SafetyChecker(const SafetyLimits &lim) : lim_(lim) {}

SafetyReport SafetyChecker::pending_report(SafetyEvent e,
                                           const std::string &why) {
    SafetyReport r;
    r.ok = false;
    r.event = e;
    r.detail = why;
    return r;
}

void SafetyChecker::on_state_received(uint32_t crc_errors) {
    comm_age_ = 0.0;
    ever_received_ = true;
    crc_errors_ = crc_errors;
}

SafetyReport SafetyChecker::check_initial(const RobotState &st) const {
    if (st.joints.empty())
        return {false, SafetyEvent::NOT_INITIALIZED, "关节数量为 0（未收到过状态？）"};
    for (size_t i = 0; i < st.joints.size(); ++i) {
        const auto &j = st.joints[i];
        if (!j.valid)
            return {false, SafetyEvent::NOT_INITIALIZED,
                    "关节 " + std::to_string(i) + " 数据无效"};
        if (!std::isfinite(j.q) || !std::isfinite(j.dq))
            return {false, SafetyEvent::NOT_INITIALIZED,
                    "关节 " + std::to_string(i) + " 数值非法（NaN/Inf）"};
        if (i < jl_.size()) {
            const auto &L = jl_[i];
            if (j.q < L.pos_min - 1e-9 || j.q > L.pos_max + 1e-9)
                return {false, SafetyEvent::JOINT_LIMIT,
                        "关节 " + std::to_string(i) + " 初始位置超限位"};
        }
    }
    if (!st.imu_rpy.is_finite())
        return {false, SafetyEvent::NOT_INITIALIZED, "IMU 姿态非法"};
    if (std::fabs(st.imu_rpy.x) > lim_.roll_max ||
        std::fabs(st.imu_rpy.y) > lim_.pitch_max)
        return {false, SafetyEvent::ATTITUDE, "初始姿态异常（可能未放平）"};
    if (st.battery < lim_.battery_min)
        return {false, SafetyEvent::BATTERY, "初始电量过低"};
    return {};
}

SafetyReport SafetyChecker::tick(const RobotState &st, double dt,
                                 const std::vector<JointCommand> &cmd) {
    /* 急停是锁存的，优先级最高 */
    if (estop_) {
        if (history_.empty() || history_.back() != SafetyEvent::ESTOP)
            history_.push_back(SafetyEvent::ESTOP);
        return pending_report(SafetyEvent::ESTOP, "急停已触发（需显式解除）");
    }

    if (!ever_received_)
        return pending_report(SafetyEvent::COMM_LOST, "尚未收到过机器人状态");

    comm_age_ += dt;

    /* 逐个把"可疑"的事件找出来，再按严重度决定报哪个 */
    SafetyEvent found = SafetyEvent::NONE;
    std::string why;

    if (comm_age_ > lim_.comm_timeout_s) {
        found = SafetyEvent::COMM_LOST;
        char buf[96];
        std::snprintf(buf, sizeof(buf), "通信丢失 %.0f ms（阈值 %.0f ms）",
                      comm_age_ * 1000.0, lim_.comm_timeout_s * 1000.0);
        why = buf;
    }
    if (found == SafetyEvent::NONE && crc_errors_ > lim_.crc_error_max) {
        found = SafetyEvent::CRC;
        why = "CRC 错误累计 " + std::to_string(crc_errors_) + " 次";
    }
    if (found == SafetyEvent::NONE && st.battery < lim_.battery_min) {
        found = SafetyEvent::BATTERY;
        char buf[64];
        std::snprintf(buf, sizeof(buf), "电量过低 %.0f%%", st.battery * 100.0);
        why = buf;
    }
    if (found == SafetyEvent::NONE) {
        if (std::fabs(st.imu_rpy.x) > lim_.roll_max ||
            std::fabs(st.imu_rpy.y) > lim_.pitch_max) {
            found = SafetyEvent::ATTITUDE;
            char buf[96];
            std::snprintf(buf, sizeof(buf), "姿态异常 roll=%.2f pitch=%.2f（阈值 %.2f）",
                          st.imu_rpy.x, st.imu_rpy.y, lim_.roll_max);
            why = buf;
        } else if (st.imu_gyro.norm() > lim_.gyro_max) {
            found = SafetyEvent::ATTITUDE;
            why = "角速度过大（可能正在摔倒）";
        }
    }
    if (found == SafetyEvent::NONE) {
        for (size_t i = 0; i < st.joints.size(); ++i) {
            const auto &j = st.joints[i];
            if (j.temperature > lim_.temp_max) {
                found = SafetyEvent::TEMPERATURE;
                char buf[80];
                std::snprintf(buf, sizeof(buf), "关节 %zu 过温 %.0f℃", i, j.temperature);
                why = buf;
                break;
            }
            if (i < jl_.size()) {
                const auto &L = jl_[i];
                if (j.q < L.pos_min - 1e-9 || j.q > L.pos_max + 1e-9) {
                    found = SafetyEvent::JOINT_LIMIT;
                    char buf[96];
                    std::snprintf(buf, sizeof(buf),
                                  "关节 %zu 位置 %.3f 超限位 [%.3f, %.3f]",
                                  i, j.q, L.pos_min, L.pos_max);
                    why = buf;
                    break;
                }
                if (std::fabs(j.dq) > L.vel_max) {
                    found = SafetyEvent::VELOCITY_LIMIT;
                    char buf[80];
                    std::snprintf(buf, sizeof(buf), "关节 %zu 速度 %.2f 超限", i, j.dq);
                    why = buf;
                    break;
                }
                if (std::fabs(j.tau) > L.torque_max * 1.05) {
                    found = SafetyEvent::TORQUE_LIMIT;
                    char buf[80];
                    std::snprintf(buf, sizeof(buf), "关节 %zu 力矩 %.1f 超限（可能被撞）",
                                  i, j.tau);
                    why = buf;
                    break;
                }
            }
        }
    }
    /* 指令本身也要查：Kp/Kd 或目标位置非法会直接损坏硬件 */
    if (found == SafetyEvent::NONE && !cmd.empty()) {
        for (size_t i = 0; i < cmd.size(); ++i) {
            const auto &c = cmd[i];
            if (!std::isfinite(c.q) || !std::isfinite(c.kp) || !std::isfinite(c.kd)) {
                found = SafetyEvent::JOINT_LIMIT;
                why = "指令含 NaN/Inf（关节 " + std::to_string(i) + "）";
                break;
            }
            if (i < jl_.size()) {
                const auto &L = jl_[i];
                if (c.q < L.pos_min || c.q > L.pos_max) {
                    found = SafetyEvent::JOINT_LIMIT;
                    why = "指令目标超限位（关节 " + std::to_string(i) + "）";
                    break;
                }
                if (c.kp > L.kp_max || c.kd > L.kd_max) {
                    found = SafetyEvent::TORQUE_LIMIT;
                    why = "增益超上限（关节 " + std::to_string(i) + "）";
                    break;
                }
            }
        }
    }

    if (found == SafetyEvent::NONE) {
        pending_ = SafetyEvent::NONE;
        count_ = 0;
        return {};
    }

    /* 去抖：连续 N 拍同一类异常才确认 */
    if (pending_ == found) {
        count_++;
    } else {
        pending_ = found;
        count_ = 1;
    }
    if (count_ < lim_.fault_debounce) {
        SafetyReport r;
        r.ok = true;      /* 还在去抖中，先不切安全态 */
        r.event = found;
        r.detail = why + "（去抖 " + std::to_string(count_) + "/" +
                   std::to_string(lim_.fault_debounce) + "）";
        return r;
    }

    if (history_.empty() || history_.back() != found) history_.push_back(found);
    return pending_report(found, why);
}

/* =============================================================== Monitor */
void Monitor::register_channel(const std::string &name) {
    if (!has(name)) {
        ChannelStats s;
        s.name = name;
        ch_.push_back(s);
    }
}

ChannelStats *Monitor::find(const std::string &name) {
    for (auto &c : ch_)
        if (c.name == name) return &c;
    return nullptr;
}

bool Monitor::has(const std::string &name) const {
    for (const auto &c : ch_)
        if (c.name == name) return true;
    return false;
}

void Monitor::record(const std::string &name, double value, double stamp_s) {
    ChannelStats *c = find(name);
    if (!c) {
        register_channel(name);
        c = find(name);
    }
    if (c->last_stamp_s >= 0.0) {
        const double gap = stamp_s - c->last_stamp_s;
        if (gap > c->max_gap_s) c->max_gap_s = gap;
    }
    if (c->samples == 0) {
        c->min_v = c->max_v = value;
    } else {
        c->min_v = std::min(c->min_v, value);
        c->max_v = std::max(c->max_v, value);
    }
    c->mean = (c->mean * static_cast<double>(c->samples) + value) /
              static_cast<double>(c->samples + 1);
    c->last = value;
    c->samples++;
    c->last_stamp_s = stamp_s;
}

void Monitor::mark_gap(const std::string &name) {
    ChannelStats *c = find(name);
    if (c) c->gaps++;
}

const ChannelStats *Monitor::stats(const std::string &name) const {
    for (const auto &c : ch_)
        if (c.name == name) return &c;
    return nullptr;
}

std::vector<ChannelStats> Monitor::all_stats() const { return ch_; }

std::vector<std::string> Monitor::timed_out(double now_s) const {
    std::vector<std::string> out;
    for (const auto &c : ch_) {
        if (c.last_stamp_s < 0.0 || (now_s - c.last_stamp_s) > timeout_s_)
            out.push_back(c.name);
    }
    return out;
}

void Monitor::record_state(const RobotState &st, double stamp_s) {
    char buf[64];
    for (size_t i = 0; i < st.joints.size(); ++i) {
        std::snprintf(buf, sizeof(buf), "joint%d.q", static_cast<int>(i));
        record(buf, st.joints[i].q, stamp_s);
        std::snprintf(buf, sizeof(buf), "joint%d.tau", static_cast<int>(i));
        record(buf, st.joints[i].tau, stamp_s);
    }
    record("imu.roll", st.imu_rpy.x, stamp_s);
    record("imu.pitch", st.imu_rpy.y, stamp_s);
    record("imu.gyro_norm", st.imu_gyro.norm(), stamp_s);
    record("foot_force", st.foot_force.norm(), stamp_s);
    record("battery", st.battery, stamp_s);
}

/* =============================================================== SimPort */
SimPort::SimPort(size_t joint_count, double dt, uint32_t seed)
    : n_(joint_count), dt_(dt), seed_(seed ? seed : 1u) {
    state_.joints.resize(n_);
    last_cmd_.resize(n_);
    for (size_t i = 0; i < n_; ++i) {
        state_.joints[i].valid = true;
        state_.joints[i].q = 0.0;
        state_.joints[i].temperature = 28.0 + static_cast<double>(i % 4);
    }
}

uint32_t SimPort::rnd() {
    /* xorshift32：确定性伪随机，便于复现 */
    seed_ ^= seed_ << 13;
    seed_ ^= seed_ >> 17;
    seed_ ^= seed_ << 5;
    return seed_;
}

bool SimPort::connect() {
    connected_ = true;
    return true;
}

void SimPort::disconnect() { connected_ = false; }

bool SimPort::write_command(const std::vector<JointCommand> &cmd) {
    if (cmd.size() != n_) return false;      /* 尺寸不符：直接判失败，别让上层以为发成功了 */
    last_cmd_ = cmd;
    cmds_sent_++;
    return true;
}

void SimPort::step_physics(double dt) {
    /* 一阶跟随：q' = q + dt * (dq_cmd + kp_eff * (q_cmd - q))
     * 用 Kp/Kd 折算成一个等效时间常数，模拟"关节在增益作用下趋向目标"。 */
    for (size_t i = 0; i < n_; ++i) {
        auto &j = state_.joints[i];
        const auto &c = last_cmd_[i];
        const double kp_eff = c.kp > 0.0 ? std::min(c.kp, 200.0) / 100.0 : 0.0;
        const double kd_eff = c.kd > 0.0 ? std::min(c.kd, 20.0) / 20.0 : 0.0;
        const double dq = c.dq * kd_eff + kp_eff * (c.q - j.q) * 8.0;
        j.dq = dq;
        j.q += dq * dt;
        /* 力矩按弹簧-阻尼模型估算（用于触发力矩保护） */
        j.tau = c.kp * (c.q - j.q) - c.kd * j.dq;
        /* 温度缓慢上升，长时间大电流会累积 */
        j.temperature += std::fabs(j.tau) * dt * 2e-4;
    }
    tick_++;
    state_.stamp_us = static_cast<uint64_t>(tick_ * dt * 1e6);
}

bool SimPort::read_state(RobotState &out) {
    if (!connected_) return false;
    step_physics(dt_);
    if (state_timeout_) return false;                 // 模拟状态话题不发数据
    if (drop_rate_ > 0.0 && (rnd() % 1000) < static_cast<uint32_t>(drop_rate_ * 1000.0))
        return false;                                  // 模拟丢包
    if (crc_rate_ > 0.0 && (rnd() % 1000) < static_cast<uint32_t>(crc_rate_ * 1000.0))
        crc_err_++;                                    // 模拟 CRC 错
    state_.crc_errors = crc_err_;
    out = state_;
    return true;
}

bool SimPort::switch_service(const std::string &service) {
    service_log_.push_back(service);
    return true;
}

void SimPort::set_initial_pose(const std::vector<double> &q) {
    for (size_t i = 0; i < n_ && i < q.size(); ++i) {
        state_.joints[i].q = q[i];
        last_cmd_[i].q = q[i];
    }
}

void SimPort::set_attitude(const Vec3 &rpy, const Vec3 &gyro) {
    state_.imu_rpy = rpy;
    state_.imu_gyro = gyro;
}

/* =============================================================== 控制器 */
RobotController::RobotController(RobotPort &port, const ControlConfig &cfg)
    : port_(port), cfg_(cfg), safety_(), monitor_(0.5) {
    jl_.resize(64);   /* 默认给足；start() 里按实际关节数裁剪 */
}

bool RobotController::start() {
    if (!port_.connect()) return false;
    if (!port_.read_state(state_)) return false;

    jl_.resize(state_.joints.size());
    safety_.set_joint_limits(jl_);

    const SafetyReport r = safety_.check_initial(state_);
    if (!r.ok) {
        safety_.estop();
        return false;
    }
    if (!port_.switch_service("stand")) return false;
    service_ = "stand";
    cmd_.assign(state_.joints.size(), JointCommand{});
    for (size_t i = 0; i < cmd_.size(); ++i) {
        cmd_[i].q = state_.joints[i].q;
        cmd_[i].kp = cfg_.kp_default;
        cmd_[i].kd = cfg_.kd_default;
    }
    started_ = true;
    stand_by_ = true;
    return true;
}

void RobotController::stop() {
    if (started_) {
        port_.switch_service("damp");
        service_ = "damp";
        started_ = false;
    }
    port_.disconnect();
}

void RobotController::move_joints(const std::vector<double> &q_target,
                                  double duration, Curve curve) {
    if (q_target.size() != cmd_.size()) return;
    per_joint_.clear();
    per_joint_.resize(cmd_.size());
    for (size_t i = 0; i < cmd_.size(); ++i) {
        per_joint_[i].start_from(cmd_[i].q, q_target[i], duration, curve);
    }
    playing_ = false;
}

void RobotController::play(const Sequence &seq) {
    seq_ = seq;
    seq_.set_initial(cmd_);      /* 关键：以**当前实际指令**为起点，避免首拍跳变 */
    seq_.reset();
    playing_ = true;
}

bool RobotController::stand(double duration) {
    if (!port_.switch_service("stand")) return false;
    service_ = "stand";
    /* 站立 = 所有关节回零位（真实机器人这里会是"站姿目标"） */
    std::vector<double> zero(cmd_.size(), 0.0);
    move_joints(zero, duration, Curve::SCURVE);
    stand_by_ = true;
    return true;
}

bool RobotController::walk(double vx, double vy, double vyaw) {
    if (!port_.switch_service("walk")) return false;
    service_ = "walk";
    stand_by_ = false;
    /* 高层行走：把速度指令写进"行走指令缓存"，本体的步态控制器会消费它。
     * 这里把速度映射成髋/膝关节的周期性摆动（只是一个可运行的示意）， */
    walk_cmd_.assign(cmd_.size(), JointCommand{});
    for (size_t i = 0; i < cmd_.size(); ++i) {
        walk_cmd_[i].kp = cfg_.kp_default;
        walk_cmd_[i].kd = cfg_.kd_default;
        walk_cmd_[i].dq = (i % 2 == 0) ? vx : -vx;
        walk_cmd_[i].q = cmd_[i].q + vyaw * 0.1 * static_cast<double>((i % 3) - 1);
    }
    return true;
}

bool RobotController::follow_trajectory(const std::vector<Vec3> &path, double speed) {
    if (path.size() < 2) return false;
    if (!port_.switch_service("walk")) return false;
    service_ = "walk";
    (void)speed;
    /* 轨迹跟随：真实系统里是"给速度指令 + 本体里程计闭环"。
     * 这里转成关节点位序列，交给上层 Sequence 播。 */
    Sequence seq;
    std::vector<double> q = std::vector<double>(cmd_.size(), 0.0);
    for (size_t k = 1; k < path.size(); ++k) {
        SequenceStep st;
        st.name = "wp" + std::to_string(k);
        st.duration = 1.0;
        st.targets.assign(cmd_.size(), JointCommand{});
        for (size_t j = 0; j < cmd_.size(); ++j) {
            st.targets[j].kp = cfg_.kp_default;
            st.targets[j].kd = cfg_.kd_default;
            q[j] = 0.15 * path[k].x + 0.05 * path[k].y;
            st.targets[j].q = q[j];
        }
        seq.add(st);
    }
    play(seq);
    return true;
}

bool RobotController::damp() {
    if (!port_.switch_service("damp")) return false;
    service_ = "damp";
    stand_by_ = false;
    for (auto &c : cmd_) {
        c.kp = 0.0;
        c.kd = 1.0;
        c.dq = 0.0;
        c.tau = 0.0;
    }
    return true;
}

std::vector<JointCommand> RobotController::compute_commands() {
    /* 优先级：序列 > 关节轨迹 > 行走指令 > 保持 */
    if (playing_ && !seq_.empty()) {
        std::vector<JointCommand> out;
        if (!seq_.tick(cfg_.dt, out)) playing_ = false;
        return out;
    }
    const double t = sim_time_;
    for (size_t j = 0; j < per_joint_.size() && j < cmd_.size(); ++j) {
        if (per_joint_[j].empty()) continue;
        const TrajectoryPoint p = per_joint_[j].sample(t);
        cmd_[j].q = p.q;
        cmd_[j].dq = p.dq;
        cmd_[j].tau = 0.0;
    }
    if (service_ == "walk" && !walk_cmd_.empty()) return walk_cmd_;
    return cmd_;
}

bool RobotController::send_commands(const std::vector<JointCommand> &cmd,
                                    ControlResult &res) {
    if (!port_.write_command(cmd)) {
        res.ok = false;
        res.fault = SafetyEvent::COMM_LOST;
        res.detail = "指令下发失败";
        return false;
    }
    return true;
}

ControlResult RobotController::tick() {
    ControlResult res;
    res.ticks = ++ticks_;

    RobotState st = state_;
    if (port_.read_state(st)) {
        state_ = st;
        safety_.on_state_received(st.crc_errors);
        monitor_.record_state(st, sim_time_);
    } else {
        /* 本拍没有新数据：不算错，但要记 gap，让超时判定自然触发 */
        monitor_.mark_gap("imu.roll");
    }
    sim_time_ += cfg_.dt;

    const auto cmd = compute_commands();

    /* 安全检查在**下发之前**做 —— 指令本身不合法时绝不能发出去 */
    const SafetyReport sr = safety_.tick(state_, cfg_.dt, cmd);
    if (!sr.ok) {
        res.ok = false;
        res.fault = sr.event;
        res.detail = sr.detail;
        if (service_ != "damp") {
            port_.switch_service("damp");
            service_ = "damp";
            res.service_switched = true;
        }
        return res;
    }

    if (!send_commands(cmd, res)) return res;
    return res;
}

ControlResult RobotController::run(uint64_t n) {
    ControlResult last;
    for (uint64_t i = 0; i < n; ++i) {
        last = tick();
        if (!last.ok) break;
    }
    return last;
}

}  // namespace motion
