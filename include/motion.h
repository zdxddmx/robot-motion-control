/*
 * motion.h — 人形/四足机器人运动控制与安全内核
 *
 * 对应「机器人二次开发工程师」的这几条职责：
 *   ③ 底层关节控制：关节位置/速度/力矩 + Kp/Kd，平滑插值、周期控制、动作序列
 *   ④ 状态数据处理：关节/IMU 状态、数据记录、异常检测、超时处理、运行监控
 *   ⑥ 实机安全机制：运动服务切换、初始状态检查、关节限位、姿态异常保护、
 *      通信丢失保护、CRC 校验、紧急停止
 *
 * 设计原则（都是被现场问题逼出来的）：
 *   · **周期控制**：所有输出都按固定控制周期算出，不依赖"什么时候被调用"
 *   · **插值先行**：上层给目标，下层负责平滑过渡，避免阶跃指令打坏减速器
 *   · **安全优先**：安全检查在**每一拍**都跑，任何一项不过就直接切到安全状态
 *   · **可测**：不依赖真实机器人、不依赖 DDS 库，仿真后端即可完整验证
 *
 * C++17，零第三方依赖。
 */
#ifndef MOTION_H
#define MOTION_H

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace motion {

/* =============================================================== 基础类型 */

/** 三维向量（位置/速度/姿态角都用它）。 */
struct Vec3 {
    double x = 0.0, y = 0.0, z = 0.0;

    Vec3() = default;
    Vec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}

    Vec3 operator+(const Vec3 &o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vec3 operator-(const Vec3 &o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vec3 operator*(double s) const { return {x * s, y * s, z * s}; }
    double norm() const;
    bool is_finite() const;
};

/** 关节限位。 */
struct JointLimit {
    double pos_min = -3.14, pos_max = 3.14;   // 位置限位 [rad]
    double vel_max = 20.0;                    // 速度上限 [rad/s]
    double torque_max = 60.0;                 // 力矩上限 [N·m]
    double kp_max = 500.0;                    // 刚度上限
    double kd_max = 20.0;                     // 阻尼上限
};

/** 单关节指令（对应 Unitree LowCmd 里一个电机的字段形态）。 */
struct JointCommand {
    double q = 0.0;          // 目标位置 [rad]
    double dq = 0.0;         // 目标速度 [rad/s]
    double tau = 0.0;        // 前馈力矩 [N·m]
    double kp = 0.0;         // 位置刚度
    double kd = 0.0;         // 速度阻尼
};

/** 单关节状态（对应 Unitree LowState 里一个电机的字段形态）。 */
struct JointState {
    double q = 0.0;          // 当前位置 [rad]
    double dq = 0.0;         // 当前速度 [rad/s]
    double tau = 0.0;        // 当前力矩 [N·m]
    double temperature = 25.0;  // 温度 [℃]
    bool valid = false;      // 本拍数据是否有效
};

/** 机器人整体状态。 */
struct RobotState {
    std::vector<JointState> joints;
    Vec3 imu_rpy;            // 姿态角（roll/pitch/yaw）[rad]
    Vec3 imu_gyro;           // 角速度 [rad/s]
    Vec3 imu_acc;            // 加速度 [m/s²]
    Vec3 foot_force;         // 足端力（四足）/ 支撑力（人形）[N]
    double battery = 1.0;    // 电量比例 0~1
    uint64_t stamp_us = 0;   // 时间戳 [us]
    uint8_t mode = 0;        // 机器人上报的模式
    uint32_t crc_errors = 0; // 累计 CRC 错误（用于判断通信质量）
};

/* =============================================================== 插值 */

/** 插值曲线类型。 */
enum class Curve {
    LINEAR,      // 线性：最快但加速度不连续
    TRAPEZOID,   // 梯形速度：加速度有跳变，工程上最常用
    SCURVE,      // S 曲线：加速度连续，冲击最小
};

/** 单关节轨迹段。 */
struct TrajectorySegment {
    double q_start = 0.0;
    double q_end = 0.0;
    double duration = 1.0;   // 段时长 [s]
    Curve curve = Curve::SCURVE;
    /** 梯形/S 曲线的最大速度、最大加速度（<=0 表示不限制，由时长决定） */
    double v_max = 0.0;
    double a_max = 0.0;
};

/**
 * 轨迹求值。
 * t 为段内时间（<=0 返回起点，>=duration 返回终点）。
 * 同时给出位置、速度、加速度 —— 下层的 Kp/Kd 需要速度做前馈。
 */
struct TrajectoryPoint {
    double q = 0.0;
    double dq = 0.0;
    double ddq = 0.0;
};

TrajectoryPoint eval_segment(const TrajectorySegment &seg, double t);

/** 把一串路点串成"每段定长时间"的轨迹，并支持按总时间求值。 */
class Trajectory {
public:
    void clear();
    /** 追加一段（上一段终点自动作为本段起点）。 */
    void add(double q_target, double duration, Curve curve = Curve::SCURVE,
             double v_max = 0.0, double a_max = 0.0);
    /** 从当前位置直接出发（用于运行时改目标）。 */
    void start_from(double q0, double q_target, double duration,
                    Curve curve = Curve::SCURVE);
    bool empty() const { return segs_.empty(); }
    double duration() const;                 // 总时长
    /** 按绝对时间求值（超过总时长则停在终点）。 */
    TrajectoryPoint sample(double t) const;
    double target() const;                   // 最终目标位置

private:
    std::vector<TrajectorySegment> segs_;
};

/* =============================================================== 动作序列 */

/** 序列中的一步：在一段时间内把关节走到目标，然后可执行一个回调。 */
struct SequenceStep {
    std::string name;
    double duration = 1.0;
    Curve curve = Curve::SCURVE;
    std::vector<JointCommand> targets;    // 与关节数一一对应（空表示保持）
    std::function<void()> on_enter;       // 进入本步时回调（如切模式、播报）
    bool hold_after = false;              // 本步结束后是否保持
};

/**
 * 动作序列（站位 → 行走 → 摆臂 → 复位 …）。
 *
 * 现场要点：
 *   · 序列要能**中途打断并安全退出**（收到急停时不能把后半段走完）
 *   · 每步都要有名字与进度，便于上位机显示与日志排查
 */
class Sequence {
public:
    void clear();
    void add(const SequenceStep &step);
    size_t size() const { return steps_.size(); }
    bool empty() const { return steps_.empty(); }

    const SequenceStep *current() const;
    size_t index() const { return idx_; }
    double step_progress() const;         // 当前步 0~1
    bool finished() const { return idx_ >= steps_.size(); }

    /** 推进 dt 秒，填入本拍该发的关节指令。返回 false 表示序列已结束。 */
    bool tick(double dt, std::vector<JointCommand> &out);

    /** 复位到开头。 */
    void reset();

    /**
     * 设置"进入序列前机器人当前所处的指令状态"。
     *
     * 不设的话，第一步的起点会退化成该步的目标值 —— 结果是第一拍就跳到目标，
     * 位置出现阶跃（测试里表现为 1.0 rad 的跳变）。
     * 实机上这会把减速器打坏，所以必须由调用方注入当前实际指令。
     */
    void set_initial(const std::vector<JointCommand> &cur);

private:
    std::vector<SequenceStep> steps_;
    size_t idx_ = 0;
    double t_ = 0.0;
    /** [步][关节] 的轨迹：每步每个关节一条，才能各自的起点不同 */
    std::vector<std::vector<Trajectory>> traj_;
    /** 上一拍输出的指令（下一步的起点从这里接，保证连续、不跳变） */
    std::vector<JointCommand> prev_;
};

/* =============================================================== 安全 */

enum class SafetyEvent : uint8_t {
    NONE = 0,
    JOINT_LIMIT,          // 关节超限位
    VELOCITY_LIMIT,       // 速度超限
    TORQUE_LIMIT,         // 力矩超限（可能撞到东西）
    TEMPERATURE,          // 关节过温
    ATTITUDE,             // 姿态异常（可能摔倒）
    COMM_LOST,            // 通信丢失（心跳超时）
    CRC,                  // CRC 校验失败过多
    BATTERY,              // 电量过低
    ESTOP,                // 紧急停止
    NOT_INITIALIZED,      // 初始状态检查未通过
};

const char *safety_event_name(SafetyEvent e);

/** 安全阈值。 */
struct SafetyLimits {
    double roll_max = 0.6;        // 允许的最大横滚 [rad]（约 34°）
    double pitch_max = 0.6;       // 允许的最大俯仰 [rad]
    double gyro_max = 12.0;       // 角速度上限 [rad/s]
    double temp_max = 80.0;       // 关节温度上限 [℃]
    double battery_min = 0.15;    // 最低电量
    double comm_timeout_s = 0.2;  // 通信心跳超时 [s]
    uint32_t crc_error_max = 10;  // 允许的累计 CRC 错误数
    uint32_t fault_debounce = 3;  // 连续多少拍异常才确认（去抖）
};

/** 安全检查结果。 */
struct SafetyReport {
    bool ok = true;
    SafetyEvent event = SafetyEvent::NONE;
    std::string detail;
};

/**
 * 安全检查器。
 *
 * 关键设计：**每一拍都全量检查**，且异常要**去抖**（连续 N 拍才确认）——
 * 因为传感器单次毛刺就急停会让机器人没法用，而漏掉真实异常会摔机器人，
 * 两头都不能偏。
 */
class SafetyChecker {
public:
    explicit SafetyChecker(const SafetyLimits &lim = {});

    void set_limits(const SafetyLimits &lim) { lim_ = lim; }
    const SafetyLimits &limits() const { return lim_; }
    void set_joint_limits(const std::vector<JointLimit> &jl) { jl_ = jl; }

    /** 初始状态检查：上电后、使能前必须过。 */
    SafetyReport check_initial(const RobotState &st) const;

    /** 每拍检查（dt 用于心跳超时判定）。 */
    SafetyReport tick(const RobotState &st, double dt,
                      const std::vector<JointCommand> &cmd);

    /** 触发急停 / 解除。急停是**锁存**的，必须显式解除。 */
    void estop() { estop_ = true; }
    void clear_estop() { estop_ = false; pending_ = SafetyEvent::NONE; count_ = 0; }
    bool estopped() const { return estop_; }

    /** 距上次收到状态数据过了多久（秒）。 */
    double comm_age() const { return comm_age_; }
    /** 把"收到一帧有效数据"告诉检查器（重置心跳与 CRC 统计）。 */
    void on_state_received(uint32_t crc_errors);
    /** 把"收到一帧 CRC 错"告诉检查器。 */
    void on_crc_error() { crc_errors_++; }

    const std::vector<SafetyEvent> &history() const { return history_; }
    void clear_history() { history_.clear(); }

private:
    SafetyReport pending_report(SafetyEvent e, const std::string &why);

    SafetyLimits lim_;
    std::vector<JointLimit> jl_;
    bool estop_ = false;
    SafetyEvent pending_ = SafetyEvent::NONE;
    uint32_t count_ = 0;
    double comm_age_ = 0.0;
    bool ever_received_ = false;
    uint32_t crc_errors_ = 0;
    std::vector<SafetyEvent> history_;
};

/* =============================================================== 运行监控 */

/** 单个通道的监控统计（用于"数据记录 + 异常检测 + 超时处理"）。 */
struct ChannelStats {
    std::string name;
    double last = 0.0;
    double min_v = 0.0;
    double max_v = 0.0;
    double mean = 0.0;
    uint64_t samples = 0;
    uint64_t gaps = 0;          // 本拍缺失次数
    double last_stamp_s = -1.0;
    double max_gap_s = 0.0;
};

/**
 * 运行监控：记录各通道的统计、检测超时与跳变。
 *
 * 与 SafetyChecker 的分工：
 *   · SafetyChecker 关心"会不会摔/会不会坏" -> 触发保护
 *   · Monitor 关心"数据本身正常吗" -> 记录与告警，供事后排查
 */
class Monitor {
public:
    explicit Monitor(double timeout_s = 0.5) : timeout_s_(timeout_s) {}

    void register_channel(const std::string &name);
    /** 记录一个值；stamp_s 为绝对时间。 */
    void record(const std::string &name, double value, double stamp_s);
    /** 本拍该通道未提供数据。 */
    void mark_gap(const std::string &name);

    bool has(const std::string &name) const;
    const ChannelStats *stats(const std::string &name) const;
    std::vector<ChannelStats> all_stats() const;

    /** 超时通道（距上次数据超过 timeout_s）。 */
    std::vector<std::string> timed_out(double now_s) const;

    /** 记录机器人整机状态（关节数、IMU、足端力）。 */
    void record_state(const RobotState &st, double stamp_s);

    void set_timeout(double s) { timeout_s_ = s; }
    double timeout() const { return timeout_s_; }

private:
    double timeout_s_;
    std::vector<ChannelStats> ch_;
    ChannelStats *find(const std::string &name);
};

/* =============================================================== 通信端口 */

/**
 * 机器人通信端口抽象。
 *
 * 职责⑤要求用 DDS Topic / 发布订阅 / Client API 与本体通信。
 * 这里把它抽象成接口，于是：
 *   · 真机：DDS 后端（Unitree SDK2 的 LowCmd/LowState 话题）
 *   · 仿真/单测：SimPort（可在毫秒内跑完几万拍，还能注入丢包与 CRC 错）
 */
class RobotPort {
public:
    virtual ~RobotPort() = default;
    virtual bool connect() = 0;
    virtual void disconnect() = 0;
    virtual bool connected() const = 0;
    /** 下发一帧关节指令。返回 false 表示发送失败。 */
    virtual bool write_command(const std::vector<JointCommand> &cmd) = 0;
    /** 读取最新一帧机器人状态。返回 false 表示本拍没有新数据（超时）。 */
    virtual bool read_state(RobotState &out) = 0;
    /** 请求切换到某个运动服务（如 stand / walk / damp）。 */
    virtual bool switch_service(const std::string &service) = 0;
    virtual std::string name() const = 0;
};

/** 仿真端口：内置一个"关节跟随"的被控对象，可注入丢包/CRC 错/延迟。 */
class SimPort : public RobotPort {
public:
    SimPort(size_t joint_count = 12, double dt = 0.002, uint32_t seed = 20240923);

    bool connect() override;
    void disconnect() override;
    bool connected() const override { return connected_; }
    bool write_command(const std::vector<JointCommand> &cmd) override;
    bool read_state(RobotState &out) override;
    bool switch_service(const std::string &service) override;
    std::string name() const override { return "sim"; }

    /* --- 注入用（测试/演示） --- */
    void set_drop_rate(double r) { drop_rate_ = r; }
    void set_crc_error_rate(double r) { crc_rate_ = r; }
    void set_state_timeout(bool v) { state_timeout_ = v; }
    void set_initial_pose(const std::vector<double> &q);
    void set_attitude(const Vec3 &rpy, const Vec3 &gyro);
    void set_battery(double b) { state_.battery = b; }
    /** 直接推进仿真（不经过 tick），用于构造特定场景。 */
    void step_physics(double dt);

    const RobotState &state() const { return state_; }
    const std::vector<std::string> &service_log() const { return service_log_; }
    uint64_t commands_sent() const { return cmds_sent_; }

private:
    size_t n_;
    double dt_;
    bool connected_ = false;
    double drop_rate_ = 0.0;
    double crc_rate_ = 0.0;
    bool state_timeout_ = false;
    RobotState state_;
    std::vector<JointCommand> last_cmd_;
    std::vector<std::string> service_log_;
    uint64_t cmds_sent_ = 0;
    uint64_t tick_ = 0;
    uint32_t seed_;
    uint32_t crc_err_ = 0;

    uint32_t rnd();
};

/* =============================================================== 控制器 */

/** 控制器配置。 */
struct ControlConfig {
    double dt = 0.002;              // 控制周期 [s]（500 Hz）
    double kp_default = 60.0;       // 默认位置刚度
    double kd_default = 2.0;        // 默认速度阻尼
    double tau_limit_ratio = 1.0;   // 力矩限幅相对 JointLimit 的比例
    uint32_t max_consecutive_faults = 3;
};

/** 一次 tick 的结果。 */
struct ControlResult {
    bool ok = true;
    SafetyEvent fault = SafetyEvent::NONE;
    std::string detail;
    uint64_t ticks = 0;
    bool service_switched = false;
};

/**
 * 机器人应用主控制器。
 *
 * 把"上层给目标 / 下层出指令 / 每拍做安全检查"这条主循环收在一个类里，
 * 上机与仿真走同一套代码 —— 这正是职责⑧里"仿真验证 -> 实机联调"的基础。
 */
class RobotController {
public:
    RobotController(RobotPort &port, const ControlConfig &cfg = {});

    bool start();                   // 连接 + 初始状态检查 + 切到站立
    void stop();                    // 切阻尼（安全态）后断开

    /** 设定关节目标（会走插值，不会阶跃）。 */
    void move_joints(const std::vector<double> &q_target, double duration,
                     Curve curve = Curve::SCURVE);
    /** 播放一段动作序列。 */
    void play(const Sequence &seq);
    /** 高层动作（对应职责②的站立/行走/速度控制等）。 */
    bool stand(double duration = 1.5);
    bool walk(double vx, double vy, double vyaw);
    bool follow_trajectory(const std::vector<Vec3> &path, double speed);
    bool damp();                    // 阻尼保护态

    /** 走一拍：读状态 -> 安全检查 -> 算指令 -> 下发。 */
    ControlResult tick();

    /** 跑 n 拍（仿真/测试用）。 */
    ControlResult run(uint64_t n_ticks);

    void estop() { safety_.estop(); }
    void clear_estop() { safety_.clear_estop(); }

    const RobotState &state() const { return state_; }
    const SafetyChecker &safety() const { return safety_; }
    const Monitor &monitor() const { return monitor_; }
    Monitor &monitor() { return monitor_; }
    const std::string &service() const { return service_; }
    bool stand_by() const { return stand_by_; }

private:
    bool send_commands(const std::vector<JointCommand> &cmd, ControlResult &res);
    std::vector<JointCommand> compute_commands();

    RobotPort &port_;
    ControlConfig cfg_;
    SafetyChecker safety_;
    Monitor monitor_;
    RobotState state_;
    std::vector<JointLimit> jl_;
    std::vector<JointCommand> cmd_;
    Trajectory traj_;                     // 关节空间的统一轨迹（按关节分别插值）
    std::vector<Trajectory> per_joint_;
    Sequence seq_;
    bool playing_ = false;
    std::string service_ = "idle";
    bool stand_by_ = false;
    bool started_ = false;
    uint64_t ticks_ = 0;
    double sim_time_ = 0.0;
    std::vector<JointCommand> walk_cmd_;
};

/* =============================================================== 工具 */
const char *curve_name(Curve c);
bool is_valid_curve(int v);

}  // namespace motion

#endif  // MOTION_H
