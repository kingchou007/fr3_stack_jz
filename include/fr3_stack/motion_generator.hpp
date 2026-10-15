// Cartesian motion generators — sit between an external command source and
// the impedance controller. Each tick the active generator produces a target
// pose for the controller to track.
//
// Why this layer exists:
//   - 1 kHz controller needs a setpoint every tick; user commands arrive at
//     10–30 Hz (policies) or as discrete "go to pose" events.
//   - PassThrough turns sparse external setpoints into a continuous stream by
//     holding the last value (zero-order hold). Low-rate policy use case.
//   - MinJerk turns a (goal, duration) request into a smooth time-parameterized
//     trajectory. Reset / move-to-pose use case.
//
// RT-safety: all step() work is fixed-size math (no allocation, no I/O).
// Construction (std::make_unique) happens on the cmd thread; ownership is
// handed to the RT thread via std::move under the pending mutex.

#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>

struct CartesianTarget {
    Eigen::Vector3d    pos{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond quat{Eigen::Quaterniond::Identity()};
};

class TargetGenerator {
 public:
    virtual ~TargetGenerator() = default;
    // Snapshot the current pose as the trajectory's starting point. Called
    // from the RT thread when the generator first becomes active, so it has
    // access to the live robot state.
    virtual void start(const Eigen::Vector3d& pos0,
                       const Eigen::Quaterniond& quat0) = 0;
    // Advance the trajectory by dt seconds and return the current setpoint.
    virtual CartesianTarget step(double dt) = 0;
    // True once the trajectory has reached its goal. PassThrough never
    // finishes; MinJerk finishes after run_time elapsed.
    virtual bool finished() const = 0;
    // Called once right after start() with the base-frame end-effector
    // Jacobian at the start configuration. Generators with a joint-speed
    // bound may lengthen their duration here; the default does nothing.
    virtual void limit_joint_speed(const Eigen::Matrix<double, 6, 7>& /*J*/) {}
};

// Zero-order hold on the last externally-set target. Use for ML policies that
// stream Cartesian setpoints at their own (typically 10–30 Hz) rate. The
// 1 kHz control loop just keeps replaying whatever target was last received,
// while the impedance controller's LP filter handles the remaining smoothness.
class PassThroughGenerator : public TargetGenerator {
 public:
    void set_target(const Eigen::Vector3d& pos, const Eigen::Quaterniond& quat) {
        target_.pos  = pos;
        target_.quat = quat.normalized();
    }
    void start(const Eigen::Vector3d& pos0,
               const Eigen::Quaterniond& quat0) override {
        // Anchor at the current pose so the very first tick (before any
        // external set_target() call) doesn't jump to whatever was in the
        // member's default-initialized state.
        target_.pos  = pos0;
        target_.quat = quat0.normalized();
    }
    CartesianTarget step(double /*dt*/) override { return target_; }
    bool finished() const override { return false; }

 private:
    CartesianTarget target_;
};

// 5th-order min-jerk polynomial in position; SLERP in orientation.
//
// Basis: a(s) = 10s³ - 15s⁴ + 6s⁵, with s = t / run_time ∈ [0, 1].
// At s=0,1 the function value is 0,1 and the first two derivatives vanish —
// so position, velocity AND acceleration are continuous at both endpoints.
//
// run_time must be chosen by the caller to honor Cartesian velocity /
// acceleration limits (peak |v| = 1.875 · Δp / T, peak |a| = 5.7735 · Δp / T²).
// For a 50 cm move at 1 m/s peak, T ≈ 0.94 s; round up for safety.
//
// q_max_speed > 0 additionally bounds the peak joint speed, as FrankaTwin's
// move_to does: limit_joint_speed() estimates Δq ≈ J⁺·Δx at the start pose
// and stretches run_time to at least 1.875 · max|Δq| / q_max_speed. The
// estimate is a first-order linearization, so it is approximate for large
// moves. run_time is never shortened.
class MinJerkGenerator : public TargetGenerator {
 public:
    MinJerkGenerator(const Eigen::Vector3d& goal_pos,
                     const Eigen::Quaterniond& goal_quat,
                     double run_time,
                     double q_max_speed = 0.0)
        : goal_pos_(goal_pos),
          goal_quat_(goal_quat.normalized()),
          run_time_(std::max(run_time, 1e-3)),
          q_max_speed_(q_max_speed) {}

    void start(const Eigen::Vector3d& pos0,
               const Eigen::Quaterniond& quat0) override {
        p0_ = pos0;
        q0_ = quat0.normalized();
        // Take shortest-path SLERP — flip the goal hemisphere if the dot
        // product is negative. Without this the arm might rotate the long
        // way around (∼2π instead of 0).
        if (q0_.coeffs().dot(goal_quat_.coeffs()) < 0.0)
            goal_quat_.coeffs() = -goal_quat_.coeffs();
        elapsed_ = 0.0;
    }

    CartesianTarget step(double dt) override {
        elapsed_ = std::min(elapsed_ + dt, run_time_);
        const double s  = elapsed_ / run_time_;
        const double s3 = s * s * s;
        const double s4 = s3 * s;
        const double s5 = s4 * s;
        const double a  = 10.0 * s3 - 15.0 * s4 + 6.0 * s5;

        CartesianTarget out;
        out.pos  = p0_ + a * (goal_pos_ - p0_);
        out.quat = q0_.slerp(a, goal_quat_);
        return out;
    }

    bool finished() const override { return elapsed_ >= run_time_; }

    void limit_joint_speed(const Eigen::Matrix<double, 6, 7>& J) override {
        if (!(q_max_speed_ > 0.0)) return;
        // Twist that moves the start pose to the goal: translation, then the
        // base-frame rotation vector of goal · start⁻¹ (start() already put
        // the goal in the start's hemisphere).
        Eigen::Matrix<double, 6, 1> dx;
        dx.head<3>() = goal_pos_ - p0_;
        const Eigen::AngleAxisd aa(goal_quat_ * q0_.conjugate());
        dx.tail<3>() = aa.angle() * aa.axis();
        // Damped least-squares pseudo-inverse: finite near singularities.
        constexpr double kLambda2 = 1e-4;
        const Eigen::Matrix<double, 6, 6> JJt =
            J * J.transpose() + kLambda2 * Eigen::Matrix<double, 6, 6>::Identity();
        const Eigen::Matrix<double, 7, 1> dq =
            J.transpose() * JJt.ldlt().solve(dx);
        run_time_ = std::max(run_time_,
                             1.875 * dq.cwiseAbs().maxCoeff() / q_max_speed_);
    }

    double run_time() const { return run_time_; }

 private:
    Eigen::Vector3d    p0_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d    goal_pos_;
    Eigen::Quaterniond q0_{Eigen::Quaterniond::Identity()};
    Eigen::Quaterniond goal_quat_;
    double             run_time_;
    double             q_max_speed_{0.0};   // rad/s; 0 ⇒ no joint-speed bound
    double             elapsed_{0.0};
};
