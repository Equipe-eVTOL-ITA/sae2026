#pragma once

#include <Eigen/Eigen>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "drone/PidController.hpp"

/**
 * @brief Aligns drone over hose center (XY plane only).
 * Uses PD control on normalized camera error to center the hose image.
 * Returns "XY_ALIGNED" when centered, "HOSE_LOST" on timeout/failure, "" while aligning.
 */
class MangueiraAlignXYState : public fsm::State {
public:
    MangueiraAlignXYState()
        : fsm::State(),
          alignment_timeout_(30.0f),
          lost_timeout_(6),
          min_detections_(10),
          tolerance_(0.10f),
          kp_(0.7f), kd_(0.05f), max_vel_(0.3f),
          entry_z_(0.0f), initial_yaw_(0.0f),
          err_x_prev_(0.0f), err_y_prev_(0.0f),
          aligned_counter_(0), miss_counter_(0), tick_(0) {}

    void on_enter(fsm::Blackboard &blackboard) override {
        drone_ = *blackboard.get<std::shared_ptr<Drone>>("drone");
        if (!drone_) return;

        drone_->log("");
        drone_->log("STATE: MANGUEIRA_ALIGN_XY");

        alignment_timeout_ = blackboard.contains("align_timeout")
            ? *blackboard.get<float>("align_timeout") : 30.0f;
        lost_timeout_ = blackboard.contains("lost_timeout")
            ? static_cast<int>(*blackboard.get<float>("lost_timeout")) : 6;
        tolerance_ = blackboard.contains("position_tolerance_align")
            ? *blackboard.get<float>("position_tolerance_align") : 0.10f;
        kp_ = blackboard.contains("align_kp")
            ? *blackboard.get<float>("align_kp") : 0.7f;
        kd_ = blackboard.contains("align_kd")
            ? *blackboard.get<float>("align_kd") : 0.05f;
        max_vel_ = blackboard.contains("max_horizontal_velocity_align")
            ? *blackboard.get<float>("max_horizontal_velocity_align") : 0.3f;
        min_detections_ = blackboard.contains("align_min_detections")
            ? static_cast<int>(*blackboard.get<float>("align_min_detections")) : 10;

        entry_z_ = static_cast<float>(drone_->getLocalPosition().z());
        initial_yaw_ = drone_->getOrientation()[2];

        err_x_prev_ = 0.0f;
        err_y_prev_ = 0.0f;
        aligned_counter_ = 0;
        miss_counter_ = 0;
        tick_ = 0;

        start_time_ = std::chrono::steady_clock::now();

        drone_->log("XY align: kp=" + std::to_string(kp_) + " kd=" + std::to_string(kd_) +
                    " tol=" + std::to_string(tolerance_) +
                    " need=" + std::to_string(min_detections_) + " frames");
    }

    std::string act(fsm::Blackboard &blackboard) override {
        if (!drone_) return "ERROR";

        auto current_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time_);
        if (elapsed.count() > alignment_timeout_) {
            drone_->log("Mangueira XY alignment timeout");
            return "HOSE_LOST";
        }

        auto pos = drone_->getLocalPosition();

        // Check blackboard for detection
        const bool have_centers = blackboard.contains("hose_center_x") && blackboard.contains("hose_center_y");
        bool hose_in_sight = true;
        if (blackboard.contains("hose_in_sight"))
            hose_in_sight = *blackboard.get<bool>("hose_in_sight");

        if (!have_centers || !hose_in_sight) {
            err_x_prev_ = 0.0f;
            err_y_prev_ = 0.0f;
            drone_->setLocalPosition(pos.x(), pos.y(), entry_z_, initial_yaw_);
            if (++miss_counter_ >= 3) aligned_counter_ = 0;
            if (tick_++ % 20 == 0)
                drone_->log("MANGUEIRA_ALIGN_XY: no detection — holding (aligned=" + std::to_string(aligned_counter_) + ")");
            return "";
        }

        // Read center values
        float center_x = *blackboard.get<float>("hose_center_x");
        float center_y = *blackboard.get<float>("hose_center_y");

        // Normalize to [-1, 1]
        float err_x = (center_x - 0.5f) * 2.0f;
        float err_y = (center_y - 0.5f) * 2.0f;

        if (miss_counter_ > 0) {
            err_x_prev_ = err_x;
            err_y_prev_ = err_y;
        }
        miss_counter_ = 0;

        // PD controller
        float d_err_x = (err_x - err_x_prev_) / 0.05f;
        float d_err_y = (err_y - err_y_prev_) / 0.05f;
        err_x_prev_ = err_x;
        err_y_prev_ = err_y;

        float vx = -(err_y * kp_ + d_err_y * kd_);
        float vy =  (err_x * kp_ + d_err_x * kd_);
        if (vx > max_vel_) vx = max_vel_;
        if (vx < -max_vel_) vx = -max_vel_;
        if (vy > max_vel_) vy = max_vel_;
        if (vy < -max_vel_) vy = -max_vel_;

        // Rotate to initial-yaw FRD frame (no yaw control in XY state)
        const float yaw_now = drone_->getOrientation()[2];
        float dq = yaw_now - initial_yaw_;
        while (dq > static_cast<float>(M_PI)) dq -= 2.0f * static_cast<float>(M_PI);
        while (dq < -static_cast<float>(M_PI)) dq += 2.0f * static_cast<float>(M_PI);
        const float c = std::cos(dq);
        const float s = std::sin(dq);
        const float vx_frd = c * vx + s * vy;
        const float vy_frd = -s * vx + c * vy;

        // Setpoint with zero yaw rate (keep initial yaw)
        drone_->setLocalVelocity(vx_frd, vy_frd, 0.0f, 0.0f);

        float err_mag = std::sqrt(err_x * err_x + err_y * err_y);
        if (err_mag < tolerance_) {
            ++aligned_counter_;
        } else {
            aligned_counter_ = 0;
        }

        if (tick_++ % 10 == 0)
            drone_->log("MANGUEIRA_ALIGN_XY err=(" + std::to_string(err_x) + "," + std::to_string(err_y) + ") mag=" + std::to_string(err_mag)
                        + " aligned=" + std::to_string(aligned_counter_) + "/" + std::to_string(min_detections_));

        if (aligned_counter_ >= min_detections_) {
            drone_->log("MANGUEIRA_ALIGN_XY: centered!");
            return "XY_ALIGNED";
        }

        return "";
    }

    void on_exit(fsm::Blackboard &blackboard) override {
        (void)blackboard;
        if (drone_) drone_->log("Exiting MANGUEIRA_ALIGN_XY state");
    }

private:
    std::shared_ptr<Drone> drone_;

    float alignment_timeout_;
    int lost_timeout_;
    int min_detections_;
    float tolerance_;
    float kp_, kd_, max_vel_;
    float entry_z_, initial_yaw_;
    float err_x_prev_, err_y_prev_;
    int   aligned_counter_, miss_counter_, tick_;

    std::chrono::steady_clock::time_point start_time_;
};

/**
 * @brief Aligns drone yaw to hose orientation.
 * Uses PD control on hose angle error to rotate drone until seta is vertical.
 * Returns "ALIGNED" when rotated, "HOSE_LOST" on timeout/failure, "" while aligning.
 * Assumes XY is already centered (called after MANGUEIRA_ALIGN_XY).
 */
class MangueiraAlignYawState : public fsm::State {
public:
    MangueiraAlignYawState()
        : fsm::State(),
          alignment_timeout_(30.0f),
          lost_timeout_(6),
          min_detections_(10),
          yaw_tolerance_(0.03f),
          kp_yaw_(0.5f), kd_yaw_(0.1f),
          max_yaw_rate_(0.5f),
          entry_z_(0.0f), initial_yaw_(0.0f),
          pid_yaw_(0.5f, 0.0f, 0.1f, 0.0f, 0.05f),
          aligned_counter_(0), miss_counter_(0), tick_(0) {}

    void on_enter(fsm::Blackboard &blackboard) override {
        drone_ = *blackboard.get<std::shared_ptr<Drone>>("drone");
        if (!drone_) return;

        drone_->log("");
        drone_->log("STATE: MANGUEIRA_ALIGN_YAW");

        alignment_timeout_ = blackboard.contains("align_timeout")
            ? *blackboard.get<float>("align_timeout") : 30.0f;
        lost_timeout_ = blackboard.contains("lost_timeout")
            ? static_cast<int>(*blackboard.get<float>("lost_timeout")) : 6;
        yaw_tolerance_ = blackboard.contains("align_tolerance_yaw")
            ? *blackboard.get<float>("align_tolerance_yaw") : 0.03f;
        min_detections_ = blackboard.contains("align_min_detections")
            ? static_cast<int>(*blackboard.get<float>("align_min_detections")) : 10;

        kp_yaw_ = blackboard.contains("align_kp_yaw")
            ? *blackboard.get<float>("align_kp_yaw") : 0.5f;
        const float ki_yaw = blackboard.contains("align_ki_yaw")
            ? *blackboard.get<float>("align_ki_yaw") : 0.0f;
        kd_yaw_ = blackboard.contains("align_kd_yaw")
            ? *blackboard.get<float>("align_kd_yaw") : 0.1f;
        max_yaw_rate_ = blackboard.contains("align_max_yaw_rate")
            ? *blackboard.get<float>("align_max_yaw_rate") : 0.5f;

        entry_z_ = static_cast<float>(drone_->getLocalPosition().z());
        initial_yaw_ = drone_->getOrientation()[2];

        aligned_counter_ = 0;
        miss_counter_ = 0;
        tick_ = 0;

        pid_yaw_ = PidController(kp_yaw_, ki_yaw, kd_yaw_, 0.0f, 0.05f);
        pid_yaw_.reset();

        start_time_ = std::chrono::steady_clock::now();

        drone_->log("YAW align: kp=" + std::to_string(kp_yaw_) + " kd=" + std::to_string(kd_yaw_) +
                    " tol=" + std::to_string(yaw_tolerance_) +
                    " need=" + std::to_string(min_detections_) + " frames");
    }

    std::string act(fsm::Blackboard &blackboard) override {
        if (!drone_) return "ERROR";

        auto current_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time_);
        if (elapsed.count() > alignment_timeout_) {
            drone_->log("Mangueira YAW alignment timeout");
            return "HOSE_LOST";
        }

        auto pos = drone_->getLocalPosition();

        // Check blackboard for detection
        bool hose_in_sight = true;
        if (blackboard.contains("hose_in_sight"))
            hose_in_sight = *blackboard.get<bool>("hose_in_sight");

        float hose_angle = blackboard.contains("hose_angle_error")
            ? *blackboard.get<float>("hose_angle_error") : 0.0f;

        if (!hose_in_sight) {
            pid_yaw_.reset();
            drone_->setLocalPosition(pos.x(), pos.y(), entry_z_, initial_yaw_);
            if (++miss_counter_ >= 3) aligned_counter_ = 0;
            if (tick_++ % 20 == 0)
                drone_->log("MANGUEIRA_ALIGN_YAW: no detection — holding (aligned=" + std::to_string(aligned_counter_) + ")");
            return "";
        }

        miss_counter_ = 0;

        // PID yaw control (outputs angular rate in rad/s)
        float yawspeed_cmd = pid_yaw_.compute(hose_angle);
        if (yawspeed_cmd > max_yaw_rate_) yawspeed_cmd = max_yaw_rate_;
        if (yawspeed_cmd < -max_yaw_rate_) yawspeed_cmd = -max_yaw_rate_;

        // Setpoint: hold XY, rotate with yaw rate
        drone_->setLocalVelocity(0.0f, 0.0f, 0.0f, yawspeed_cmd);

        float yaw_error = std::abs(hose_angle);
        if (yaw_error < yaw_tolerance_) {
            ++aligned_counter_;
        } else {
            aligned_counter_ = 0;
        }

        if (tick_++ % 10 == 0)
            drone_->log("MANGUEIRA_ALIGN_YAW yaw_err=" + std::to_string(hose_angle)
                        + " yawspeed=" + std::to_string(yawspeed_cmd)
                        + " aligned=" + std::to_string(aligned_counter_) + "/" + std::to_string(min_detections_));

        if (aligned_counter_ >= min_detections_) {
            drone_->log("MANGUEIRA_ALIGN_YAW: seta vertical!");
            return "ALIGNED";
        }

        return "";
    }

    void on_exit(fsm::Blackboard &blackboard) override {
        (void)blackboard;
        if (drone_) drone_->log("Exiting MANGUEIRA_ALIGN_YAW state");
    }

private:
    std::shared_ptr<Drone> drone_;

    float alignment_timeout_;
    int lost_timeout_;
    int min_detections_;
    float yaw_tolerance_;
    float kp_yaw_, kd_yaw_;
    float max_yaw_rate_;

    float entry_z_, initial_yaw_;
    PidController pid_yaw_;
    int   aligned_counter_, miss_counter_, tick_;

    std::chrono::steady_clock::time_point start_time_;
};

/**
 * @brief Final alignment: corrects Y position (along hose) and yaw.
 * Assumes XY is roughly centered and yaw is aligned.
 * Maintains Y centering + yaw alignment as final refinement.
 * Returns "FINAL_ALIGNED" when both converged, "HOSE_LOST" on failure, "" while refining.
 */
class MangueiraAlignFinalState : public fsm::State {
public:
    MangueiraAlignFinalState()
        : fsm::State(),
          alignment_timeout_(30.0f),
          lost_timeout_(6),
          min_detections_(10),
                    y_tolerance_(0.06f),
          yaw_tolerance_(0.03f),
          kp_y_(0.7f), kd_y_(0.05f), max_vel_y_(0.3f),
          kp_yaw_(0.5f), kd_yaw_(0.1f),
          max_yaw_rate_(0.5f),
          entry_z_(0.0f), initial_yaw_(0.0f),
          err_y_prev_(0.0f),
          pid_yaw_(0.5f, 0.0f, 0.1f, 0.0f, 0.05f),
          aligned_counter_(0), miss_counter_(0), tick_(0) {}

    void on_enter(fsm::Blackboard &blackboard) override {
        drone_ = *blackboard.get<std::shared_ptr<Drone>>("drone");
        if (!drone_) return;

        drone_->log("");
        drone_->log("STATE: MANGUEIRA_ALIGN_FINAL");

        alignment_timeout_ = blackboard.contains("align_timeout")
            ? *blackboard.get<float>("align_timeout") : 30.0f;
        lost_timeout_ = blackboard.contains("lost_timeout")
            ? static_cast<int>(*blackboard.get<float>("lost_timeout")) : 6;
        y_tolerance_ = blackboard.contains("position_tolerance_align")
            ? *blackboard.get<float>("position_tolerance_align") : 0.06f;
        yaw_tolerance_ = blackboard.contains("align_tolerance_yaw")
            ? *blackboard.get<float>("align_tolerance_yaw") : 0.03f;
        kp_y_ = blackboard.contains("align_kp")
            ? *blackboard.get<float>("align_kp") : 0.7f;
        kd_y_ = blackboard.contains("align_kd")
            ? *blackboard.get<float>("align_kd") : 0.05f;
        max_vel_y_ = blackboard.contains("max_horizontal_velocity_align")
            ? *blackboard.get<float>("max_horizontal_velocity_align") : 0.3f;
        min_detections_ = blackboard.contains("align_min_detections")
            ? static_cast<int>(*blackboard.get<float>("align_min_detections")) : 10;

        kp_yaw_ = blackboard.contains("align_kp_yaw")
            ? *blackboard.get<float>("align_kp_yaw") : 0.5f;
        const float ki_yaw = blackboard.contains("align_ki_yaw")
            ? *blackboard.get<float>("align_ki_yaw") : 0.0f;
        kd_yaw_ = blackboard.contains("align_kd_yaw")
            ? *blackboard.get<float>("align_kd_yaw") : 0.1f;
        max_yaw_rate_ = blackboard.contains("align_max_yaw_rate")
            ? *blackboard.get<float>("align_max_yaw_rate") : 0.5f;

        entry_z_ = static_cast<float>(drone_->getLocalPosition().z());
        initial_yaw_ = drone_->getOrientation()[2];

        err_y_prev_ = 0.0f;
        aligned_counter_ = 0;
        miss_counter_ = 0;
        tick_ = 0;

        pid_yaw_ = PidController(kp_yaw_, ki_yaw, kd_yaw_, 0.0f, 0.05f);
        pid_yaw_.reset();

        start_time_ = std::chrono::steady_clock::now();

        drone_->log("FINAL align: kp_y=" + std::to_string(kp_y_) + " kd_y=" + std::to_string(kd_y_) +
                    " y_tol=" + std::to_string(y_tolerance_) +
                    " yaw_tol=" + std::to_string(yaw_tolerance_) +
                    " need=" + std::to_string(min_detections_) + " frames");
    }

    std::string act(fsm::Blackboard &blackboard) override {
        if (!drone_) return "ERROR";

        auto current_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time_);
        if (elapsed.count() > alignment_timeout_) {
            drone_->log("Mangueira FINAL alignment timeout");
            return "HOSE_LOST";
        }

        auto pos = drone_->getLocalPosition();

        // Check blackboard for detection
        const bool have_center_y = blackboard.contains("hose_center_y");
        bool hose_in_sight = true;
        if (blackboard.contains("hose_in_sight"))
            hose_in_sight = *blackboard.get<bool>("hose_in_sight");

        float hose_angle = blackboard.contains("hose_angle_error")
            ? *blackboard.get<float>("hose_angle_error") : 0.0f;

        if (!have_center_y || !hose_in_sight) {
            err_y_prev_ = 0.0f;
            pid_yaw_.reset();
            drone_->setLocalPosition(pos.x(), pos.y(), entry_z_, initial_yaw_);
            if (++miss_counter_ >= 3) aligned_counter_ = 0;
            if (tick_++ % 20 == 0)
                drone_->log("MANGUEIRA_ALIGN_FINAL: no detection — holding (aligned=" + std::to_string(aligned_counter_) + ")");
            return "";
        }

        // Read only Y center
        float center_y = *blackboard.get<float>("hose_center_y");

        // Normalize to [-1, 1]
        float err_y = (center_y - 0.5f) * 2.0f;

        if (miss_counter_ > 0) {
            err_y_prev_ = err_y;
        }
        miss_counter_ = 0;

        // PD controller for Y only
        float d_err_y = (err_y - err_y_prev_) / 0.05f;
        err_y_prev_ = err_y;

        float vy = err_y * kp_y_ + d_err_y * kd_y_;
        if (vy > max_vel_y_) vy = max_vel_y_;
        if (vy < -max_vel_y_) vy = -max_vel_y_;

        // Rotate to initial-yaw FRD frame (only Y component)
        const float yaw_now = drone_->getOrientation()[2];
        float dq = yaw_now - initial_yaw_;
        while (dq > static_cast<float>(M_PI)) dq -= 2.0f * static_cast<float>(M_PI);
        while (dq < -static_cast<float>(M_PI)) dq += 2.0f * static_cast<float>(M_PI);
        const float c = std::cos(dq);
        const float s = std::sin(dq);
        const float vy_frd = -s * 0.0f + c * vy;  // No VX, only VY

        // PID yaw control
        float yawspeed_cmd = pid_yaw_.compute(hose_angle);
        if (yawspeed_cmd > max_yaw_rate_) yawspeed_cmd = max_yaw_rate_;
        if (yawspeed_cmd < -max_yaw_rate_) yawspeed_cmd = -max_yaw_rate_;

        // Setpoint: zero VX, maintain VY, control yaw
        drone_->setLocalVelocity(0.0f, vy_frd, 0.0f, yawspeed_cmd);

        float y_error = std::abs(err_y);
        float yaw_error = std::abs(hose_angle);
        if (y_error < y_tolerance_ && yaw_error < yaw_tolerance_) {
            ++aligned_counter_;
        } else {
            aligned_counter_ = 0;
        }

        if (tick_++ % 10 == 0)
            drone_->log("MANGUEIRA_ALIGN_FINAL err_y=" + std::to_string(err_y)
                        + " yaw_err=" + std::to_string(hose_angle)
                        + " yawspeed=" + std::to_string(yawspeed_cmd)
                        + " vy_frd=" + std::to_string(vy_frd)
                        + " aligned=" + std::to_string(aligned_counter_) + "/" + std::to_string(min_detections_));

        if (aligned_counter_ >= min_detections_) {
            drone_->log("MANGUEIRA_ALIGN_FINAL: fully aligned!");
            return "FINAL_ALIGNED";
        }

        return "";
    }

    void on_exit(fsm::Blackboard &blackboard) override {
        (void)blackboard;
        if (drone_) drone_->log("Exiting MANGUEIRA_ALIGN_FINAL state");
    }

private:
    std::shared_ptr<Drone> drone_;

    float alignment_timeout_;
    int lost_timeout_;
    int min_detections_;
    float y_tolerance_;
    float yaw_tolerance_;
    float kp_y_, kd_y_, max_vel_y_;
    float kp_yaw_, kd_yaw_;
    float max_yaw_rate_;

    float entry_z_, initial_yaw_;
    float err_y_prev_;
    PidController pid_yaw_;
    int   aligned_counter_, miss_counter_, tick_;

    std::chrono::steady_clock::time_point start_time_;
};

// Backward-compatibility alias (old code using MangueiraAlignState will get the XY state by default)
using MangueiraAlignState = MangueiraAlignXYState;
