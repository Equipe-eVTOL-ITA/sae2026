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

class MangueiraAlignState : public fsm::State {
public:
    // Backward-compatible constructor kept for mission_2.cpp call sites.
    MangueiraAlignState(bool align_center, bool align_yaw)
        : MangueiraAlignState() { align_center_ = align_center; align_yaw_ = align_yaw; }

    MangueiraAlignState()
        : fsm::State(),
          align_center_(true),
          align_yaw_(true),
          alignment_timeout_(30.0f),
          lost_timeout_(6),
          min_detections_(5),
          tolerance_(0.10f),
          yaw_tolerance_(0.05f),
          kp_(0.7f), kd_(0.05f), max_vel_(0.3f),
          kp_yaw_(0.5f), kd_yaw_(0.1f),
          max_yaw_rate_(0.5f),
          entry_z_(0.0f), initial_yaw_(0.0f),
          err_x_prev_(0.0f), err_y_prev_(0.0f),
          pid_yaw_(0.5f, 0.0f, 0.1f, 0.0f, 0.05f),
          aligned_counter_(0), miss_counter_(0), tick_(0) {}

    void on_enter(fsm::Blackboard &blackboard) override {
        drone_ = *blackboard.get<std::shared_ptr<Drone>>("drone");
        if (!drone_) return;

        drone_->log("");
        drone_->log("STATE: MANGUEIRA_ALIGN");

        alignment_timeout_ = blackboard.contains("align_timeout")
            ? *blackboard.get<float>("align_timeout") : 30.0f;
        lost_timeout_ = blackboard.contains("lost_timeout")
            ? static_cast<int>(*blackboard.get<float>("lost_timeout")) : 6;

        tolerance_ = blackboard.contains("position_tolerance_align")
            ? *blackboard.get<float>("position_tolerance_align") : 0.10f;
        yaw_tolerance_ = blackboard.contains("align_tolerance_yaw")
            ? *blackboard.get<float>("align_tolerance_yaw") : 0.05f;
        kp_ = blackboard.contains("align_kp")
            ? *blackboard.get<float>("align_kp") : 0.7f;
        kd_ = blackboard.contains("align_kd")
            ? *blackboard.get<float>("align_kd") : 0.05f;
        max_vel_ = blackboard.contains("max_horizontal_velocity_align")
            ? *blackboard.get<float>("max_horizontal_velocity_align") : 0.3f;
        min_detections_ = blackboard.contains("align_min_detections")
            ? static_cast<int>(*blackboard.get<float>("align_min_detections")) : 10;
        max_yaw_rate_ = blackboard.contains("align_max_yaw_rate")
            ? *blackboard.get<float>("align_max_yaw_rate") : 0.5f;

        kp_yaw_ = blackboard.contains("align_kp_yaw")
            ? *blackboard.get<float>("align_kp_yaw") : 0.5f;
        const float ki_yaw = blackboard.contains("align_ki_yaw")
            ? *blackboard.get<float>("align_ki_yaw") : 0.0f;
        kd_yaw_ = blackboard.contains("align_kd_yaw")
            ? *blackboard.get<float>("align_kd_yaw") : 0.1f;

        entry_z_ = static_cast<float>(drone_->getLocalPosition().z());
        initial_yaw_ = drone_->getOrientation()[2];

        err_x_prev_ = 0.0f;
        err_y_prev_ = 0.0f;
        aligned_counter_ = 0;
        miss_counter_ = 0;
        tick_ = 0;

        pid_yaw_ = PidController(kp_yaw_, ki_yaw, kd_yaw_, 0.0f, 0.05f);
        pid_yaw_.reset();

        start_time_ = std::chrono::steady_clock::now();

        drone_->log("kp=" + std::to_string(kp_) + " kd=" + std::to_string(kd_) +
                    " tol=" + std::to_string(tolerance_) +
                    " yaw_tol=" + std::to_string(yaw_tolerance_) +
                    " max_yawspeed=" + std::to_string(max_yaw_rate_) +
                    " need=" + std::to_string(min_detections_) + " frames");
    }

    std::string act(fsm::Blackboard &blackboard) override {
        if (!drone_) return "ERROR";

        auto current_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time_);
        if (elapsed.count() > alignment_timeout_) {
            drone_->log("Mangueira alignment timeout");
            return "HOSE_LOST";
        }

        auto pos = drone_->getLocalPosition();

        // Hold if centers missing or hose not in sight (timeout in mission_2 zeros centers but keeps keys).
        const bool have_centers = blackboard.contains("hose_center_x") && blackboard.contains("hose_center_y");
        bool hose_in_sight = true;
        if (blackboard.contains("hose_in_sight"))
            hose_in_sight = *blackboard.get<bool>("hose_in_sight");

        if (!have_centers || !hose_in_sight) {
            err_x_prev_ = 0.0f;
            err_y_prev_ = 0.0f;
            pid_yaw_.reset();
            drone_->setLocalPosition(pos.x(), pos.y(), entry_z_, initial_yaw_);
            if (++miss_counter_ >= 3) aligned_counter_ = 0;
            if (tick_++ % 20 == 0) {
                drone_->log("MANGUEIRA_ALIGN: no valid detection — holding (aligned=" +
                            std::to_string(aligned_counter_) + " in_sight=" +
                            std::string(hose_in_sight ? "1" : "0") + ")");
            }
            return "";
        }

        // Read center values
        float center_x = *blackboard.get<float>("hose_center_x");
        float center_y = *blackboard.get<float>("hose_center_y");

        float hose_angle = blackboard.contains("hose_angle_error")
            ? *blackboard.get<float>("hose_angle_error") : 0.0f;

        // Normalize to [-1, 1] as AlignState expects
        float err_x = (center_x - 0.5f) * 2.0f;
        float err_y = (center_y - 0.5f) * 2.0f;

        // First tick after a miss: seed prev to current to suppress derivative spike
        if (miss_counter_ > 0) {
            err_x_prev_ = err_x;
            err_y_prev_ = err_y;
        }
        miss_counter_ = 0;

        // PD controller — normalized camera error → NED velocity setpoint
        float d_err_x = (err_x - err_x_prev_) / 0.05f;
        float d_err_y = (err_y - err_y_prev_) / 0.05f;
        err_x_prev_ = err_x;
        err_y_prev_ = err_y;

        float vx = align_center_ ? -(err_y * kp_ + d_err_y * kd_) : 0.0f;
        float vy = align_center_ ?  (err_x * kp_ + d_err_x * kd_) : 0.0f;
        if (vx > max_vel_) vx = max_vel_;
        if (vx < -max_vel_) vx = -max_vel_;
        if (vy > max_vel_) vy = max_vel_;
        if (vy < -max_vel_) vy = -max_vel_;

        float yaw_error = 0.0f;
        // Fourth argument to setLocalVelocity is yawspeed (rad/s), not heading (see Drone::setLocalVelocity).
        float yawspeed_cmd = 0.0f;
        if (align_yaw_) {
            yawspeed_cmd = pid_yaw_.compute(hose_angle);
            yawspeed_cmd = std::clamp(yawspeed_cmd, -max_yaw_rate_, max_yaw_rate_);
            yaw_error = std::abs(hose_angle);
        }

        drone_->setLocalVelocity(vx, vy, 0.0f, yawspeed_cmd);

        float err_mag = std::sqrt(err_x * err_x + err_y * err_y);
        if (err_mag < tolerance_ && (!align_yaw_ || yaw_error < yaw_tolerance_)) {
            ++aligned_counter_;
        } else {
            aligned_counter_ = 0;
        }

        if (tick_++ % 10 == 0)
            drone_->log("MANGUEIRA_ALIGN err=(" + std::to_string(err_x) + "," + std::to_string(err_y) + ") mag=" + std::to_string(err_mag)
                        + " yaw_err=" + std::to_string(hose_angle)
                        + " yawspeed=" + std::to_string(yawspeed_cmd)
                        + " vx_vy=(" + std::to_string(vx) + "," + std::to_string(vy) + ")"
                        + " aligned=" + std::to_string(aligned_counter_) + "/" + std::to_string(min_detections_));

        if (aligned_counter_ >= min_detections_) {
            drone_->log("MANGUEIRA_ALIGN: hose centered!");
            return "ALIGNED";
        }

        return "";
    }

    void on_exit(fsm::Blackboard &blackboard) override {
        (void)blackboard;
        if (drone_) drone_->log("Exiting mangueira alignment state");
    }

private:
    std::shared_ptr<Drone> drone_;

    bool align_center_;
    bool align_yaw_;
    float alignment_timeout_;
    int lost_timeout_;
    int min_detections_;
    float tolerance_;
    float yaw_tolerance_;

    float kp_, kd_, max_vel_;
    float kp_yaw_, kd_yaw_;
    float max_yaw_rate_;

    float entry_z_, initial_yaw_;
    float err_x_prev_, err_y_prev_;
    PidController pid_yaw_;
    int   aligned_counter_, miss_counter_, tick_;

    std::chrono::steady_clock::time_point start_time_;
};
