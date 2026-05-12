#pragma once

#include <memory>
#include <string>
#include <cmath>
#include <Eigen/Eigen>

#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"

/**
 * Centers the drone over the manometer using normalized pixel error from
 * /manometer_error.  Uses a PD controller with direct velocity setpoints
 * (same approach as GoToBaseState in mission_1) — position-step setpoints
 * (pos + v*dt) generate ~1.5cm steps that PX4's outer loop ignores.
 *
 * Blackboard reads:
 *   "error_x"                       (float) — normalized [-1,1], NaN = not detected
 *   "error_y"                       (float) — normalized [-1,1], NaN = not detected
 *   "position_tolerance_align"      (float) — error magnitude threshold (default 0.10)
 *   "max_horizontal_velocity_align" (float) — max correction speed m/s (default 0.3)
 *   "align_kp"                      (float) — proportional gain (default 0.7)
 *   "align_kd"                      (float) — derivative gain (default 0.05)
 *   "align_min_detections"          (float) — consecutive aligned frames needed (default 10)
 *
 * Returns: "ALIGNED"
 */
class AlignState : public fsm::State {
public:
    AlignState() : fsm::State(),
        kp_(0.7f), kd_(0.05f), max_vel_(0.3f), tolerance_(0.10f),
        entry_z_(0.0f), initial_yaw_(0.0f),
        err_x_prev_(0.0f), err_y_prev_(0.0f),
        aligned_counter_(0), miss_counter_(0), tick_(0) {}

    void on_enter(fsm::Blackboard &blackboard) override {
        drone_ = *blackboard.get<std::shared_ptr<Drone>>("drone");
        if (!drone_) return;

        drone_->log("");
        drone_->log("STATE: ALIGN");

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

        entry_z_     = static_cast<float>(drone_->getLocalPosition().z());
        initial_yaw_ = drone_->getOrientation()[2];

        err_x_prev_      = 0.0f;
        err_y_prev_      = 0.0f;
        aligned_counter_ = 0;
        miss_counter_    = 0;
        tick_            = 0;

        drone_->log("kp=" + std::to_string(kp_)
                    + " kd=" + std::to_string(kd_)
                    + " tol=" + std::to_string(tolerance_)
                    + " need=" + std::to_string(min_detections_) + " frames");
    }

    std::string act(fsm::Blackboard &blackboard) override {
        if (!drone_) return "ERROR";

        auto pos = drone_->getLocalPosition();

        float err_x = *blackboard.get<float>("error_x");
        float err_y = *blackboard.get<float>("error_y");
        bool detected = !std::isnan(err_x) && !std::isnan(err_y);

        if (!detected) {
            // Hold position — reset derivative to avoid spike on re-detection
            err_x_prev_ = 0.0f;
            err_y_prev_ = 0.0f;
            drone_->setLocalPosition(pos.x(), pos.y(), entry_z_, initial_yaw_);
            if (++miss_counter_ >= 3)
                aligned_counter_ = 0;
            if (tick_++ % 20 == 0)
                drone_->log("ALIGN: no detection — holding (aligned="
                            + std::to_string(aligned_counter_) + ")");
            return "";
        }

        // First tick after a miss: seed prev to current to suppress derivative spike
        if (miss_counter_ > 0) {
            err_x_prev_ = err_x;
            err_y_prev_ = err_y;
        }
        miss_counter_ = 0;

        // PD controller — normalized camera error → NED velocity setpoint
        //   camera +x (right)  → drone +Y (East)
        //   camera +y (down)   → drone -X (South); top-of-image = forward → -err_y
        float d_err_x = (err_x - err_x_prev_) / 0.05f;
        float d_err_y = (err_y - err_y_prev_) / 0.05f;
        err_x_prev_ = err_x;
        err_y_prev_ = err_y;

        float vx = -(err_y * kp_ + d_err_y * kd_);
        float vy =  (err_x * kp_ + d_err_x * kd_);
        vx = std::clamp(vx, -max_vel_, max_vel_);
        vy = std::clamp(vy, -max_vel_, max_vel_);

        // Direct velocity setpoint — PX4 holds altitude with vz=0
        drone_->setLocalVelocity(vx, vy, 0.0, 0.0);

        float err_mag = std::sqrt(err_x * err_x + err_y * err_y);
        if (err_mag < tolerance_) {
            ++aligned_counter_;
        } else {
            aligned_counter_ = 0;
        }

        if (tick_++ % 10 == 0)
            drone_->log("ALIGN err=(" + std::to_string(err_x) + ","
                        + std::to_string(err_y) + ") mag=" + std::to_string(err_mag)
                        + " aligned=" + std::to_string(aligned_counter_)
                        + "/" + std::to_string(min_detections_));

        if (aligned_counter_ >= min_detections_) {
            drone_->log("ALIGN: manometer centered!");
            return "ALIGNED";
        }

        return "";
    }

private:
    std::shared_ptr<Drone> drone_;
    float tolerance_, kp_, kd_, max_vel_;
    float entry_z_, initial_yaw_;
    float err_x_prev_, err_y_prev_;
    int   min_detections_;
    int   aligned_counter_, miss_counter_, tick_;
};
