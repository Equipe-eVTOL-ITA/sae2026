#pragma once

#include <memory>
#include <string>
#include <cmath>
#include <Eigen/Eigen>

#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"

/**
 * Centers the drone over the manometer using normalized pixel error from
 * /manometer_error.  Robust to intermittent detection: holds position on
 * missed frames and only counts alignment when the detector is active.
 *
 * Blackboard reads:
 *   "error_x"                       (float) — normalized [-1,1], NaN = not detected
 *   "error_y"                       (float) — normalized [-1,1], NaN = not detected
 *   "position_tolerance_align"      (float) — error magnitude threshold (default 0.10)
 *   "max_horizontal_velocity_align" (float) — max correction speed m/s (default 0.3)
 *   "align_kp"                      (float) — proportional gain (default 0.5)
 *   "align_min_detections"          (float) — consecutive aligned frames needed (default 10)
 *
 * Returns: "ALIGNED"
 */
class AlignState : public fsm::State {
public:
    AlignState() : fsm::State(), aligned_counter_(0), miss_counter_(0), tick_(0) {}

    void on_enter(fsm::Blackboard &blackboard) override {
        drone_ = *blackboard.get<std::shared_ptr<Drone>>("drone");
        if (!drone_) return;

        drone_->log("");
        drone_->log("STATE: ALIGN");

        tolerance_ = blackboard.contains("position_tolerance_align")
            ? *blackboard.get<float>("position_tolerance_align") : 0.10f;
        kp_ = blackboard.contains("align_kp")
            ? *blackboard.get<float>("align_kp") : 0.5f;
        max_vel_ = blackboard.contains("max_horizontal_velocity_align")
            ? *blackboard.get<float>("max_horizontal_velocity_align") : 0.3f;
        min_detections_ = blackboard.contains("align_min_detections")
            ? static_cast<int>(*blackboard.get<float>("align_min_detections")) : 10;

        aligned_counter_ = 0;
        miss_counter_    = 0;
        tick_            = 0;
        initial_yaw_     = drone_->getOrientation()[2];

        drone_->log("kp=" + std::to_string(kp_)
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
            // Hold current position — no valid detection this tick
            drone_->setLocalPosition(pos.x(), pos.y(), pos.z(), initial_yaw_);
            // Reset alignment counter only after 3 consecutive missed frames
            if (++miss_counter_ >= 3)
                aligned_counter_ = 0;
            if (tick_++ % 20 == 0)
                drone_->log("ALIGN: no detection — holding (aligned="
                            + std::to_string(aligned_counter_) + ")");
            return "";
        }

        miss_counter_ = 0;

        // P-controller: normalized camera error → NED velocity
        //   camera +x (right)  → drone +Y (East)
        //   camera +y (down)   → drone -X (South); top-of-image = forward → -err_y
        float vx = -err_y * kp_;
        float vy =  err_x * kp_;
        vx = std::clamp(vx, -max_vel_, max_vel_);
        vy = std::clamp(vy, -max_vel_, max_vel_);

        // Position setpoint one FSM tick ahead
        constexpr float dt = 0.05f;
        drone_->setLocalPosition(
            pos.x() + vx * dt,
            pos.y() + vy * dt,
            pos.z(),
            initial_yaw_);

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
    float tolerance_, kp_, max_vel_;
    float initial_yaw_;
    int   min_detections_;
    int   aligned_counter_, miss_counter_, tick_;
};
