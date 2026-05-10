#pragma once

#include <memory>
#include <string>
#include <cmath>

#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "drone/movement.hpp"

class GoToBaseState : public fsm::State {
public:
    GoToBaseState() : fsm::State(), entry_z_(0.0f) {}

    void on_enter(fsm::Blackboard &blackboard) override {
        drone_ = *blackboard.get<std::shared_ptr<Drone>>("drone");
        if (drone_ == nullptr) return;

        drone_->log("");
        drone_->log("STATE: GO_TO_BASE");

        if (blackboard.contains("base_tolerance")) {
            tolerance_ = *blackboard.get<float>("base_tolerance");
        } else {
            tolerance_ = 0.05f;
        }

        kp_x_ = blackboard.get<float>("base_kp_x") ? *blackboard.get<float>("base_kp_x") : 0.5f;
        kp_y_ = blackboard.get<float>("base_kp_y") ? *blackboard.get<float>("base_kp_y") : 0.5f;

        entry_z_ = static_cast<float>(drone_->getLocalPosition().z());

        aligned_counter_ = 0;
        miss_counter_ = 0;
        tick_         = 0;
    }

    std::string act(fsm::Blackboard &blackboard) override {
        if (drone_ == nullptr) return "ERROR";

        bool is_detected = false;
        if (blackboard.contains("target_base_in_sight")) {
            is_detected = *blackboard.get<bool>("target_base_in_sight");
        }

        if (!is_detected) {
            miss_counter_++;
            if (miss_counter_ > max_misses_) {
                drone_->log("Base lost! Returning to search.");
                return "BASE_LOST";
            }
            // Directly set PX4 setpoint — move_local_by_waypoint with speed=0 is a
            // no-op (little_goal = current_pos regardless of target z), so we bypass it.
            auto cur = drone_->getLocalPosition();
            drone_->setLocalPosition(
                static_cast<float>(cur.x()),
                static_cast<float>(cur.y()),
                entry_z_,
                static_cast<float>(drone_->getOrientation()[2])
            );
            return "";
        }

        miss_counter_ = 0;

        float err_x = *blackboard.get<float>("target_base_x_error");
        float err_y = *blackboard.get<float>("target_base_y_error");

        float vx = -err_y * kp_x_;
        float vy =  err_x * kp_y_;

        float max_v = 1.0f;
        vx = std::clamp(vx, -max_v, max_v);
        vy = std::clamp(vy, -max_v, max_v);

        // Gently correct altitude back to entry z if PX4 drifts (max 0.5 m/s)
        float cur_z = static_cast<float>(drone_->getLocalPosition().z());
        float vz = std::clamp((entry_z_ - cur_z) / 0.05f, -0.5f, 0.5f);
        move_local_by_vel_as_position(drone_, vx, vy, vz);

        if (std::abs(err_x) < tolerance_ && std::abs(err_y) < tolerance_) {
            aligned_counter_++;
        } else {
            aligned_counter_ = 0;
        }

        if (tick_++ % 10 == 0)
            drone_->log("Base err=(" + std::to_string(err_x) + "," + std::to_string(err_y)
                        + ") aligned=" + std::to_string(aligned_counter_) + "/20");

        if (aligned_counter_ > 10) {
            drone_->log("Aligned with Base! Ready to land.");
            return "ALIGNED";
        }

        return "";
    }

private:
    std::shared_ptr<Drone> drone_;
    float tolerance_;
    float kp_x_, kp_y_;
    float entry_z_;
    int aligned_counter_;
    int miss_counter_;
    int tick_;
    static constexpr int max_misses_ = 30;  // 1.5s at 20Hz before declaring lost
};
