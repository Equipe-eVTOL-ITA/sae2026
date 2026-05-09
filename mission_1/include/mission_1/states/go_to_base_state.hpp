#pragma once

#include <memory>
#include <string>
#include <cmath>

#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "drone/movement.hpp"

class GoToBaseState : public fsm::State {
public:
    GoToBaseState() : fsm::State() {}

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
            move_local_by_waypoint(drone_, drone_->getLocalPosition(), 0.0f);
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

        move_local_by_speed(drone_, vx, vy, 0.0f);

        if (std::abs(err_x) < tolerance_ && std::abs(err_y) < tolerance_) {
            aligned_counter_++;
        } else {
            aligned_counter_ = 0;
        }

        if (tick_++ % 10 == 0)
            drone_->log("Base err=(" + std::to_string(err_x) + "," + std::to_string(err_y)
                        + ") aligned=" + std::to_string(aligned_counter_) + "/20");

        if (aligned_counter_ > 20) {
            drone_->log("Aligned with Base! Ready to land.");
            return "ALIGNED";
        }

        return "";
    }

private:
    std::shared_ptr<Drone> drone_;
    float tolerance_;
    float kp_x_, kp_y_;
    int aligned_counter_;
    int miss_counter_;
    int tick_;
    static constexpr int max_misses_ = 30;  // 1.5s at 20Hz before declaring lost
};
