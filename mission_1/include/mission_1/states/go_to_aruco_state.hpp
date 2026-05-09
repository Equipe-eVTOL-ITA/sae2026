#pragma once

#include <memory>
#include <string>
#include <cmath>

#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "drone/movement.hpp"

class GoToArucoState : public fsm::State {
public:
    GoToArucoState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {
        drone_ = *blackboard.get<std::shared_ptr<Drone>>("drone");
        if (drone_ == nullptr) return;

        drone_->log("");
        drone_->log("STATE: GO_TO_ARUCO");

        if (blackboard.contains("aruco_tolerance")) {
            tolerance_ = *blackboard.get<float>("aruco_tolerance");
        } else {
            tolerance_ = 0.1f;
        }

        kp_x_ = blackboard.get<float>("aruco_kp_x") ? *blackboard.get<float>("aruco_kp_x") : 0.5f;
        kp_y_ = blackboard.get<float>("aruco_kp_y") ? *blackboard.get<float>("aruco_kp_y") : 0.5f;

        aligned_counter_ = 0;
        miss_counter_ = 0;
        tick_         = 0;
    }

    std::string act(fsm::Blackboard &blackboard) override {
        if (drone_ == nullptr) return "ERROR";

        bool is_detected = false;
        if (blackboard.contains("aruco_detected")) {
            is_detected = *blackboard.get<bool>("aruco_detected");
        }

        if (!is_detected) {
            miss_counter_++;
            if (miss_counter_ > max_misses_) {
                drone_->log("ArUco lost! Returning to search.");
                return "ARUCO_LOST";
            }
            // Hover and wait — the spiral left residual velocity that takes
            // a few ticks to arrest; the ArUco typically reappears within 200ms.
            move_local_by_waypoint(drone_, drone_->getLocalPosition(), 0.0f);
            return "";
        }

        miss_counter_ = 0;

        float err_x = *blackboard.get<float>("aruco_x_error");
        float err_y = *blackboard.get<float>("aruco_y_error");

        // P controller: camera frame → drone FRD frame
        // top-of-image = drone forward (+X); err_y<0 means ArUco above center = forward
        float vx = -err_y * kp_x_;
        float vy =  err_x * kp_y_;

        float max_v = 1.0f;
        vx = std::clamp(vx, -max_v, max_v);
        vy = std::clamp(vy, -max_v, max_v);

        move_local_by_speed(drone_, vx, vy, 0.0f);

        bool target_calculated = blackboard.contains("target_calculated") &&
                                 *blackboard.get<bool>("target_calculated");

        if (std::abs(err_x) < tolerance_ && std::abs(err_y) < tolerance_) {
            aligned_counter_++;
        } else {
            aligned_counter_ = 0;
        }

        // Fewer frames required once the target is already known
        int required_aligned = target_calculated ? 10 : 20;

        if (tick_++ % 10 == 0)
            drone_->log("ArUco err=(" + std::to_string(err_x) + "," + std::to_string(err_y)
                        + ") aligned=" + std::to_string(aligned_counter_)
                        + "/" + std::to_string(required_aligned));

        if (aligned_counter_ > required_aligned) {
            if (target_calculated) {
                drone_->log("ArUco Aligned and Target Calculated!");
                std::string target = *blackboard.get<std::string>("target_base");
                drone_->log("Target Base is: " + target);

                bool target_base_in_sight = false;
                if (blackboard.contains("target_base_in_sight")) {
                    target_base_in_sight = *blackboard.get<bool>("target_base_in_sight");
                }

                if (target_base_in_sight) {
                    return "KNOWN_BASE";
                } else {
                    return "UNKNOWN_BASE";
                }
            } else {
                drone_->log("Aligned but shape unknown. Descending for closer look.");
                return "SHAPE_UNKNOWN";
            }
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
    static constexpr int max_misses_ = 30;  // 1.5s at 20Hz — allows for inertia settling after spiral
};
