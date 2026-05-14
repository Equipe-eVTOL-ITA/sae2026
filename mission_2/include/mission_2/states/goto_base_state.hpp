#pragma once

#include <Eigen/Eigen>
#include <cmath>
#include <memory>
#include <string>

#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "drone/movement.hpp"

/**
 * Navigate back to the home base at a limited velocity, then hover
 * for `hover_before_landing_ticks` ticks before transitioning to ARRIVED.
 *
 * Improvements over the generic GoToState:
 *  - Velocity-limited via move_local_by_waypoint (smoother return)
 *  - Configurable hover before landing so the drone is stable before descent
 */
class GoToBaseState : public fsm::State {
public:
    GoToBaseState() : fsm::State(),
        target_x_(0.0f), target_y_(0.0f), target_z_(-1.5f), target_yaw_(0.0f),
        tolerance_(0.15f), max_vel_(0.5f), hover_ticks_(40), hover_counter_(0) {}

    void on_enter(fsm::Blackboard &blackboard) override {
        drone_ = *blackboard.get<std::shared_ptr<Drone>>("drone");
        if (!drone_) return;

        drone_->log("");
        drone_->log("STATE: GOTO_BASE");

        target_x_  = blackboard.contains("target_x")   ? *blackboard.get<float>("target_x")  : 0.0f;
        target_y_  = blackboard.contains("target_y")   ? *blackboard.get<float>("target_y")  : 0.0f;
        target_z_  = blackboard.contains("target_z")   ? *blackboard.get<float>("target_z")  : -1.5f;
        target_yaw_= blackboard.contains("target_yaw") ? *blackboard.get<float>("target_yaw"): 0.0f;
        tolerance_ = blackboard.contains("position_tolerance")
            ? *blackboard.get<float>("position_tolerance") : 0.15f;
        max_vel_   = blackboard.contains("max_vel_goto")
            ? *blackboard.get<float>("max_vel_goto") : 0.5f;
        hover_ticks_ = blackboard.contains("hover_before_landing_ticks")
            ? static_cast<int>(*blackboard.get<float>("hover_before_landing_ticks")) : 40;

        hover_counter_ = 0;

        drone_->log("Returning to (" + std::to_string(target_x_) + "," +
                    std::to_string(target_y_) + "," + std::to_string(target_z_) + ")" +
                    " max_vel=" + std::to_string(max_vel_) +
                    " hover=" + std::to_string(hover_ticks_) + " ticks");
    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void)blackboard;
        if (!drone_) return "ERROR";

        auto pos = drone_->getLocalPosition();
        float dx = target_x_ - static_cast<float>(pos.x());
        float dy = target_y_ - static_cast<float>(pos.y());
        float dz = target_z_ - static_cast<float>(pos.z());
        float dist = std::sqrt(dx*dx + dy*dy + dz*dz);

        if (dist < tolerance_) {
            // Hold precisely at target during hover phase
            drone_->setLocalPosition(target_x_, target_y_, target_z_, target_yaw_);
            if (++hover_counter_ >= hover_ticks_) {
                drone_->log("Hover complete — transitioning to LANDING");
                return "ARRIVED";
            }
            if (hover_counter_ % 20 == 0) {
                drone_->log("Hovering before landing (" +
                            std::to_string(hover_counter_) + "/" +
                            std::to_string(hover_ticks_) + " ticks)");
            }
            return "";
        }

        // Outside tolerance: approach at limited velocity
        hover_counter_ = 0;
        Eigen::Vector3d tgt(target_x_, target_y_, target_z_);
        move_local_by_waypoint(drone_, tgt, max_vel_, tolerance_, target_yaw_);
        return "";
    }

private:
    std::shared_ptr<Drone> drone_;
    float target_x_, target_y_, target_z_, target_yaw_;
    float tolerance_, max_vel_;
    int   hover_ticks_, hover_counter_;
};
