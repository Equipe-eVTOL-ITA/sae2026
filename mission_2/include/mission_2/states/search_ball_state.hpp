#pragma once

#include <memory>
#include <string>

#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "drone/movement.hpp"
#include <cmath>

class SearchBallState : public fsm::State {
public:
    SearchBallState() : fsm::State(), target_yaw_(0.0f) {}

    void on_enter(fsm::Blackboard &blackboard) override {
        drone_ = *blackboard.get<std::shared_ptr<Drone>>("drone");
        if (drone_ == nullptr) return;

        drone_->log("");
        drone_->log("STATE: SEARCH_BALL");

        yaw_rate_ = *blackboard.get<float>("search_yaw_rate");
        target_yaw_ = drone_->getOrientation()[2];
    }

    std::string act(fsm::Blackboard &blackboard) override {
        if (drone_ == nullptr) return "ERROR";

        bool is_detected = false;
        if (blackboard.contains("ball_is_detected")) {
            is_detected = *blackboard.get<bool>("ball_is_detected");
        }

        if (is_detected) {
            // Stop drone
            move_local_by_waypoint(drone_, drone_->getLocalPosition(), 0.0f);
            drone_->log("Ball detected!");
            return "BALL_FOUND";
        }

        // Spin in yaw using position-based rotation
        target_yaw_ += yaw_rate_ * 0.1f; // assuming approx 10Hz tick rate
        if (target_yaw_ > M_PI) target_yaw_ -= 2.0 * M_PI;
        if (target_yaw_ < -M_PI) target_yaw_ += 2.0 * M_PI;
        
        rotateYaw(drone_, target_yaw_, yaw_rate_);

        return "";
    }

private:
    std::shared_ptr<Drone> drone_;
    float yaw_rate_;
    float target_yaw_;
};
