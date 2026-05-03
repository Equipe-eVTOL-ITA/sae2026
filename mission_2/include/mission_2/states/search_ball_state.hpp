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
        start_position_ = drone_->getLocalPosition();
    }

    std::string act(fsm::Blackboard &blackboard) override {
        if (drone_ == nullptr) return "ERROR";

        bool is_detected = false;
        if (blackboard.contains("ball_is_detected")) {
            is_detected = *blackboard.get<bool>("ball_is_detected");
        }

        if (is_detected) {
            move_local_by_waypoint(drone_, drone_->getLocalPosition(), 0.0f); // parar o drone
            drone_->log("Ball detected!");
            return "BALL_FOUND";
        }

        // rodando o yaw do drone de forma suave usando controle de posição do PX4
        target_yaw_ += yaw_rate_ * 0.1f; // considerando um tick de 10 Hz
        if (target_yaw_ > M_PI) target_yaw_ -= 2.0 * M_PI;
        if (target_yaw_ < -M_PI) target_yaw_ += 2.0 * M_PI;
        
        // Mantém a posição inicial fixa e gira o yaw
        move_local_by_waypoint(drone_, start_position_, 0.5f, 0.1f, target_yaw_);

        return "";
    }

private:
    std::shared_ptr<Drone> drone_;
    float yaw_rate_;
    float target_yaw_;
    Eigen::Vector3d start_position_;
};
