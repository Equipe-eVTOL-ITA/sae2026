#pragma once

#include <cmath>
#include <memory>
#include <string>

#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "drone/movement.hpp"

class RiseState : public fsm::State {
public:
    RiseState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {
        drone_ = *blackboard.get<std::shared_ptr<Drone>>("drone");
        if (drone_ == nullptr) return;

        drone_->log("");
        drone_->log("STATE: RISE");

        float delta_z = *blackboard.get<float>("rise_delta_z");
        
        // drone_ z is negative in FRD (up is negative). So we subtract to rise.
        target_z_ = drone_->getLocalPosition().z() - delta_z; 
        tolerance_ = *blackboard.get<float>("position_tolerance");
    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void)blackboard;
        if (drone_ == nullptr) return "ERROR";

        float current_z = drone_->getLocalPosition().z();
        float err = std::abs(target_z_ - current_z);

        if (err < tolerance_) {
            move_local_by_waypoint(drone_, drone_->getLocalPosition(), 0.0f);
            drone_->log("Rise completed");
            return "RISE_COMPLETED";
        }

        // Move to target_z (maintaining current X, Y)
        Eigen::Vector3d pos = drone_->getLocalPosition();
        drone_->setLocalPosition(pos.x(), pos.y(), target_z_, drone_->getOrientation().z());

        return "";
    }

private:
    std::shared_ptr<Drone> drone_;
    float target_z_;
    float tolerance_;
};
