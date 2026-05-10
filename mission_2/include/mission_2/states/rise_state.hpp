#pragma once

#include <cmath>
#include <memory>
#include <string>

#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "drone/movement.hpp"

class RiseState : public fsm::State {
public:
    RiseState() : fsm::State(), target_z_(0.0f), tolerance_(0.15f) {}

    void on_enter(fsm::Blackboard &blackboard) override {
        drone_ = *blackboard.get<std::shared_ptr<Drone>>("drone");
        if (drone_ == nullptr) return;

        drone_->log("");
        drone_->log("STATE: RISE");

        // Absolute target altitude (NED, negative = above ground).
        // Using a fixed altitude prevents cumulative drift when re-entering.
        target_z_ = blackboard.contains("rise_target_z")
            ? *blackboard.get<float>("rise_target_z")
            : static_cast<float>(drone_->getLocalPosition().z()) - 2.0f;

        tolerance_ = blackboard.contains("position_tolerance")
            ? *blackboard.get<float>("position_tolerance") : 0.15f;

        drone_->log("Rising to z=" + std::to_string(target_z_));
    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void)blackboard;
        if (drone_ == nullptr) return "ERROR";

        auto pos = drone_->getLocalPosition();
        float err = std::abs(target_z_ - static_cast<float>(pos.z()));

        if (err < tolerance_) {
            // Hold at target altitude (direct setLocalPosition — speed=0 is no-op)
            drone_->setLocalPosition(
                static_cast<float>(pos.x()),
                static_cast<float>(pos.y()),
                target_z_,
                static_cast<float>(drone_->getOrientation()[2])
            );
            drone_->log("Rise completed");
            return "RISE_COMPLETED";
        }

        drone_->setLocalPosition(
            static_cast<float>(pos.x()),
            static_cast<float>(pos.y()),
            target_z_,
            static_cast<float>(drone_->getOrientation()[2])
        );

        return "";
    }

private:
    std::shared_ptr<Drone> drone_;
    float target_z_;
    float tolerance_;
};
