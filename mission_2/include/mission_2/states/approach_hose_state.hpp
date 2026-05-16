#pragma once

#include <memory>
#include <string>
#include <cmath>

#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "drone/movement.hpp"

/**
 * Moves forward slowly at the RISE altitude until the vertical camera sees
 * the mangueira (hose). Transitions to "HOSE_FOUND" once detected.
 *
 * Blackboard reads:
 *   "hose_in_sight"      (bool)  — set True by mission_2.cpp when mangueira/position
 *                                   arrives; reset False after 500ms of no detection.
 *   "hose_approach_speed" (float) — body-frame forward speed in m/s (default 0.2)
 *
 * Transitions:
 *   "HOSE_FOUND" — mangueira visible → proceed to ALIGN
 */
class ApproachHoseState : public fsm::State {
public:
    ApproachHoseState() : fsm::State(), approach_speed_(0.2f), entry_z_(0.0f) {}

    void on_enter(fsm::Blackboard &blackboard) override {
        drone_ = *blackboard.get<std::shared_ptr<Drone>>("drone");
        if (drone_ == nullptr) return;

        drone_->log("");
        drone_->log("STATE: APPROACH_HOSE");

        approach_speed_ = blackboard.contains("hose_approach_speed")
            ? *blackboard.get<float>("hose_approach_speed") : 0.2f;

        // Fix altitude from the end of RISE so we don't drift vertically
        entry_z_ = static_cast<float>(drone_->getLocalPosition().z());
        entry_yaw_ = static_cast<float>(drone_->getOrientation()[2]);
    }

    std::string act(fsm::Blackboard &blackboard) override {
        if (drone_ == nullptr) return "ERROR";

        bool in_sight = blackboard.contains("hose_in_sight") &&
                        *blackboard.get<bool>("hose_in_sight");

        if (in_sight) {
            // Hold position — ALIGN will take over the lateral correction
            auto cur = drone_->getLocalPosition();
            drone_->setLocalPosition(
                static_cast<float>(cur.x()),
                static_cast<float>(cur.y()),
                entry_z_,
                entry_yaw_
            );
            drone_->log("Mangueira found! Transitioning to ALIGN.");
            return "HOSE_FOUND";
        }

        // Move forward (body frame +X) at fixed altitude
        auto cur = drone_->getLocalPosition();
        float yaw = entry_yaw_;
        float wx  = approach_speed_ * std::cos(yaw);
        float wy  = approach_speed_ * std::sin(yaw);
        constexpr float dt = 0.05f;
        drone_->setLocalPosition(
            static_cast<float>(cur.x()) + wx * dt,
            static_cast<float>(cur.y()) + wy * dt,
            entry_z_,
            yaw
        );

        return "";
    }

private:
    std::shared_ptr<Drone> drone_;
    float approach_speed_;
    float entry_z_;
    float entry_yaw_;
};
