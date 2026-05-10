#pragma once

#include <memory>
#include <string>
#include <Eigen/Eigen>

#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"

/**
 * Descends toward the manometer, holds altitude briefly, then waits an
 * additional settling period so the drone is fully stable before PHOTO reads.
 *
 * Blackboard reads:
 *   "manometer_approach_altitude" (float) — NED z to descend to (default -1.9)
 *   "approach_velocity"           (float) — descent speed m/s (default 0.3)
 *   "approach_hold_ticks"         (float) — motor-stabilisation ticks after altitude (default 20)
 *   "approach_settle_ticks"       (float) — sensor-stabilisation ticks before PHOTO (default 40)
 *   "position_tolerance"          (float) — waypoint acceptance radius (default 0.15)
 *
 * Returns: "AT_ALTITUDE"
 */
class ApproachState : public fsm::State {
public:
    ApproachState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {
        drone_ = *blackboard.get<std::shared_ptr<Drone>>("drone");
        if (!drone_) return;

        drone_->log("");
        drone_->log("STATE: APPROACH");

        approach_alt_ = blackboard.contains("manometer_approach_altitude")
            ? *blackboard.get<float>("manometer_approach_altitude") : -1.9f;
        velocity_ = blackboard.contains("approach_velocity")
            ? *blackboard.get<float>("approach_velocity") : 0.3f;
        hold_ticks_ = blackboard.contains("approach_hold_ticks")
            ? static_cast<int>(*blackboard.get<float>("approach_hold_ticks")) : 20;
        settle_ticks_ = blackboard.contains("approach_settle_ticks")
            ? static_cast<int>(*blackboard.get<float>("approach_settle_ticks")) : 40;
        tolerance_ = blackboard.contains("position_tolerance")
            ? *blackboard.get<float>("position_tolerance") : 0.15f;

        auto pos     = drone_->getLocalPosition();
        hold_x_      = static_cast<float>(pos.x());
        hold_y_      = static_cast<float>(pos.y());
        initial_yaw_ = drone_->getOrientation()[2];
        hold_counter_   = 0;
        settle_counter_ = 0;
        phase_          = Phase::DESCEND;

        drone_->log("Descending to z=" + std::to_string(approach_alt_)
                    + " (currently z=" + std::to_string(pos.z()) + ")");
    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void)blackboard;
        if (!drone_) return "ERROR";

        auto pos = drone_->getLocalPosition();

        switch (phase_) {
        case Phase::DESCEND: {
            Eigen::Vector3d target(hold_x_, hold_y_, approach_alt_);
            Eigen::Vector3d diff = target - pos;
            if (diff.norm() < tolerance_) {
                drone_->log("Approach altitude reached. Stabilising motors...");
                phase_ = Phase::HOLD;
            } else {
                Eigen::Vector3d step = pos + (diff.norm() > velocity_
                                             ? diff.normalized() * velocity_
                                             : diff);
                drone_->setLocalPosition(step.x(), step.y(), step.z(), initial_yaw_);
            }
            break;
        }
        case Phase::HOLD:
            drone_->setLocalPosition(hold_x_, hold_y_, approach_alt_, initial_yaw_);
            if (++hold_counter_ >= hold_ticks_) {
                drone_->log("Motors stable. Waiting " + std::to_string(settle_ticks_)
                            + " ticks for sensor stabilisation...");
                phase_ = Phase::SETTLE;
            }
            break;

        case Phase::SETTLE:
            drone_->setLocalPosition(hold_x_, hold_y_, approach_alt_, initial_yaw_);
            if (++settle_counter_ >= settle_ticks_) {
                drone_->log("Approach complete. Ready to measure.");
                return "AT_ALTITUDE";
            }
            break;
        }

        return "";
    }

private:
    enum class Phase { DESCEND, HOLD, SETTLE };

    std::shared_ptr<Drone> drone_;
    float approach_alt_, velocity_, tolerance_;
    float hold_x_, hold_y_, initial_yaw_;
    int   hold_ticks_, hold_counter_;
    int   settle_ticks_, settle_counter_;
    Phase phase_;
};
