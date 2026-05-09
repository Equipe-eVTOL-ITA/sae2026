#pragma once

#include <memory>
#include <string>
#include <Eigen/Eigen>

#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"

/**
 * Descends slowly toward the manometer so it fills more of the camera frame,
 * then holds altitude for a stabilisation period before handing over to PHOTO.
 *
 * Blackboard reads:
 *   "manometer_approach_altitude" (float) — NED z to descend to (default -1.0 m, i.e. ~1 m AGL)
 *   "approach_velocity"           (float) — descent speed m/s (default 0.3)
 *   "approach_hold_ticks"         (float) — ticks to hold at altitude before exiting (default 20)
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
            ? *blackboard.get<float>("manometer_approach_altitude") : -1.0f;
        velocity_ = blackboard.contains("approach_velocity")
            ? *blackboard.get<float>("approach_velocity") : 0.3f;
        hold_ticks_ = blackboard.contains("approach_hold_ticks")
            ? static_cast<int>(*blackboard.get<float>("approach_hold_ticks")) : 20;
        tolerance_ = blackboard.contains("position_tolerance")
            ? *blackboard.get<float>("position_tolerance") : 0.15f;

        auto pos     = drone_->getLocalPosition();
        hold_x_      = static_cast<float>(pos.x());
        hold_y_      = static_cast<float>(pos.y());
        initial_yaw_ = drone_->getOrientation()[2];
        hold_counter_ = 0;
        phase_        = Phase::DESCEND;

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
                drone_->log("Approach altitude reached. Stabilising...");
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
                drone_->log("Approach complete. Ready to measure.");
                return "AT_ALTITUDE";
            }
            break;
        }

        return "";
    }

private:
    enum class Phase { DESCEND, HOLD };

    std::shared_ptr<Drone> drone_;
    float approach_alt_, velocity_, tolerance_;
    float hold_x_, hold_y_, initial_yaw_;
    int   hold_ticks_, hold_counter_;
    Phase phase_;
};
