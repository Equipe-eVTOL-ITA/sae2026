#pragma once

#include <cmath>
#include <memory>
#include <string>
#include <chrono>

#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "drone/movement.hpp"

class RiseState : public fsm::State {
public:
    RiseState() : fsm::State(), target_z_(0.0f), tolerance_(0.15f), 
                  climb_rate_(0.3f), current_z_(0.0f), 
                  start_z_(0.0f), hose_seen_frames_(0), required_hose_frames_(3) {
        state_start_time_ = std::chrono::high_resolution_clock::now();
    }

    void on_enter(fsm::Blackboard &blackboard) override {
        drone_ = *blackboard.get<std::shared_ptr<Drone>>("drone");
        if (drone_ == nullptr) return;

        drone_->log("");
        drone_->log("STATE: RISE (gradual ascent with hose search)");

        // Get current position
        auto pos = drone_->getLocalPosition();
        start_z_ = static_cast<float>(pos.z());
        current_z_ = start_z_;

        // Absolute target altitude (NED, negative = above ground).
        // Using a fixed altitude prevents cumulative drift when re-entering.
        target_z_ = blackboard.contains("rise_target_z")
            ? *blackboard.get<float>("rise_target_z")
            : start_z_ - 2.0f;

        tolerance_ = blackboard.contains("position_tolerance")
            ? *blackboard.get<float>("position_tolerance") : 0.15f;

        // Climb rate (m/s) - how fast to ascend
        climb_rate_ = blackboard.contains("rise_climb_rate")
            ? *blackboard.get<float>("rise_climb_rate") : 0.3f;

        state_start_time_ = std::chrono::high_resolution_clock::now();
        hose_seen_frames_ = 0;
        if (blackboard.contains("rise_hose_confirm_frames")) {
            float *p = blackboard.get<float>("rise_hose_confirm_frames");
            required_hose_frames_ = p ? static_cast<int>(*p) : 3;
        } else {
            required_hose_frames_ = 3;
        }

        rise_timeout_s_ = blackboard.contains("rise_timeout_s")
            ? *blackboard.get<float>("rise_timeout_s") : 12.0f;

        drone_->log("Rising to z=" + std::to_string(target_z_) +
                    " with climb rate=" + std::to_string(climb_rate_) + " m/s" +
                    " timeout=" + std::to_string(rise_timeout_s_) + "s");
    }

    std::string act(fsm::Blackboard &blackboard) override {
        if (drone_ == nullptr) return "ERROR";

        auto pos = drone_->getLocalPosition();
        float current_z = static_cast<float>(pos.z());

        // Calculate elapsed time since entering this state
        auto now = std::chrono::high_resolution_clock::now();
        float elapsed_time = std::chrono::duration<float>(now - state_start_time_).count();

        // Calculate desired altitude based on climb rate
        float desired_z = start_z_ - (climb_rate_ * elapsed_time);

        // Clamp to target altitude (don't go below target)
        if ((target_z_ < start_z_ && desired_z < target_z_) ||
            (target_z_ > start_z_ && desired_z > target_z_)) {
            desired_z = target_z_;
        }

        current_z_ = desired_z;

        // Check for hose detection
        bool hose_in_sight = false;
        if (blackboard.contains("hose_in_sight")) {
            hose_in_sight = *blackboard.get<bool>("hose_in_sight");
        }

        // Calculate altitude error
        float alt_err = std::abs(target_z_ - current_z);

        // Count consecutive frames where hose is in sight before transitioning
        if (hose_in_sight) {
            hose_seen_frames_++;
        } else {
            hose_seen_frames_ = 0;
        }

        if (hose_seen_frames_ >= required_hose_frames_) {
            drone_->log("Hose seen for required frames during rise — transitioning to ALIGN");
            return "HOSE_DETECTED";
        }

        // If reached target altitude, stay at that altitude and continue searching
        if (alt_err < tolerance_) {
            // Hold at target altitude
            drone_->setLocalPosition(
                static_cast<float>(pos.x()),
                static_cast<float>(pos.y()),
                target_z_,
                static_cast<float>(drone_->getOrientation()[2])
            );
            // Timeout: if hose never detected within rise_timeout_s_, proceed anyway
            if (elapsed_time > rise_timeout_s_) {
                drone_->log("RISE: timeout — no hose detected, proceeding to APPROACH_HOSE");
                return "RISE_COMPLETED";
            }
            // Log periodically (every 20 cycles)
            static int log_counter = 0;
            if (++log_counter % 20 == 0) {
                drone_->log("Reached target altitude, waiting for hose detection ("
                    + std::to_string(static_cast<int>(rise_timeout_s_ - elapsed_time)) + "s left)");
            }
            return "";  // Stay in RISE state
        }

        // Continue climbing gradually
        drone_->setLocalPosition(
            static_cast<float>(pos.x()),
            static_cast<float>(pos.y()),
            desired_z,
            static_cast<float>(drone_->getOrientation()[2])
        );

        return "";  // Stay in RISE state
    }

private:
    std::shared_ptr<Drone> drone_;
    float target_z_;
    float tolerance_;
    float climb_rate_;           // m/s
    float current_z_;            // Current target altitude during climb
    float start_z_;              // Starting altitude when entering state
    std::chrono::high_resolution_clock::time_point state_start_time_;
    int   hose_seen_frames_;
    int   required_hose_frames_;
    float rise_timeout_s_;
};

