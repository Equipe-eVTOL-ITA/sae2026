#pragma once

#include <memory>
#include <string>
#include <cmath>

#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "drone/movement.hpp"

/**
 * Circular search for the ball, followed by a slow-arc centering phase.
 *
 * Flow:
 *   1. SEARCH: drone follows a circle at full speed, looking outward.
 *   2. CENTER: ball detected → continue arc at reduced speed (ball stays in
 *              frame longer) while adjusting yaw offset toward ball.
 *              When |x_error| < threshold for N frames → "BALL_FOUND".
 *
 * Key parameter: ball_centering_kp is the yaw offset in radians applied
 * per unit of x_error (NOT multiplied by dt). For a 90° horizontal FOV:
 *   kp ≈ 0.78 rad brings x_error=1.0 (edge) to yaw correction of 45°.
 */
class SearchBallState : public fsm::State {
public:
    SearchBallState() : fsm::State(),
        current_angle_(0.0f), entry_z_(0.0f),
        centering_phase_(false), centering_counter_(0),
        centering_miss_counter_(0), centering_miss_tol_(10),
        centering_threshold_(0.15f), centering_kp_(0.8f), centering_frames_(5) {}

    void on_enter(fsm::Blackboard &blackboard) override {
        drone_ = *blackboard.get<std::shared_ptr<Drone>>("drone");
        if (drone_ == nullptr) return;

        drone_->log("");
        drone_->log("STATE: SEARCH_BALL");

        search_speed_  = blackboard.contains("search_speed")
            ? *blackboard.get<float>("search_speed")  : 0.5f;
        search_radius_ = blackboard.contains("search_radius")
            ? *blackboard.get<float>("search_radius") : 3.0f;

        centering_threshold_ = blackboard.contains("ball_centering_threshold")
            ? *blackboard.get<float>("ball_centering_threshold") : 0.15f;
        // centering_kp: radians of yaw offset per unit of x_error (no dt factor).
        // For 90° FOV camera: kp≈0.78 brings x_error=1.0 to a 45° correction.
        centering_kp_ = blackboard.contains("ball_centering_kp")
            ? *blackboard.get<float>("ball_centering_kp") : 0.8f;
        centering_frames_ = blackboard.contains("ball_centering_frames")
            ? static_cast<int>(*blackboard.get<float>("ball_centering_frames")) : 5;
        centering_miss_tol_ = blackboard.contains("ball_centering_miss_tol")
            ? static_cast<int>(*blackboard.get<float>("ball_centering_miss_tol")) : 10;

        entry_z_ = static_cast<float>(drone_->getLocalPosition().z());

        // Canonical circle center: always the FIRST entry position.
        // Never update on re-entry (BALL_LOST → SEARCH_BALL) so the
        // circle stays centered at the takeoff/initial search point.
        if (!blackboard.contains("search_ball_center_x")) {
            blackboard.set<float>("search_ball_center_x",
                static_cast<float>(drone_->getLocalPosition().x()));
            blackboard.set<float>("search_ball_center_y",
                static_cast<float>(drone_->getLocalPosition().y()));
        }
        start_position_.x() = *blackboard.get<float>("search_ball_center_x");
        start_position_.y() = *blackboard.get<float>("search_ball_center_y");
        start_position_.z() = entry_z_;

        // Preserve angle across re-entries so we don't re-sweep covered area.
        // On first entry current_angle_ is the constructor default (0.0).
        // On re-entries it keeps its value from the previous run.
        if (!blackboard.contains("search_ball_angle_init")) {
            blackboard.set<bool>("search_ball_angle_init", true);
            current_angle_ = static_cast<float>(drone_->getOrientation()[2]);
        }

        centering_phase_        = false;
        centering_counter_      = 0;
        centering_miss_counter_ = 0;
    }

    std::string act(fsm::Blackboard &blackboard) override {
        if (drone_ == nullptr) return "ERROR";

        bool is_detected = blackboard.contains("ball_is_detected") &&
                           *blackboard.get<bool>("ball_is_detected");

        float x_error = (blackboard.contains("ball_x_error") && is_detected)
            ? *blackboard.get<float>("ball_x_error") : 0.0f;

        // ── Phase 2: CENTER (slow arc + yaw offset toward ball) ──────────
        if (centering_phase_) {
            // Advance arc at 15% of search speed — keeps ball in FOV much longer
            // than stopping completely while still allowing visual tracking.
            float slow_arc = (search_speed_ * 0.15f / search_radius_) * 0.05f;
            current_angle_ += slow_arc;
            if (current_angle_ >  M_PI) current_angle_ -= 2.0f * M_PI;
            if (current_angle_ < -M_PI) current_angle_ += 2.0f * M_PI;

            Eigen::Vector3d target = start_position_;
            target.x() += search_radius_ * std::cos(current_angle_);
            target.y() += search_radius_ * std::sin(current_angle_);

            if (!is_detected) {
                centering_miss_counter_++;
                // Hold slow arc even during brief misses (blackboard filter
                // means ball_is_detected stays True for ~1s, so this is a
                // genuine loss after the filter expires).
                move_local_constant_step(drone_, target, search_speed_ * 0.15f,
                                       0.1f, current_angle_);
                if (centering_miss_counter_ >= centering_miss_tol_) {
                    centering_phase_        = false;
                    centering_counter_      = 0;
                    centering_miss_counter_ = 0;
                    drone_->log("Ball lost during centering. Resuming search.");
                }
                return "";
            }

            centering_miss_counter_ = 0;

            // Yaw offset from arc direction: kp is in radians (no dt factor).
            // x_error = -0.94 → offset = -0.94 * 0.8 = -0.75 rad ≈ 43° left.
            // This immediately points the camera toward the ball.
            float target_yaw = current_angle_ + x_error * centering_kp_;

            move_local_constant_step(drone_, target, search_speed_ * 0.15f,
                                   0.1f, target_yaw);

            if (std::abs(x_error) < centering_threshold_) {
                centering_counter_++;
                if (centering_counter_ >= centering_frames_) {
                    drone_->log("Ball centered (x_err=" +
                        std::to_string(x_error) + "). Transitioning.");
                    return "BALL_FOUND";
                }
            } else {
                centering_counter_ = 0;
            }
            return "";
        }

        // ── Phase 1: SEARCH (full-speed circle) ───────────────────────────
        if (is_detected) {
            drone_->log("Ball detected at x_err=" + std::to_string(x_error) +
                        ". Slowing arc for centering...");
            centering_phase_        = true;
            centering_counter_      = 0;
            centering_miss_counter_ = 0;
            return "";
        }

        float angular_increment = (search_speed_ / search_radius_) * 0.05f;
        current_angle_ += angular_increment;
        if (current_angle_ >  M_PI) current_angle_ -= 2.0f * M_PI;
        if (current_angle_ < -M_PI) current_angle_ += 2.0f * M_PI;

        Eigen::Vector3d target_pos = start_position_;
        target_pos.x() += search_radius_ * std::cos(current_angle_);
        target_pos.y() += search_radius_ * std::sin(current_angle_);

        move_local_constant_step(drone_, target_pos, search_speed_, 0.1f, current_angle_);
        return "";
    }

private:
    std::shared_ptr<Drone> drone_;
    float search_speed_;
    float search_radius_;
    float current_angle_;
    float entry_z_;
    Eigen::Vector3d start_position_;

    bool  centering_phase_;
    int   centering_counter_;
    int   centering_miss_counter_;
    int   centering_miss_tol_;
    float centering_threshold_;
    float centering_kp_;        // radians of yaw offset per unit x_error (no dt)
    int   centering_frames_;
};
