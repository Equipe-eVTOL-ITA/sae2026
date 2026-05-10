#pragma once

#include <memory>
#include <string>
#include <cmath>

#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "drone/movement.hpp"

class SearchBaseState : public fsm::State {
public:
    SearchBaseState() : fsm::State(),
        center_z_(0.0f), base_detection_counter_(0), base_miss_counter_(0),
        min_base_detections_(3), current_leg_length_(1.0f), current_leg_(0),
        steps_in_current_length_(0) {}

    void on_enter(fsm::Blackboard &blackboard) override {
        drone_ = *blackboard.get<std::shared_ptr<Drone>>("drone");
        if (drone_ == nullptr) return;

        drone_->log("");
        drone_->log("STATE: SEARCH_BASE (Square Spiral)");

        // Fixed altitude: captured on first entry, reused on all re-entries to
        // prevent the altitude ratchet caused by PX4 drift between cycles.
        if (!blackboard.contains("search_base_altitude"))
            blackboard.set<float>("search_base_altitude",
                static_cast<float>(drone_->getLocalPosition().z()));
        center_z_ = *blackboard.get<float>("search_base_altitude");

        step_size_ = blackboard.contains("base_spiral_step")
            ? *blackboard.get<float>("base_spiral_step") : 1.0f;

        min_base_detections_ = blackboard.contains("base_persistence_frames")
            ? static_cast<int>(*blackboard.get<float>("base_persistence_frames")) : 3;
        base_detection_counter_ = 0;
        base_miss_counter_       = 0;

        bool first_entry = !blackboard.contains("search_base_visited");
        blackboard.set<bool>("search_base_visited", true);

        Eigen::Vector3d spiral_center;

        if (first_entry) {
            // Mirror SearchArucoState: first search always starts from local origin.
            spiral_center = Eigen::Vector3d(0.0, 0.0, center_z_);
            drone_->log("First entry — spiral from home (0,0)");
        } else {
            // Re-entry after BASE_LOST: check if we have a cached world position.
            bool used_cache = false;
            if (blackboard.contains("target_calculated") &&
                *blackboard.get<bool>("target_calculated") &&
                blackboard.contains("target_base")) {

                std::string target = *blackboard.get<std::string>("target_base");
                std::string key_x  = "known_base_" + target + "_x";

                if (blackboard.contains(key_x)) {
                    float cx = *blackboard.get<float>(key_x);
                    float cy = *blackboard.get<float>("known_base_" + target + "_y");
                    spiral_center = Eigen::Vector3d(cx, cy, center_z_);
                    step_size_ = 0.5f;  // tight local spiral around cached position
                    drone_->log("Cached pos for " + target +
                        " (" + std::to_string(cx) + "," + std::to_string(cy) + ")");
                    used_cache = true;
                }
            }
            if (!used_cache) {
                spiral_center = drone_->getLocalPosition();
                spiral_center.z() = center_z_;
            }
        }

        target_pos_ = spiral_center;
        current_leg_length_      = step_size_;
        current_leg_             = 0;
        steps_in_current_length_ = 0;

        directions_[0] = Eigen::Vector2d( 1.0,  0.0);
        directions_[1] = Eigen::Vector2d( 0.0,  1.0);
        directions_[2] = Eigen::Vector2d(-1.0,  0.0);
        directions_[3] = Eigen::Vector2d( 0.0, -1.0);

        update_target();
    }

    std::string act(fsm::Blackboard &blackboard) override {
        if (drone_ == nullptr) return "ERROR";

        // Persistence filter (mirrors SearchArucoState): require N consecutive
        // detection frames before transitioning to avoid flickering detections.
        bool is_detected = blackboard.contains("target_base_in_sight") &&
                           *blackboard.get<bool>("target_base_in_sight");

        if (is_detected) {
            base_miss_counter_ = 0;
            base_detection_counter_++;
            if (base_detection_counter_ >= min_base_detections_) {
                move_local_by_waypoint(drone_, drone_->getLocalPosition(), 0.0f);
                drone_->log("Target Base confirmed after " +
                    std::to_string(base_detection_counter_) + " frames!");
                return "BASE_FOUND";
            }
        } else {
            base_miss_counter_++;
            if (base_miss_counter_ >= base_miss_tolerance_)
                base_detection_counter_ = 0;
        }

        Eigen::Vector3d current_pos = drone_->getLocalPosition();
        float dist = std::hypot(current_pos.x() - target_pos_.x(),
                                current_pos.y() - target_pos_.y());

        if (dist < 0.15f) {
            current_leg_ = (current_leg_ + 1) % 4;
            steps_in_current_length_++;
            if (steps_in_current_length_ >= 2) {
                current_leg_length_ += step_size_;
                steps_in_current_length_ = 0;
            }
            update_target();
        }

        move_local_by_waypoint(drone_, target_pos_, 0.4f);
        return "";
    }

private:
    void update_target() {
        target_pos_.x() += directions_[current_leg_].x() * current_leg_length_;
        target_pos_.y() += directions_[current_leg_].y() * current_leg_length_;
        target_pos_.z()  = center_z_;  // always maintain fixed search altitude
    }

    std::shared_ptr<Drone> drone_;
    Eigen::Vector3d target_pos_;

    float step_size_;
    float center_z_;
    int   base_detection_counter_;
    int   base_miss_counter_;
    int   min_base_detections_;
    static constexpr int base_miss_tolerance_ = 3;

    float           current_leg_length_;
    int             current_leg_;
    int             steps_in_current_length_;
    Eigen::Vector2d directions_[4];
};
