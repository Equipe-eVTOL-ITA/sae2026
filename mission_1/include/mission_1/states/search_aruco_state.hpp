#pragma once

#include <memory>
#include <string>
#include <cmath>
#include <algorithm>

#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "drone/movement.hpp"

class SearchArucoState : public fsm::State {
public:
    SearchArucoState() : fsm::State(),
        detection_counter_(0), miss_counter_(0), last_detected_id_(-1),
        min_detections_(3), max_misses_(3) {}

    void on_enter(fsm::Blackboard &blackboard) override {
        drone_ = *blackboard.get<std::shared_ptr<Drone>>("drone");
        if (drone_ == nullptr) return;

        drone_->log("");
        drone_->log("STATE: SEARCH_ARUCO (Archimedean Spiral)");

        step_size_ = 2.5f;
        if (blackboard.contains("aruco_spiral_step"))
            step_size_ = *blackboard.get<float>("aruco_spiral_step");

        velocity_ = 1.0f;
        if (blackboard.contains("search_aruco_velocity"))
            velocity_ = *blackboard.get<float>("search_aruco_velocity");

        arc_step_ = 0.5f;
        if (blackboard.contains("aruco_spiral_arc_step"))
            arc_step_ = *blackboard.get<float>("aruco_spiral_arc_step");

        if (blackboard.contains("aruco_persistence_frames"))
            min_detections_ = static_cast<int>(*blackboard.get<float>("aruco_persistence_frames"));

        // r(θ) = b·θ  →  after 2π radians, radius grows by step_size_
        b_ = step_size_ / (2.0f * static_cast<float>(M_PI));

        detection_counter_ = 0;
        miss_counter_ = 0;
        last_detected_id_ = -1;

        center_z_ = drone_->getLocalPosition().z();

        // Start at a small offset so the first waypoint isn't exactly at (0,0).
        theta_ = 0.1f;
        update_target();
    }

    std::string act(fsm::Blackboard &blackboard) override {
        if (drone_ == nullptr) return "ERROR";

        // --- Persistence filter with miss tolerance ---
        bool is_detected = false;
        if (blackboard.contains("aruco_detected"))
            is_detected = *blackboard.get<bool>("aruco_detected");

        if (is_detected) {
            miss_counter_ = 0;
            int current_id = 0;
            if (blackboard.contains("aruco_id"))
                current_id = *blackboard.get<int>("aruco_id");
            if (current_id == last_detected_id_) {
                detection_counter_++;
            } else {
                detection_counter_ = 1;
                last_detected_id_ = current_id;
            }
            if (detection_counter_ >= min_detections_) {
                blackboard.set<int>("confirmed_aruco_id", last_detected_id_);
                move_local_by_waypoint(drone_, drone_->getLocalPosition(), 0.0f);
                drone_->log("ArUco ID " + std::to_string(last_detected_id_) +
                            " confirmed after " + std::to_string(detection_counter_) + " frames. Transitioning.");
                return "ARUCO_FOUND";
            }
        } else {
            // Only reset counter after max_misses_ consecutive non-detections.
            // This tolerates 1 dropped camera frame (~100ms) without restarting.
            miss_counter_++;
            if (miss_counter_ >= max_misses_) {
                detection_counter_ = 0;
                last_detected_id_ = -1;
            }
        }

        // --- Spiral movement ---
        Eigen::Vector3d current_pos = drone_->getLocalPosition();
        float dist = std::hypot(current_pos.x() - target_pos_.x(),
                                current_pos.y() - target_pos_.y());

        if (dist < 0.15f) {
            float r = b_ * theta_;
            // Adaptive step: keep arc length ≈ arc_step_ at all radii.
            float d_theta = (r > 0.05f) ? (arc_step_ / r) : 0.5f;
            d_theta = std::clamp(d_theta, 0.05f, 0.5f);
            theta_ += d_theta;
            update_target();
        }

        move_local_by_waypoint(drone_, target_pos_, velocity_);

        return "";
    }

private:
    void update_target() {
        float r = b_ * theta_;
        target_pos_.x() = r * std::cos(theta_);
        target_pos_.y() = r * std::sin(theta_);
        target_pos_.z() = center_z_;
    }

    std::shared_ptr<Drone> drone_;
    Eigen::Vector3d target_pos_;

    float step_size_;
    float b_;
    float theta_;
    float arc_step_;
    float velocity_;
    float center_z_;

    int detection_counter_;
    int miss_counter_;
    int last_detected_id_;
    int min_detections_;
    int max_misses_;
};
