#pragma once

#include <memory>
#include <string>
#include <cmath>

#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "drone/movement.hpp"

class InitialArucoSearchState : public fsm::State {
public:
    InitialArucoSearchState() : fsm::State(),
        detection_counter_(0), miss_counter_(0), last_detected_id_(-1),
        min_detections_(3), max_misses_(3) {}

    void on_enter(fsm::Blackboard &blackboard) override {
        drone_ = *blackboard.get<std::shared_ptr<Drone>>("drone");
        if (drone_ == nullptr) return;

        drone_->log("");
        drone_->log("STATE: INITIAL_ARUCO_SEARCH");

        if (blackboard.contains("z_max_search")) {
            z_max_search_ = *blackboard.get<float>("z_max_search");
        } else {
            z_max_search_ = -10.0f; // Default 10 meters up (negative is up)
        }
        
        start_position_ = drone_->getLocalPosition();

        detection_counter_ = 0;
        miss_counter_ = 0;
        last_detected_id_ = -1;
        if (blackboard.contains("aruco_persistence_frames"))
            min_detections_ = static_cast<int>(*blackboard.get<float>("aruco_persistence_frames"));

        blackboard.set<bool>("aruco_detected", false);
    }

    std::string act(fsm::Blackboard &blackboard) override {
        if (drone_ == nullptr) return "ERROR";

        bool is_detected = false;
        if (blackboard.contains("aruco_detected")) {
            is_detected = *blackboard.get<bool>("aruco_detected");
        }

        if (is_detected) {
            int current_id = 0;
            if (blackboard.contains("aruco_id")) {
                current_id = *blackboard.get<int>("aruco_id");
            }
            if (current_id == last_detected_id_) {
                detection_counter_++;
            } else {
                detection_counter_ = 1;
                last_detected_id_ = current_id;
            }
            if (detection_counter_ >= min_detections_) {
                blackboard.set<int>("confirmed_aruco_id", last_detected_id_);
                move_local_constant_step(drone_, drone_->getLocalPosition(), 0.0f);
                drone_->log("ArUco ID " + std::to_string(last_detected_id_) + " confirmed during initial ascent. Transitioning.");
                return "ARUCO_FOUND";
            }
        } else {
            miss_counter_++;
            if (miss_counter_ >= max_misses_) {
                detection_counter_ = 0;
                last_detected_id_ = -1;
            }
        }

        // Elevate purely in Z
        Eigen::Vector3d target_pos = start_position_;
        target_pos.z() = z_max_search_;
        
        // Move slowly upwards. If it reaches the target altitude (within tolerance), start spiral search.
        if (move_local_constant_step(drone_, target_pos, 0.5f, 0.3f)) {
            move_local_by_vel_as_position(drone_, 0.0f, 0.0f, 0.0f); // hold position
            drone_->log("Reached max search altitude. Switching to spiral search.");
            return "MAX_ALTITUDE_REACHED";
        }

        return "";
    }

private:
    std::shared_ptr<Drone> drone_;
    float z_max_search_;
    Eigen::Vector3d start_position_;

    int detection_counter_;
    int miss_counter_;
    int last_detected_id_;
    int min_detections_;
    int max_misses_;
};
