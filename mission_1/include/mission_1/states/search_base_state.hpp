#pragma once

#include <memory>
#include <string>
#include <cmath>

#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "drone/movement.hpp"

class SearchBaseState : public fsm::State {
public:
    SearchBaseState() : fsm::State(), current_leg_length_(2.0f), current_leg_(0), steps_in_current_length_(0) {}

    void on_enter(fsm::Blackboard &blackboard) override {
        drone_ = *blackboard.get<std::shared_ptr<Drone>>("drone");
        if (drone_ == nullptr) return;

        drone_->log("");
        drone_->log("STATE: SEARCH_BASE (Square Spiral)");

        start_position_ = drone_->getLocalPosition();
        start_position_.x() = 0.0;
        start_position_.y() = 0.0;
        target_pos_ = start_position_;
        
        if (blackboard.contains("base_spiral_step")) {
            step_size_ = *blackboard.get<float>("base_spiral_step");
        } else {
            step_size_ = 1.0f;
        }

        current_leg_length_ = step_size_; 
        current_leg_ = 0;
        steps_in_current_length_ = 0;
        
        directions_[0] = Eigen::Vector2d(1.0, 0.0);
        directions_[1] = Eigen::Vector2d(0.0, 1.0);
        directions_[2] = Eigen::Vector2d(-1.0, 0.0);
        directions_[3] = Eigen::Vector2d(0.0, -1.0);
        
        update_target();
    }

    std::string act(fsm::Blackboard &blackboard) override {
        if (drone_ == nullptr) return "ERROR";

        bool is_detected = false;
        if (blackboard.contains("target_base_in_sight")) {
            is_detected = *blackboard.get<bool>("target_base_in_sight");
        }

        if (is_detected) {
            move_local_by_waypoint(drone_, drone_->getLocalPosition(), 0.0f); // parar
            drone_->log("Target Base detected during spiral search!");
            return "BASE_FOUND";
        }

        Eigen::Vector3d current_pos = drone_->getLocalPosition();
        float dist = std::sqrt(std::pow(current_pos.x() - target_pos_.x(), 2) + 
                               std::pow(current_pos.y() - target_pos_.y(), 2));

        if (dist < 0.15f) {
            current_leg_ = (current_leg_ + 1) % 4;
            steps_in_current_length_++;
            
            if (steps_in_current_length_ >= 2) {
                current_leg_length_ += step_size_; 
                steps_in_current_length_ = 0;
            }
            
            update_target();
        }

        move_local_by_waypoint(drone_, target_pos_, 0.4f); // Move slowly

        return "";
    }

private:
    void update_target() {
        target_pos_.x() += directions_[current_leg_].x() * current_leg_length_;
        target_pos_.y() += directions_[current_leg_].y() * current_leg_length_;
    }

    std::shared_ptr<Drone> drone_;
    Eigen::Vector3d start_position_;
    Eigen::Vector3d target_pos_;
    
    float current_leg_length_;
    float step_size_;
    int current_leg_;
    int steps_in_current_length_;
    Eigen::Vector2d directions_[4];
};
