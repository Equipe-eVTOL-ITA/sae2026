#pragma once

#include <cmath>
#include <memory>
#include <string>

#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "drone/PidController.hpp"
#include "drone/movement.hpp"
#include "drone/transformations.hpp"

class GoToBallState : public fsm::State {
public:
    GoToBallState() 
        : fsm::State(), 
          pid_x_(0.0f, 0.0f, 0.0f, 0.0f), pid_y_(0.0f, 0.0f, 0.0f, 0.0f) {}

    void on_enter(fsm::Blackboard &blackboard) override {
        drone_ = *blackboard.get<std::shared_ptr<Drone>>("drone");
        if (drone_ == nullptr) return;

        drone_->log("");
        drone_->log("STATE: GOTO_BALL");

        trigger_distance_ = *blackboard.get<float>("ball_trigger_distance");
        
        // PIDs
        pid_x_ = PidController(
            *blackboard.get<float>("ball_kp_x"), 
            *blackboard.get<float>("ball_ki_x"), 
            *blackboard.get<float>("ball_kd_x"), 
            0.0f); // Setpoint is 0 (center of image)
            
        pid_y_ = PidController(
            *blackboard.get<float>("ball_kp_y"), 
            *blackboard.get<float>("ball_ki_y"), 
            *blackboard.get<float>("ball_kd_y"), 
            0.0f);

        pid_x_.reset();
        pid_y_.reset();
    }

    std::string act(fsm::Blackboard &blackboard) override {
        if (drone_ == nullptr) return "ERROR";

        bool is_detected = false;
        if (blackboard.contains("ball_is_detected")) {
            is_detected = *blackboard.get<bool>("ball_is_detected");
        }

        if (!is_detected) {
            move_local_by_waypoint(drone_, drone_->getLocalPosition(), 0.0f);
            return "BALL_LOST";
        }

        float dist = 999.0f;
        float center_x = 0.0f;
        float center_y = 0.0f;

        if (blackboard.contains("ball_distance")) {
            dist = *blackboard.get<float>("ball_distance");
        }
        if (blackboard.contains("ball_center_x")) {
            center_x = *blackboard.get<float>("ball_center_x");
        }
        if (blackboard.contains("ball_center_y")) {
            center_y = *blackboard.get<float>("ball_center_y");
        }

        if (dist > 0.0f && dist <= trigger_distance_) {
            move_local_by_waypoint(drone_, drone_->getLocalPosition(), 0.0f);
            drone_->log("Reached ball trigger distance.");
            return "REACHED";
        }

        float vx = pid_x_.compute(center_x); 
        float vy = pid_y_.compute(center_y);

        // Treat vx and vy as position steps (delta)
        Eigen::Vector3d local_delta(vx, vy, 0.0);
        Eigen::Vector3d world_delta = adjust_velocity_using_yaw(local_delta, drone_->getOrientation().z());

        Eigen::Vector3d target_pos = drone_->getLocalPosition() + world_delta;
        
        // speed doesn't matter much if delta is small, move_local_by_waypoint will move step by step
        float speed = std::sqrt(vx*vx + vy*vy);
        if (speed < 0.1f) speed = 0.1f;

        move_local_by_waypoint(drone_, target_pos, speed);

        return "";
    }

private:
    std::shared_ptr<Drone> drone_;
    float trigger_distance_;
    PidController pid_x_;
    PidController pid_y_;
};
