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

        trigger_score_ = *blackboard.get<float>("ball_trigger_score");
        
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

        lost_detection_count_ = 0;
        max_lost_detection_count_ = 5;
        if (blackboard.contains("ball_lost_frames")) {
            max_lost_detection_count_ = static_cast<int>(*blackboard.get<float>("ball_lost_frames"));
        }
        if (max_lost_detection_count_ < 1) {
            max_lost_detection_count_ = 1;
        }
    }

    std::string act(fsm::Blackboard &blackboard) override {
        if (drone_ == nullptr) return "ERROR";

        bool is_detected = false;
        if (blackboard.contains("ball_is_detected")) {
            is_detected = *blackboard.get<bool>("ball_is_detected");
        }

        if (!is_detected) {
            ++lost_detection_count_;
            // speed=0 is a no-op in move_local_constant_step — hold directly
            auto cur = drone_->getLocalPosition();
            drone_->setLocalPosition(
                static_cast<float>(cur.x()), static_cast<float>(cur.y()),
                static_cast<float>(cur.z()), static_cast<float>(drone_->getOrientation()[2]));
            if (lost_detection_count_ < max_lost_detection_count_) {
                return "";
            }
            return "BALL_LOST";
        }

        lost_detection_count_ = 0;

        float score = 0.0f;
        float x_error = 0.0f;
        float y_error = 0.0f;

        if (blackboard.contains("ball_target_score")) {
            score = *blackboard.get<float>("ball_target_score");
        }
        if (blackboard.contains("ball_x_error")) {
            x_error = *blackboard.get<float>("ball_x_error");
        }
        if (blackboard.contains("ball_y_error")) {
            y_error = *blackboard.get<float>("ball_y_error");
        }

        if (score >= trigger_score_) {
            move_local_constant_step(drone_, drone_->getLocalPosition(), 0.0f);
            drone_->log("Reached ball trigger score.");
            return "REACHED";
        }

        // Slow approach with continuous centering correction.
        // Invert lateral sign so positive image x_error (ball to right)
        // produces positive local y (right) movement.
        float vy_lateral = -pid_x_.compute(x_error);
        float vz_vertical = pid_y_.compute(-y_error); // FRD sign correction

        float vx_forward = 0.12f;
        if (blackboard.contains("ball_approach_velocity")) {
            vx_forward = *blackboard.get<float>("ball_approach_velocity");
        }

        float yaw_adjust = x_error * 0.15f;
        if (yaw_adjust > 0.12f) yaw_adjust = 0.12f;
        if (yaw_adjust < -0.12f) yaw_adjust = -0.12f;

        Eigen::Vector3d local_delta(vx_forward, vy_lateral, vz_vertical);
        Eigen::Vector3d world_delta = adjust_velocity_using_yaw(local_delta, drone_->getOrientation().z());
        Eigen::Vector3d target_pos = drone_->getLocalPosition() + world_delta;

        float speed = std::sqrt(vx_forward*vx_forward + vy_lateral*vy_lateral + vz_vertical*vz_vertical);
        if (speed < 0.1f) speed = 0.1f;

        move_local_constant_step(
            drone_,
            target_pos,
            speed,
            0.1f,
            static_cast<float>(drone_->getOrientation()[2]) + yaw_adjust
        );

        return "";
    }

private:
    std::shared_ptr<Drone> drone_;
    float trigger_score_;
    PidController pid_x_;
    PidController pid_y_;
    int lost_detection_count_;
    int max_lost_detection_count_;
};
