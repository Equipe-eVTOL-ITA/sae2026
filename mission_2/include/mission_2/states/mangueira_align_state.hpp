#pragma once

#include <Eigen/Eigen>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "drone/PidController.hpp"
#include "drone/movement.hpp"
#include "drone/transformations.hpp"

class MangueiraAlignState : public fsm::State {
public:
    MangueiraAlignState(bool align_center, bool align_yaw)
        : MangueiraAlignState() {
        align_center_ = align_center;
        align_yaw_ = align_yaw;
    }

    MangueiraAlignState()
        : fsm::State(),
          align_center_(true),
          align_yaw_(true),
          alignment_timeout_(30.0f),
          lost_timeout_(6),
          stable_required_frames_(5),
          align_tolerance_x_(0.10f),
          align_tolerance_y_(0.10f),
          yaw_tolerance_(0.05f),
          pid_x_(0.0f, 0.0f, 0.0f, 0.5f),
          pid_y_(0.0f, 0.0f, 0.0f, 0.5f),
          pid_yaw_(0.0f, 0.0f, 0.0f, 0.0f),
          target_yaw_(0.0f),
          stable_count_(0),
          lost_detection_count_(0) {}

    void on_enter(fsm::Blackboard &blackboard) override {
        drone_ = *blackboard.get<std::shared_ptr<Drone>>("drone");
        if (drone_ == nullptr) return;

        drone_->log("");
        drone_->log("STATE: MANGUEIRA_ALIGN");

        alignment_timeout_ = blackboard.contains("align_timeout")
            ? *blackboard.get<float>("align_timeout") : 30.0f;
        lost_timeout_ = blackboard.contains("lost_timeout")
            ? static_cast<int>(*blackboard.get<float>("lost_timeout")) : 6;

        align_tolerance_x_ = blackboard.contains("align_tolerance_x")
            ? *blackboard.get<float>("align_tolerance_x")
            : (blackboard.contains("align_tolerance_y") ? *blackboard.get<float>("align_tolerance_y") : 0.10f);
        align_tolerance_y_ = blackboard.contains("align_tolerance_y")
            ? *blackboard.get<float>("align_tolerance_y")
            : align_tolerance_x_;
        yaw_tolerance_ = blackboard.contains("align_tolerance_yaw")
            ? *blackboard.get<float>("align_tolerance_yaw") : 0.05f;

        stable_required_frames_ = blackboard.contains("align_stable_frames")
            ? static_cast<int>(*blackboard.get<float>("align_stable_frames")) : 5;

        const float kp_x = blackboard.contains("align_kp_x") ? *blackboard.get<float>("align_kp_x") : 0.5f;
        const float ki_x = blackboard.contains("align_ki_x") ? *blackboard.get<float>("align_ki_x") : 0.0f;
        const float kd_x = blackboard.contains("align_kd_x") ? *blackboard.get<float>("align_kd_x") : 0.1f;

        const float kp_y = blackboard.contains("align_kp_y") ? *blackboard.get<float>("align_kp_y") : 0.5f;
        const float ki_y = blackboard.contains("align_ki_y") ? *blackboard.get<float>("align_ki_y") : 0.0f;
        const float kd_y = blackboard.contains("align_kd_y") ? *blackboard.get<float>("align_kd_y") : 0.1f;

        const float kp_yaw = blackboard.contains("align_kp_yaw") ? *blackboard.get<float>("align_kp_yaw") : 0.5f;
        const float ki_yaw = blackboard.contains("align_ki_yaw") ? *blackboard.get<float>("align_ki_yaw") : 0.0f;
        const float kd_yaw = blackboard.contains("align_kd_yaw") ? *blackboard.get<float>("align_kd_yaw") : 0.1f;

        pid_x_ = PidController(kp_x, ki_x, kd_x, 0.5f);
        pid_y_ = PidController(kp_y, ki_y, kd_y, 0.5f);
        pid_yaw_ = PidController(kp_yaw, ki_yaw, kd_yaw, 0.0f);

        pid_x_.reset();
        pid_y_.reset();
        pid_yaw_.reset();

        target_yaw_ = static_cast<float>(drone_->getOrientation()[2]);
        start_time_ = std::chrono::steady_clock::now();
        stable_start_time_ = start_time_;
        stable_count_ = 0;
        lost_detection_count_ = 0;

        drone_->log("Starting mangueira alignment with PID control");
    }

    std::string act(fsm::Blackboard &blackboard) override {
        if (drone_ == nullptr) return "ERROR";

        auto current_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time_);
        if (elapsed.count() > alignment_timeout_) {
            drone_->log("Mangueira alignment timeout");
            return "HOSE_LOST";
        }

        auto pos = drone_->getLocalPosition();
        float current_yaw = static_cast<float>(drone_->getOrientation()[2]);

        bool in_sight = blackboard.contains("hose_in_sight") &&
                        *blackboard.get<bool>("hose_in_sight");

        if (!in_sight) {
            lost_detection_count_++;
            if (lost_detection_count_ > lost_timeout_) {
                drone_->log("Lost mangueira detections for too long");
                return "HOSE_LOST";
            }

            drone_->setLocalPosition(pos.x(), pos.y(), pos.z(), current_yaw);
            return "";
        }

        lost_detection_count_ = 0;

        float center_x = 0.5f;
        float center_y = 0.5f;

        if (blackboard.contains("hose_center_x")) {
            center_x = *blackboard.get<float>("hose_center_x");
        } else if (blackboard.contains("hose_offset_x")) {
            center_x = *blackboard.get<float>("hose_offset_x");
        }

        if (blackboard.contains("hose_center_y")) {
            center_y = *blackboard.get<float>("hose_center_y");
        } else if (blackboard.contains("hose_offset_y")) {
            center_y = *blackboard.get<float>("hose_offset_y");
        }

        float hose_angle = 0.0f;
        if (blackboard.contains("hose_angle_error")) {
            hose_angle = *blackboard.get<float>("hose_angle_error");
        }

        float pid_output_x = align_center_ ? pid_x_.compute(center_x) : 0.0f;
        float pid_output_y = align_center_ ? pid_y_.compute(center_y) : 0.0f;
        float pid_output_yaw = align_yaw_ ? pid_yaw_.compute(hose_angle) : 0.0f;

        // INVERTED XY MOVEMENT HERE:
        // The mangueira detector / current FSM convention needs the planar command
        // flipped so the drone moves toward the hose instead of away from it.
        Eigen::Vector3d local_delta(-pid_output_x, -pid_output_y, 0.0);
        Eigen::Vector3d world_delta = adjust_velocity_using_yaw(local_delta, current_yaw);
        Eigen::Vector3d target_pos = drone_->getLocalPosition() + world_delta;

        target_yaw_ += pid_output_yaw * 0.1f;
        if (target_yaw_ > M_PI) target_yaw_ -= 2.0f * M_PI;
        if (target_yaw_ < -M_PI) target_yaw_ += 2.0f * M_PI;

        float pos_err_x = std::abs(center_x - 0.5f);
        float pos_err_y = std::abs(center_y - 0.5f);
        float angle_err = std::abs(hose_angle);

        if (pos_err_x < align_tolerance_x_ && pos_err_y < align_tolerance_y_ && angle_err < yaw_tolerance_) {
            if (stable_count_ == 0) {
                stable_start_time_ = current_time;
            }
            stable_count_++;

            auto stable_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                current_time - stable_start_time_);

            if (stable_count_ >= stable_required_frames_ || stable_elapsed.count() > 500) {
                drone_->log("Mangueira alignment achieved");
                return "ALIGNED";
            }
        } else {
            stable_count_ = 0;
            stable_start_time_ = current_time;
        }

        float speed = std::max(std::abs(pid_output_x), std::abs(pid_output_y));
        if (speed < 0.1f) speed = 0.1f;

        move_local_by_waypoint(drone_, target_pos, speed, 0.1f, target_yaw_);

        if (stable_count_ % 20 == 0) {
            drone_->log("Aligning mangueira - Pos error x: " + std::to_string(pos_err_x) +
                        ", y: " + std::to_string(pos_err_y) +
                        ", angle error: " + std::to_string(angle_err));
        }

        return "";
    }

    void on_exit(fsm::Blackboard &blackboard) override {
        (void)blackboard;
        if (drone_ != nullptr) {
            drone_->log("Exiting mangueira alignment state");
        }
    }

private:
    std::shared_ptr<Drone> drone_;
    bool align_center_;
    bool align_yaw_;
    float alignment_timeout_;
    int lost_timeout_;
    int stable_required_frames_;
    float align_tolerance_x_;
    float align_tolerance_y_;
    float yaw_tolerance_;

    PidController pid_x_;
    PidController pid_y_;
    PidController pid_yaw_;
    float target_yaw_;

    std::chrono::steady_clock::time_point start_time_;
    std::chrono::steady_clock::time_point stable_start_time_;
    int stable_count_;
    int lost_detection_count_;
};
