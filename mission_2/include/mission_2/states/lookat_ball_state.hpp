#pragma once

#include <cmath>
#include <memory>
#include <string>

#include "drone/Drone.hpp"
#include "drone/movement.hpp"
#include "fsm/fsm.hpp"

class LookAtBallState : public fsm::State {
public:
    LookAtBallState()
        : fsm::State(),
          stable_counter_(0),
          lost_detection_count_(0),
          max_lost_detection_count_(5),
          x_tolerance_(0.12f),
          y_tolerance_(0.10f),
          stable_frames_(5),
          yaw_kp_(0.45f),
          altitude_kp_(0.25f),
          max_yaw_step_(0.12f),
          max_vertical_step_(0.08f) {}

    void on_enter(fsm::Blackboard &blackboard) override {
        drone_ = *blackboard.get<std::shared_ptr<Drone>>("drone");
        if (drone_ == nullptr) return;

        drone_->log("");
        drone_->log("STATE: LOOKAT_BALL");

        trigger_score_ = blackboard.contains("ball_trigger_score")
            ? *blackboard.get<float>("ball_trigger_score") : 5000.0f;

        x_tolerance_ = blackboard.contains("ball_lookat_x_tolerance")
            ? *blackboard.get<float>("ball_lookat_x_tolerance") : 0.12f;
        y_tolerance_ = blackboard.contains("ball_lookat_y_tolerance")
            ? *blackboard.get<float>("ball_lookat_y_tolerance") : 0.10f;
        stable_frames_ = blackboard.contains("ball_lookat_frames")
            ? static_cast<int>(*blackboard.get<float>("ball_lookat_frames")) : 5;
        if (stable_frames_ < 1) stable_frames_ = 1;

        yaw_kp_ = blackboard.contains("ball_lookat_yaw_kp")
            ? *blackboard.get<float>("ball_lookat_yaw_kp") : 0.45f;
        altitude_kp_ = blackboard.contains("ball_lookat_alt_kp")
            ? *blackboard.get<float>("ball_lookat_alt_kp") : 0.25f;
        max_yaw_step_ = blackboard.contains("ball_lookat_max_yaw_step")
            ? *blackboard.get<float>("ball_lookat_max_yaw_step") : 0.12f;
        max_vertical_step_ = blackboard.contains("ball_lookat_max_vertical_step")
            ? *blackboard.get<float>("ball_lookat_max_vertical_step") : 0.08f;

        max_lost_detection_count_ = blackboard.contains("ball_lost_frames")
            ? static_cast<int>(*blackboard.get<float>("ball_lost_frames")) : 5;
        if (max_lost_detection_count_ < 1) {
            max_lost_detection_count_ = 1;
        }

        entry_z_ = static_cast<float>(drone_->getLocalPosition().z());
        stable_counter_ = 0;
        lost_detection_count_ = 0;
    }

    std::string act(fsm::Blackboard &blackboard) override {
        if (drone_ == nullptr) return "ERROR";

        bool is_detected = blackboard.contains("ball_is_detected") &&
                           *blackboard.get<bool>("ball_is_detected");

        if (!is_detected) {
            ++lost_detection_count_;
            auto cur = drone_->getLocalPosition();
            // Hold at entry_z_ (not cur.z()) to prevent altitude drift from being cemented.
            drone_->setLocalPosition(
                static_cast<float>(cur.x()),
                static_cast<float>(cur.y()),
                entry_z_,
                static_cast<float>(drone_->getOrientation()[2])
            );

            if (lost_detection_count_ < max_lost_detection_count_) {
                return "";
            }

            drone_->log("Ball lost during look-at phase. Returning to search.");
            return "BALL_LOST";
        }

        lost_detection_count_ = 0;

        float score = blackboard.contains("ball_target_score")
            ? *blackboard.get<float>("ball_target_score") : 0.0f;
        float x_error = blackboard.contains("ball_x_error")
            ? *blackboard.get<float>("ball_x_error") : 0.0f;
        float y_error = blackboard.contains("ball_y_error")
            ? *blackboard.get<float>("ball_y_error") : 0.0f;

        if (score >= trigger_score_) {
            drone_->log("Ball already close enough for GOTO_BALL.");
            return "BALL_ALIGNED";
        }

        float yaw_adjust = x_error * yaw_kp_;
        if (yaw_adjust > max_yaw_step_) yaw_adjust = max_yaw_step_;
        if (yaw_adjust < -max_yaw_step_) yaw_adjust = -max_yaw_step_;

        // Sign convention (horizontal camera, NED z negative = above ground):
        //   y_error < 0 → ball above center → drone must ASCEND → z more negative → add negative value
        //   y_error > 0 → ball below center → drone must DESCEND → z less negative → add positive value
        // Therefore: vertical_adjust = +y_error (no negation).
        // Previous code had -y_error which reversed the direction → drone descended into ground.
        float vertical_adjust = y_error * altitude_kp_;
        if (vertical_adjust > max_vertical_step_) vertical_adjust = max_vertical_step_;
        if (vertical_adjust < -max_vertical_step_) vertical_adjust = -max_vertical_step_;

        auto cur = drone_->getLocalPosition();

        // Direct setLocalPosition bypasses move_local_by_waypoint's tolerance check
        // (0.1 m). Since vertical_adjust ≤ 0.08 m (max_vertical_step), the waypoint
        // helper's pos_reached = true every tick → altitude is silently ignored.
        float target_yaw = static_cast<float>(drone_->getOrientation()[2]) + yaw_adjust;
        drone_->setLocalPosition(
            static_cast<float>(cur.x()),
            static_cast<float>(cur.y()),
            static_cast<float>(cur.z()) + vertical_adjust,
            target_yaw
        );

        bool centered = std::fabs(x_error) <= x_tolerance_ && std::fabs(y_error) <= y_tolerance_;
        if (centered) {
            ++stable_counter_;
            if (stable_counter_ >= stable_frames_) {
                drone_->log("Ball centered and stabilized. Transitioning to GOTO_BALL.");
                return "BALL_ALIGNED";
            }
        } else {
            stable_counter_ = 0;
        }

        return "";
    }

private:
    std::shared_ptr<Drone> drone_;
    float trigger_score_;
    float entry_z_;
    int stable_counter_;
    int lost_detection_count_;
    int max_lost_detection_count_;
    float x_tolerance_;
    float y_tolerance_;
    int stable_frames_;
    float yaw_kp_;
    float altitude_kp_;
    float max_yaw_step_;
    float max_vertical_step_;
};