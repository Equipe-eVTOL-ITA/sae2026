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

/**
 * Three-phase hose alignment:
 *
 *   Phase 1 — TRANSLATE:  XY PD only (no yaw), loose tolerance.
 *             Centres the drone over the hose centre-of-mass before any rotation.
 *
 *   Phase 2 — YAW:        yaw PID only (vx=vy=0).
 *             Rotates to match hose orientation, keeping position fixed.
 *
 *   Phase 3 — FINE:       XY PD only (no yaw), tight tolerance.
 *             Corrects any lateral drift that occurred during yaw phase.
 *
 * Separating the phases prevents the drone from simultaneously
 * translating and rotating, which caused it to slide along the hose axis.
 */
class MangueiraAlignState : public fsm::State {
public:
    MangueiraAlignState(bool align_center, bool align_yaw)
        : MangueiraAlignState() { align_center_ = align_center; align_yaw_ = align_yaw; }

    MangueiraAlignState()
        : fsm::State(),
          align_center_(true),
          align_yaw_(true),
          alignment_timeout_(30.0f),
          kp_(0.4f), kd_(0.10f), max_vel_(0.20f),
          translate_tolerance_(0.25f), translate_frames_(5),
          yaw_tolerance_(0.15f), yaw_frames_(5),
          fine_tolerance_(0.10f), fine_frames_(8),
          kp_yaw_(0.15f), kd_yaw_(0.05f), max_yaw_rate_(0.5f),
          entry_z_(0.0f),
          err_x_prev_(0.0f), err_y_prev_(0.0f),
          pid_yaw_(0.15f, 0.0f, 0.05f, 0.0f, 0.05f),
          phase_(Phase::TRANSLATE),
          phase_counter_(0), miss_counter_(0), tick_(0) {}

    void on_enter(fsm::Blackboard &blackboard) override {
        drone_ = *blackboard.get<std::shared_ptr<Drone>>("drone");
        if (!drone_) return;

        drone_->log("");
        drone_->log("STATE: MANGUEIRA_ALIGN");

        alignment_timeout_ = blackboard.contains("align_timeout")
            ? *blackboard.get<float>("align_timeout") : 30.0f;

        kp_ = blackboard.contains("align_kp")
            ? *blackboard.get<float>("align_kp") : 0.4f;
        kd_ = blackboard.contains("align_kd")
            ? *blackboard.get<float>("align_kd") : 0.10f;
        max_vel_ = blackboard.contains("max_horizontal_velocity_align")
            ? *blackboard.get<float>("max_horizontal_velocity_align") : 0.20f;

        translate_tolerance_ = blackboard.contains("align_translate_tolerance")
            ? *blackboard.get<float>("align_translate_tolerance") : 0.25f;
        translate_frames_ = blackboard.contains("align_translate_frames")
            ? static_cast<int>(*blackboard.get<float>("align_translate_frames")) : 5;
        yaw_tolerance_ = blackboard.contains("align_tolerance_yaw")
            ? *blackboard.get<float>("align_tolerance_yaw") : 0.15f;
        yaw_frames_ = blackboard.contains("align_yaw_frames")
            ? static_cast<int>(*blackboard.get<float>("align_yaw_frames")) : 5;
        fine_tolerance_ = blackboard.contains("align_fine_tolerance")
            ? *blackboard.get<float>("align_fine_tolerance") : 0.10f;
        fine_frames_ = blackboard.contains("align_fine_frames")
            ? static_cast<int>(*blackboard.get<float>("align_fine_frames")) : 8;

        kp_yaw_ = blackboard.contains("align_kp_yaw")
            ? *blackboard.get<float>("align_kp_yaw") : 0.15f;
        const float ki_yaw = blackboard.contains("align_ki_yaw")
            ? *blackboard.get<float>("align_ki_yaw") : 0.0f;
        kd_yaw_ = blackboard.contains("align_kd_yaw")
            ? *blackboard.get<float>("align_kd_yaw") : 0.05f;
        max_yaw_rate_ = blackboard.contains("align_max_yaw_rate")
            ? *blackboard.get<float>("align_max_yaw_rate") : 0.5f;

        entry_z_ = static_cast<float>(drone_->getLocalPosition().z());

        err_x_prev_ = 0.0f;
        err_y_prev_ = 0.0f;
        phase_         = Phase::TRANSLATE;
        phase_counter_ = 0;
        miss_counter_  = 0;
        tick_          = 0;

        pid_yaw_ = PidController(kp_yaw_, ki_yaw, kd_yaw_, 0.0f, 0.05f);
        pid_yaw_.reset();

        start_time_ = std::chrono::steady_clock::now();

        drone_->log("kp=" + std::to_string(kp_) + " kd=" + std::to_string(kd_) +
                    " translate_tol=" + std::to_string(translate_tolerance_) +
                    " yaw_tol=" + std::to_string(yaw_tolerance_) +
                    " fine_tol=" + std::to_string(fine_tolerance_));
    }

    std::string act(fsm::Blackboard &blackboard) override {
        if (!drone_) return "ERROR";

        // Global timeout
        auto elapsed_s = std::chrono::duration<float>(
            std::chrono::steady_clock::now() - start_time_).count();
        if (elapsed_s > alignment_timeout_) {
            drone_->log("Mangueira alignment timeout");
            return "HOSE_LOST";
        }

        auto pos = drone_->getLocalPosition();

        // Z is now held by setMixedSetpoint (position mode) — no manual vz needed.

        // Hose detection check
        const bool have_centers = blackboard.contains("hose_center_x") &&
                                   blackboard.contains("hose_center_y");
        bool hose_in_sight = blackboard.contains("hose_in_sight")
            ? *blackboard.get<bool>("hose_in_sight") : false;

        if (!have_centers || !hose_in_sight) {
            // Hold position; reset derivative state to suppress spike on re-detect
            err_x_prev_ = 0.0f;
            err_y_prev_ = 0.0f;
            pid_yaw_.reset();
            drone_->setLocalPosition(
                static_cast<float>(pos.x()),
                static_cast<float>(pos.y()),
                entry_z_,
                static_cast<float>(drone_->getOrientation()[2]));
            if (++miss_counter_ >= 3) phase_counter_ = 0;
            if (tick_++ % 20 == 0) {
                drone_->log("MANGUEIRA_ALIGN: no valid detection — holding (aligned=" +
                            std::to_string(phase_counter_) + " in_sight=0)");
            }
            return "";
        }

        // Seed prev on first tick after miss
        if (miss_counter_ > 0) {
            err_x_prev_ = (blackboard.contains("hose_center_x"))
                ? (*blackboard.get<float>("hose_center_x") - 0.5f) * 2.0f : 0.0f;
            err_y_prev_ = (blackboard.contains("hose_center_y"))
                ? (*blackboard.get<float>("hose_center_y") - 0.5f) * 2.0f : 0.0f;
        }
        miss_counter_ = 0;

        float center_x = *blackboard.get<float>("hose_center_x");
        float center_y = *blackboard.get<float>("hose_center_y");
        float hose_angle = blackboard.contains("hose_angle_error")
            ? *blackboard.get<float>("hose_angle_error") : 0.0f;

        // Normalize to [-1, 1]
        float err_x = (center_x - 0.5f) * 2.0f;
        float err_y = (center_y - 0.5f) * 2.0f;
        float err_mag = std::sqrt(err_x * err_x + err_y * err_y);

        // PD for XY
        float d_err_x = (err_x - err_x_prev_) / 0.05f;
        float d_err_y = (err_y - err_y_prev_) / 0.05f;
        err_x_prev_ = err_x;
        err_y_prev_ = err_y;

        float vx = std::clamp(-(err_y * kp_ + d_err_y * kd_), -max_vel_, max_vel_);
        float vy = std::clamp( (err_x * kp_ + d_err_x * kd_), -max_vel_, max_vel_);

        // yawspeed_cmd removed — Phase 2 now uses incremental setLocalPosition.

        // ── Phase dispatch ────────────────────────────────────────────────────
        switch (phase_) {

        case Phase::TRANSLATE: {
            // Move to hose centre — XY velocity + Z position hold (no yaw correction)
            drone_->setMixedSetpoint(vx, vy, entry_z_,
                static_cast<float>(drone_->getOrientation()[2]));
            if (err_mag < translate_tolerance_) {
                if (++phase_counter_ >= translate_frames_) {
                    drone_->log("ALIGN Phase 1 DONE (translate). -> YAW");
                    phase_         = Phase::YAW;
                    phase_counter_ = 0;
                    err_x_prev_    = 0.0f;
                    err_y_prev_    = 0.0f;
                    pid_yaw_.reset();
                }
            } else {
                phase_counter_ = 0;
            }
            if (tick_++ % 10 == 0)
                drone_->log("ALIGN [P1/TRANSLATE] err=(" + std::to_string(err_x) +
                    "," + std::to_string(err_y) + ") mag=" + std::to_string(err_mag) +
                    " vx_vy=(" + std::to_string(vx) + "," + std::to_string(vy) + ")" +
                    " cnt=" + std::to_string(phase_counter_) + "/" + std::to_string(translate_frames_));
            break;
        }

        case Phase::YAW: {
            // 180° ambiguity: hose direction is irrelevant for hooking.
            // Always rotate CCW (anti-horário) — abs() removes sign ambiguity.
            float yaw_step = std::clamp(std::abs(hose_angle) * kp_yaw_, 0.0f, 0.05f);
            drone_->setLocalPosition(
                static_cast<float>(pos.x()),
                static_cast<float>(pos.y()),
                entry_z_,
                static_cast<float>(drone_->getOrientation()[2]) + yaw_step);
            float yaw_err_abs = std::abs(hose_angle);
            if (!align_yaw_ || yaw_err_abs < yaw_tolerance_) {
                if (++phase_counter_ >= yaw_frames_) {
                    drone_->log("ALIGN Phase 2 DONE (yaw). -> FINE");
                    this->aligned_yaw_ = this->drone_->getOrientation()[2];
                    phase_         = Phase::FINE;
                    phase_counter_ = 0;
                    err_x_prev_    = 0.0f;
                    err_y_prev_    = 0.0f;
                }
            } else {
                phase_counter_ = 0;
            }
            if (tick_++ % 10 == 0)
                drone_->log("ALIGN [P2/YAW] hose_angle=" + std::to_string(hose_angle) +
                    " yaw_step=" + std::to_string(yaw_step) +
                    " cnt=" + std::to_string(phase_counter_) + "/" + std::to_string(yaw_frames_));
            break;
        }

        case Phase::FINE: {
            // Fine XY centering — XY velocity + Z position hold, yaw fixed
            drone_->setMixedSetpoint(vx, vy, entry_z_,
                static_cast<float>(this->aligned_yaw_));
                //static_cast<float>(drone_->getOrientation()[2]));
            if (err_mag < fine_tolerance_) {
                if (++phase_counter_ >= fine_frames_) {
                    drone_->log("MANGUEIRA_ALIGN: hose centered (all 3 phases done)!");
                    return "ALIGNED";
                }
            } else {
                phase_counter_ = 0;
            }
            if (tick_++ % 10 == 0)
                drone_->log("ALIGN [P3/FINE] err=(" + std::to_string(err_x) +
                    "," + std::to_string(err_y) + ") mag=" + std::to_string(err_mag) +
                    " vx_vy=(" + std::to_string(vx) + "," + std::to_string(vy) + ")" +
                    " cnt=" + std::to_string(phase_counter_) + "/" + std::to_string(fine_frames_));
            break;
        }

        }

        return "";
    }

    void on_exit(fsm::Blackboard &blackboard) override {
        (void)blackboard;
        if (drone_) drone_->log("Exiting mangueira alignment state");
    }

private:
    enum class Phase { TRANSLATE, YAW, FINE };

    std::shared_ptr<Drone> drone_;

    bool  align_center_;
    bool  align_yaw_;
    float alignment_timeout_;

    float kp_, kd_, max_vel_;
    float translate_tolerance_;  int translate_frames_;
    float yaw_tolerance_;        int yaw_frames_;
    float fine_tolerance_;       int fine_frames_;
    float kp_yaw_, kd_yaw_;
    float max_yaw_rate_;
    float entry_z_;

    float aligned_yaw_;

    float err_x_prev_, err_y_prev_;
    PidController pid_yaw_;

    Phase phase_;
    int   phase_counter_;
    int   miss_counter_;
    int   tick_;

    std::chrono::steady_clock::time_point start_time_;
};
