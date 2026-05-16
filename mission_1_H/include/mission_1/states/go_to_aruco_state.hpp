#pragma once

#include <memory>
#include <string>
#include <cmath>

#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "drone/movement.hpp"

class GoToArucoState : public fsm::State {
public:
    GoToArucoState() : fsm::State(),
        entry_z_(0.0f), kd_x_(0.0f), kd_y_(0.0f),
        err_x_prev_(0.0f), err_y_prev_(0.0f),
        align_frames_when_known_(5), align_frames_when_unknown_(10) {}

    void on_enter(fsm::Blackboard &blackboard) override {
        drone_ = *blackboard.get<std::shared_ptr<Drone>>("drone");
        if (drone_ == nullptr) return;

        drone_->log("");
        drone_->log("STATE: GO_TO_ARUCO");

        if (blackboard.contains("aruco_tolerance")) {
            tolerance_ = *blackboard.get<float>("aruco_tolerance");
        } else {
            tolerance_ = 0.1f;
        }

        kp_x_ = blackboard.get<float>("aruco_kp_x") ? *blackboard.get<float>("aruco_kp_x") : 0.5f;
        kp_y_ = blackboard.get<float>("aruco_kp_y") ? *blackboard.get<float>("aruco_kp_y") : 0.5f;

        align_frames_when_known_ = blackboard.contains("aruco_align_frames")
            ? static_cast<int>(*blackboard.get<float>("aruco_align_frames")) : 5;
        align_frames_when_unknown_ = align_frames_when_known_ * 2;

        entry_z_ = static_cast<float>(drone_->getLocalPosition().z());

        kd_x_ = blackboard.contains("aruco_kd_x") ? *blackboard.get<float>("aruco_kd_x") : 0.0f;
        kd_y_ = blackboard.contains("aruco_kd_y") ? *blackboard.get<float>("aruco_kd_y") : 0.0f;

        cam_scale_ = blackboard.contains("base_cam_scale")
            ? *blackboard.get<float>("base_cam_scale") : 0.7f;

        err_x_prev_ = 0.0f;
        err_y_prev_ = 0.0f;

        aligned_counter_ = 0;
        miss_counter_ = 0;
        tick_         = 0;
    }

    std::string act(fsm::Blackboard &blackboard) override {
        if (drone_ == nullptr) return "ERROR";

        bool is_detected = false;
        if (blackboard.contains("aruco_detected")) {
            is_detected = *blackboard.get<bool>("aruco_detected");
        }

        if (!is_detected) {
            miss_counter_++;
            if (miss_counter_ > max_misses_) {
                drone_->log("ArUco lost! Returning to search.");
                return "ARUCO_LOST";
            }

            // Kinematic prediction: the ArUco is fixed on the ground, so its
            // apparent position in the camera moves opposite to the drone's velocity.
            //   d_err_x/dt = -vy_frd / (alt × cam_scale)   (lateral)
            //   d_err_y/dt = -vx_frd / (alt × cam_scale)   (forward)
            // Updating err_prev with the prediction has two benefits:
            //   1. Drone keeps moving toward the predicted ArUco position.
            //   2. On re-detection, d_err = (measured - predicted) ≈ 0 → no spike.
            auto vel = drone_->getLocalVelocity();   // FRD frame
            float alt = std::max(0.3f, -static_cast<float>(drone_->getLocalPosition().z()));
            constexpr float dt = 0.05f;
            err_x_prev_ = std::clamp(err_x_prev_ - static_cast<float>(vel.y()) / (alt * cam_scale_) * dt, -1.0f, 1.0f);
            err_y_prev_ = std::clamp(err_y_prev_ - static_cast<float>(vel.x()) / (alt * cam_scale_) * dt, -1.0f, 1.0f);

            // Continue toward predicted position at half speed (no derivative during miss)
            float max_v = 1.0f;
            float vx = std::clamp(-err_y_prev_ * kp_x_, -max_v * 0.5f, max_v * 0.5f);
            float vy = std::clamp( err_x_prev_ * kp_y_, -max_v * 0.5f, max_v * 0.5f);
            move_local_by_speed(drone_, vx, vy, 0.0f);
            return "";
        }

        miss_counter_ = 0;

        float err_x = *blackboard.get<float>("aruco_x_error");
        float err_y = *blackboard.get<float>("aruco_y_error");

        // PD controller: camera frame → drone FRD frame
        // D term damps oscillation when approaching center (like SAE 2025 approach)
        // Note: err_x_prev_ contains the kinematic prediction if coming from a miss,
        // so d_err ≈ (measured - predicted) rather than raw derivative → no spike.
        float d_err_x = (err_x - err_x_prev_) / 0.05f;
        float d_err_y = (err_y - err_y_prev_) / 0.05f;
        err_x_prev_ = err_x;
        err_y_prev_ = err_y;

        float vx = -(err_y * kp_x_ + d_err_y * kd_x_);  // FRD forward
        float vy =  (err_x * kp_y_ + d_err_x * kd_y_);   // FRD right

        float max_v = 1.0f;
        vx = std::clamp(vx, -max_v, max_v);
        vy = std::clamp(vy, -max_v, max_v);

        // Use velocity setpoints (more responsive than position steps — mirrors 2025 align)
        move_local_by_speed(drone_, vx, vy, 0.0f);

        bool target_calculated = blackboard.contains("target_calculated") &&
                                 *blackboard.get<bool>("target_calculated");

        if (std::abs(err_x) < tolerance_ && std::abs(err_y) < tolerance_) {
            aligned_counter_++;
        } else {
            aligned_counter_ = 0;
        }

        int required_aligned = target_calculated ? align_frames_when_known_ : align_frames_when_unknown_;

        if (tick_++ % 10 == 0)
            drone_->log("ArUco err=(" + std::to_string(err_x) + "," + std::to_string(err_y)
                        + ") aligned=" + std::to_string(aligned_counter_)
                        + "/" + std::to_string(required_aligned));

        if (aligned_counter_ > required_aligned) {
            if (target_calculated) {
                drone_->log("ArUco Aligned and Target Calculated!");
                std::string target = *blackboard.get<std::string>("target_base");
                drone_->log("Target Base is: " + target);

                bool target_base_in_sight = false;
                if (blackboard.contains("target_base_in_sight")) {
                    target_base_in_sight = *blackboard.get<bool>("target_base_in_sight");
                }

                if (target_base_in_sight) {
                    return "KNOWN_BASE";
                } else {
                    /*
                    Pegar o vetor posição relativa entre o marcador aruco encontrado e a base de lançamento
                    Tomar a direção perpendicular a esse vetor, no sentido horário e também antihorário
                    Esses vetores serão passados para o estado de busca em H.
                    O estado de busca em H vai seguir primeiro na direção do primeiro vetor (o do sentido horário) e realizar a primeira parte da busca
                    Depois ele vai seguir na direção do segundo vetor (sentido anti-horário) e realizar a segunda parte da busca.
                    O caminho é: base de lançamento -> marcador aruco -> volta para a base de lancamento -> segue na direção do primeiro vetor -> realiza parte 1 da busca -> volta para a base de lançamento -> segue na direção do segundo vetor -> realiza parte 2 da busca
                    */
                    return "UNKNOWN_BASE";
                }
            } else {
                drone_->log("Aligned but shape unknown. Descending for closer look.");
                return "SHAPE_UNKNOWN";
            }
        }

        return "";
    }

private:
    std::shared_ptr<Drone> drone_;
    float tolerance_;
    float kp_x_, kp_y_;
    float kd_x_, kd_y_;
    float entry_z_;
    float cam_scale_;         // camera field-of-view scale for kinematic prediction
    float err_x_prev_, err_y_prev_;
    int aligned_counter_;
    int miss_counter_;
    int tick_;
    int align_frames_when_known_;
    int align_frames_when_unknown_;
    static constexpr int max_misses_ = 30;  // 1.5s at 20Hz — allows for inertia settling after spiral
};
