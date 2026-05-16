#pragma once

#include <cmath>
#include <memory>
#include <string>

#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "drone/movement.hpp"

/*
 * Alinha o drone sobre a base gabarito usando controle PD sobre os erros
 * normalizados da câmera.
 *
 * Comportamento durante detecção perdida (miss):
 *   Segura posição XY atual + altitude de referência (sem "aproximação") para não
 *   sair da região onde a base estava visível.
 *
 * Parâmetros blackboard:
 *   base_tolerance        — erro normalizado máximo para contar alinhamento (padrão 0.10)
 *   base_kp_x, base_kp_y — ganhos proporcionais
 *   base_kd_x, base_kd_y — ganhos derivativos
 *   base_align_frames     — frames consecutivos para confirmar alinhamento (padrão 10)
 *   base_max_miss_ticks   — ticks de miss antes de BASE_LOST (padrão 60 = 3s)
 *   search_base_altitude  — altitude NED compartilhada com H_SEARCH_BASE (evita oscilação em z)
 */
class GoToBaseState : public fsm::State {
public:
    GoToBaseState() : fsm::State(),
        tolerance_(0.10f), kp_x_(0.5f), kp_y_(0.5f),
        kd_x_(0.0f), kd_y_(0.0f), entry_z_(0.0f),
        err_x_prev_(0.0f), err_y_prev_(0.0f),
        aligned_counter_(0), align_frames_(10),
        miss_counter_(0), max_misses_(60), tick_(0) {}

    void on_enter(fsm::Blackboard &blackboard) override {
        drone_ = *blackboard.get<std::shared_ptr<Drone>>("drone");
        if (!drone_) return;

        drone_->log("");
        drone_->log("STATE: GO_TO_BASE");

        tolerance_ = blackboard.contains("base_tolerance")
            ? *blackboard.get<float>("base_tolerance") : 0.10f;
        kp_x_ = blackboard.contains("base_kp_x") ? *blackboard.get<float>("base_kp_x") : 0.5f;
        kp_y_ = blackboard.contains("base_kp_y") ? *blackboard.get<float>("base_kp_y") : 0.5f;
        kd_x_ = blackboard.contains("base_kd_x") ? *blackboard.get<float>("base_kd_x") : 0.0f;
        kd_y_ = blackboard.contains("base_kd_y") ? *blackboard.get<float>("base_kd_y") : 0.0f;
        align_frames_ = blackboard.contains("base_align_frames")
            ? static_cast<int>(*blackboard.get<float>("base_align_frames")) : 10;
        max_misses_ = blackboard.contains("base_max_miss_ticks")
            ? static_cast<int>(*blackboard.get<float>("base_max_miss_ticks")) : 60;

        // Usa a mesma altitude de referência que o H_SEARCH_BASE para não oscilar em z
        if (blackboard.contains("search_base_altitude")) {
            entry_z_ = *blackboard.get<float>("search_base_altitude");
        } else {
            entry_z_ = static_cast<float>(drone_->getLocalPosition().z());
            blackboard.set<float>("search_base_altitude", entry_z_);
        }

        err_x_prev_      = 0.0f;
        err_y_prev_      = 0.0f;
        aligned_counter_ = 0;
        miss_counter_    = 0;
        tick_            = 0;
    }

    std::string act(fsm::Blackboard &blackboard) override {
        if (!drone_) return "ERROR";

        bool is_detected = blackboard.contains("target_base_in_sight")
            && *blackboard.get<bool>("target_base_in_sight");

        if (!is_detected) {
            if (++miss_counter_ > max_misses_) {
                drone_->log("Base lost! Returning to search.");
                return "BASE_LOST";
            }
            // Segura posição na altitude de referência — não se afasta da base
            auto cur = drone_->getLocalPosition();
            drone_->setLocalPosition(
                static_cast<float>(cur.x()),
                static_cast<float>(cur.y()),
                entry_z_,
                static_cast<float>(drone_->getOrientation()[2]));
            return "";
        }

        miss_counter_ = 0;

        float err_x = *blackboard.get<float>("target_base_x_error");
        float err_y = *blackboard.get<float>("target_base_y_error");

        // Controlador PD
        float d_err_x = (err_x - err_x_prev_) / 0.05f;
        float d_err_y = (err_y - err_y_prev_) / 0.05f;
        err_x_prev_ = err_x;
        err_y_prev_ = err_y;

        static constexpr float kMaxV = 0.8f;
        float vx = std::clamp(-(err_y * kp_x_ + d_err_y * kd_x_), -kMaxV, kMaxV);
        float vy = std::clamp( (err_x * kp_y_ + d_err_x * kd_y_), -kMaxV, kMaxV);

        // XY velocidade + Z posição (evita drift de altitude)
        drone_->setMixedSetpoint(vx, vy, entry_z_,
            static_cast<float>(drone_->getOrientation()[2]));

        if (std::abs(err_x) < tolerance_ && std::abs(err_y) < tolerance_) {
            aligned_counter_++;
        } else {
            aligned_counter_ = 0;
        }

        if (tick_++ % 10 == 0)
            drone_->log("Base err=(" + std::to_string(err_x) + "," + std::to_string(err_y)
                + ") aligned=" + std::to_string(aligned_counter_)
                + "/" + std::to_string(align_frames_));

        if (aligned_counter_ >= align_frames_) {
            drone_->log("Aligned with Base!");
            return "ALIGNED";
        }

        return "";
    }

private:
    std::shared_ptr<Drone> drone_;
    float tolerance_;
    float kp_x_, kp_y_;
    float kd_x_, kd_y_;
    float entry_z_;
    float err_x_prev_, err_y_prev_;
    int   aligned_counter_;
    int   align_frames_;
    int   miss_counter_;
    int   max_misses_;
    int   tick_;
};
