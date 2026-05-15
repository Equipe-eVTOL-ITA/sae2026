#pragma once

#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "drone/PidController.hpp"

/*
 * Parâmetros esperados (blackboard):
 *   base_tolerance        — erro máximo em X e Y em METROS para confirmar alinhamento (padrão 0.10)
 *   base_kp_x, base_kp_y — ganhos proporcionais dos PIDs  [m/s por metro]
 *   base_kd_x, base_kd_y — ganhos derivativos dos PIDs    [m/s por (m/s)]
 *   base_max_velocity     — velocidade máxima de alinhamento m/s (padrão 0.5)
 *   cam_scale             — tan(FOV/2) da câmera; converte erro norm. → metros (padrão 0.7 ≈ FOV 70°)
 *   align_frames          — frames consecutivos dentro da tolerância para confirmar (padrão 10)
 *   align_timeout         — timeout global em segundos (padrão 30.0)
 *
 * Conversão pixel normalizado → metros:
 *   err_m = err_norm * altitude * cam_scale
 *   (altitude = -entry_z_, pois NED z é negativo acima do solo)
 *
 * Blackboard lido em act():
 *   target_base_in_sight  — bool
 *   target_base_x_error   — float [-1, 1]  (saída direta do detector, em pixels normalizados)
 *   target_base_y_error   — float [-1, 1]
 *
 * Transições:
 *   "ALIGNED"   — alinhamento confirmado
 *   "BASE_LOST" — base perdida por mais de max_misses_ ticks consecutivos
 *   "TIMEOUT"   — timeout global atingido sem convergir
 */

class AlignState : public fsm::State {
public:
    AlignState() : fsm::State(),
        tolerance_(0.10f), entry_z_(0.0f), alt_(1.0f),
        cam_scale_(0.7f), max_v_(0.5f),
        align_frames_(10), timeout_s_(30.0f),
        aligned_counter_(0), miss_counter_(0), tick_(0),
        pid_x_(0.5f, 0.0f, 0.0f, 0.0f),
        pid_y_(0.5f, 0.0f, 0.0f, 0.0f) {}

    void on_enter(fsm::Blackboard &blackboard) override {
        drone_ = *blackboard.get<std::shared_ptr<Drone>>("drone");
        if (drone_ == nullptr) return;

        drone_->log("");
        drone_->log("STATE: ALIGN");

        tolerance_ = blackboard.contains("base_tolerance")
            ? *blackboard.get<float>("base_tolerance") : 0.10f;
        cam_scale_ = blackboard.contains("cam_scale")
            ? *blackboard.get<float>("cam_scale") : 0.7f;
        max_v_ = blackboard.contains("base_max_velocity")
            ? *blackboard.get<float>("base_max_velocity") : 0.5f;
        align_frames_ = blackboard.contains("align_frames")
            ? static_cast<int>(*blackboard.get<float>("align_frames")) : 10;
        timeout_s_ = blackboard.contains("align_timeout")
            ? *blackboard.get<float>("align_timeout") : 30.0f;

        float kp_x = blackboard.contains("base_kp_x") ? *blackboard.get<float>("base_kp_x") : 0.5f;
        float kp_y = blackboard.contains("base_kp_y") ? *blackboard.get<float>("base_kp_y") : 0.5f;
        float kd_x = blackboard.contains("base_kd_x") ? *blackboard.get<float>("base_kd_x") : 0.0f;
        float kd_y = blackboard.contains("base_kd_y") ? *blackboard.get<float>("base_kd_y") : 0.0f;

        pid_x_ = PidController(kp_x, 0.0f, kd_x, 0.0f);
        pid_y_ = PidController(kp_y, 0.0f, kd_y, 0.0f);

        entry_z_         = static_cast<float>(drone_->getLocalPosition().z());
        alt_             = std::max(0.3f, -entry_z_);  // altitude positiva em metros
        aligned_counter_ = 0;
        miss_counter_    = 0;
        tick_            = 0;
        start_time_      = std::chrono::steady_clock::now();

        drone_->log("alt=" + std::to_string(alt_) + "m cam_scale=" + std::to_string(cam_scale_)
            + " tol=" + std::to_string(tolerance_) + "m kp=(" + std::to_string(kp_x)
            + "," + std::to_string(kp_y) + ")");
    }

    std::string act(fsm::Blackboard &blackboard) override {
        if (drone_ == nullptr) return "ERROR";

        auto elapsed = std::chrono::duration<float>(
            std::chrono::steady_clock::now() - start_time_).count();
        if (elapsed > timeout_s_) {
            drone_->log("ALIGN: timeout");
            return "TIMEOUT";
        }

        bool is_detected = blackboard.contains("target_base_in_sight")
            && *blackboard.get<bool>("target_base_in_sight");

        if (!is_detected) {
            if (++miss_counter_ > max_misses_) {
                drone_->log("BASE LOST!!");
                //return "BASE_LOST";
            }
            auto cur = drone_->getLocalPosition();
            drone_->setLocalPosition(
                static_cast<float>(cur.x()),
                static_cast<float>(cur.y()),
                entry_z_,
                static_cast<float>(drone_->getOrientation()[2]));
            return "";
        }

        miss_counter_ = 0;

        // Converte erro normalizado [-1,1] para metros usando altitude e FOV da câmera
        float raw_x = *blackboard.get<float>("target_base_x_error");
        float raw_y = *blackboard.get<float>("target_base_y_error");

        float err_x = raw_x * alt_ * cam_scale_;  // metros (lateral)
        float err_y = raw_y * alt_ * cam_scale_;  // metros (frontal)

        // pid_x_ usa err_y (eixo Y da câmera → eixo X do drone)
        // pid_y_ usa -err_x (sinal invertido: alvo à direita → vy positivo)
        float vx = std::clamp(pid_x_.compute(err_y),  -max_v_, max_v_);
        float vy = std::clamp(pid_y_.compute(-err_x), -max_v_, max_v_);

        drone_->setMixedSetpoint(vx, vy, entry_z_,
            static_cast<float>(drone_->getOrientation()[2]));

        if (std::abs(err_x) < tolerance_ && std::abs(err_y) < tolerance_) {
            aligned_counter_++;
        } else {
            aligned_counter_ = 0;
        }

        if (tick_++ % 10 == 0)
            drone_->log("Base err=(" + std::to_string(err_x) + "," + std::to_string(err_y)
                + ")m aligned=" + std::to_string(aligned_counter_)
                + "/" + std::to_string(align_frames_));

        if (aligned_counter_ >= align_frames_) {
            drone_->log("ALIGNED!");
            return "ALIGNED";
        }

        return "";
    }

private:
    std::shared_ptr<Drone> drone_;

    float tolerance_;   // metros
    float entry_z_;     // NED z na entrada do estado
    float alt_;         // altitude positiva = -entry_z_ (metros)
    float cam_scale_;   // tan(FOV/2)
    float max_v_;       // m/s
    int   align_frames_;
    float timeout_s_;

    int   aligned_counter_;
    int   miss_counter_;
    int   tick_;

    static constexpr int max_misses_ = 30;

    PidController pid_x_;
    PidController pid_y_;

    std::chrono::steady_clock::time_point start_time_;
};
