#pragma once

#include <cmath>
#include <memory>
#include <string>

#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "drone/movement.hpp"

/**
 * Move para frente lentamente na altitude do RISE até a câmera vertical
 * enxergar a mangueira. Transita para "HOSE_FOUND" ao detectar.
 *
 * Usa move_local_constant_step com waypoint distante (10 m à frente):
 *   - Gera um alvo longe o suficiente para que o passo >> ruído GPS
 *   - PX4 position controller mantém trajetória reta sem drift lateral
 *   - Velocidade limitada pelo step = approach_speed_ × kLookahead (≈1s de antecipação)
 *
 * Blackboard reads:
 *   "hose_in_sight"       (bool)  — detectada pela câmera vertical
 *   "hose_approach_speed" (float) — velocidade frontal em m/s (padrão 0.2)
 *
 * Transitions:
 *   "HOSE_FOUND" — mangueira visível → MANGUEIRA_ALIGN
 */
class ApproachHoseState : public fsm::State {
public:
    ApproachHoseState() : fsm::State(), approach_speed_(0.2f), entry_z_(0.0f), entry_yaw_(0.0f) {}

    void on_enter(fsm::Blackboard &blackboard) override {
        drone_ = *blackboard.get<std::shared_ptr<Drone>>("drone");
        if (!drone_) return;

        drone_->log("");
        drone_->log("STATE: APPROACH_HOSE");

        approach_speed_ = blackboard.contains("hose_approach_speed")
            ? *blackboard.get<float>("hose_approach_speed") : 0.2f;

        auto pos = drone_->getLocalPosition();
        entry_z_   = static_cast<float>(pos.z());
        entry_yaw_ = static_cast<float>(drone_->getOrientation()[2]);

        // Waypoint 10 m à frente na direção atual do drone (FRD frame).
        // getLocalPosition() e getOrientation()[2] usam o mesmo frame FRD,
        // então (cos(yaw_FRD), sin(yaw_FRD)) é a direção "pra frente" em FRD.
        const double yaw_frd = static_cast<double>(entry_yaw_);
        far_target_ = Eigen::Vector3d(
            pos.x() + 10.0 * std::cos(yaw_frd),
            pos.y() + 10.0 * std::sin(yaw_frd),
            static_cast<double>(entry_z_));

        drone_->log("speed=" + std::to_string(approach_speed_) +
                    " entry_z=" + std::to_string(entry_z_) +
                    " yaw_frd=" + std::to_string(entry_yaw_));
    }

    std::string act(fsm::Blackboard &blackboard) override {
        if (!drone_) return "ERROR";

        bool in_sight = blackboard.contains("hose_in_sight") &&
                        *blackboard.get<bool>("hose_in_sight");

        if (in_sight) {
            auto cur = drone_->getLocalPosition();
            drone_->setLocalPosition(
                static_cast<float>(cur.x()),
                static_cast<float>(cur.y()),
                entry_z_,
                entry_yaw_);
            drone_->log("Mangueira found! Transitioning to ALIGN.");
            return "HOSE_FOUND";
        }

        // move_local_constant_step com passo = approach_speed_ × 1s de antecipação.
        // O alvo distante garante que o drone nunca "chega" ao waypoint —
        // o controlador de posição aplica velocidade constante approach_speed_.
        move_local_constant_step(drone_, far_target_, approach_speed_, 100.0f, entry_yaw_);
        return "";
    }

private:
    std::shared_ptr<Drone> drone_;
    float approach_speed_;
    float entry_z_;
    float entry_yaw_;
    Eigen::Vector3d far_target_;
};
