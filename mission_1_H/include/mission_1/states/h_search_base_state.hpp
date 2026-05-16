#pragma once

#include <Eigen/Eigen>
#include <array>
#include <cmath>
#include <memory>
#include <string>

#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "drone/movement.hpp"

/*
 * Busca em padrão "H" para localizar a base gabarito.
 *
 * Geometria (computada em on_enter a partir da posição do ArUco):
 *
 *   v_dir  = (cos(yaw), sin(yaw))  — frente do drone no momento do alinhamento
 *   v_perp = (v_dir.y, -v_dir.x) — perpendicular CW (sentido horário)
 *
 *   side1_base = v_perp  * distancia_percorrida_perpendicular
 *   side2_base = -v_perp * distancia_percorrida_perpendicular
 *
 *   waypoints_sideN[i] = sideN_base + v_dir * lambda[i]   (i = 0..3)
 *
 * Fases:
 *   MOVE_TO_SIDE1 → SEARCH_SIDE1 → RETURN_HOME → MOVE_TO_SIDE2 → SEARCH_SIDE2 → DONE
 *
 * Em cada waypoint da busca, o drone paira dwell_ticks_ ticks verificando a
 * detecção da base. Se detectada com persistência suficiente: retorna "BASE_FOUND".
 * Se ambos os lados forem esgotados: retorna "SEARCH_FAILED".
 *
 * Parâmetros blackboard:
 *   distancia_percorrida_perpendicular  — metros laterais (padrão 2.5)
 *   lambda_1..4                         — metros ao longo de v_dir (padrão 0,1.2,-1.2,0)
 *   h_search_dwell_ticks                — ticks por waypoint (padrão 40 = 2s a 20Hz)
 *   base_persistence_frames             — frames consecutivos para confirmar (padrão 3)
 *   search_base_altitude                — altitude NED de busca (padrão: z atual)
 * Nota: v_dir é derivado do yaw do drone em on_enter — não usa aruco_world_x/y.
 */

class HSearchBaseState : public fsm::State {
public:
    HSearchBaseState() : fsm::State(),
        center_z_(0.0f),
        min_base_detections_(3), dwell_ticks_(40),
        base_detection_counter_(0), base_miss_counter_(0),
        phase_(Phase::MOVE_TO_SIDE1), waypoint_idx_(0), dwell_counter_(0) {}

    void on_enter(fsm::Blackboard &blackboard) override {
        drone_ = *blackboard.get<std::shared_ptr<Drone>>("drone");
        if (!drone_) return;

        drone_->log("");
        drone_->log("STATE: H_SEARCH_BASE");

        // Altitude de busca
        if (!blackboard.contains("search_base_altitude"))
            blackboard.set<float>("search_base_altitude",
                static_cast<float>(drone_->getLocalPosition().z()));
        center_z_ = *blackboard.get<float>("search_base_altitude");

        // Parâmetros de detecção
        min_base_detections_ = blackboard.contains("base_persistence_frames")
            ? static_cast<int>(*blackboard.get<float>("base_persistence_frames")) : 3;
        dwell_ticks_ = blackboard.contains("h_search_dwell_ticks")
            ? static_cast<int>(*blackboard.get<float>("h_search_dwell_ticks")) : 40;

        // Parâmetros geométricos
        float dist_perp = blackboard.contains("distancia_percorrida_perpendicular")
            ? *blackboard.get<float>("distancia_percorrida_perpendicular") : 2.5f;

        float lambdas[4] = {
            blackboard.contains("lambda_1") ? *blackboard.get<float>("lambda_1") :  0.0f,
            blackboard.contains("lambda_2") ? *blackboard.get<float>("lambda_2") :  1.2f,
            blackboard.contains("lambda_3") ? *blackboard.get<float>("lambda_3") : -1.2f,
            blackboard.contains("lambda_4") ? *blackboard.get<float>("lambda_4") :  0.0f,
        };

        // Frente do drone: yaw = ângulo do frame FRD para o frame mundo
        float yaw = static_cast<float>(drone_->getOrientation()[2]);
        Eigen::Vector2f v_dir(std::cos(yaw), std::sin(yaw));  // frente do drone (frame mundo)
        Eigen::Vector2f v_perp(v_dir.y(), -v_dir.x());        // perpendicular CW

        // Posições laterais (base dos lados)
        Eigen::Vector2f side1_base = v_perp *  dist_perp;
        Eigen::Vector2f side2_base = v_perp * -dist_perp;

        // 4 waypoints por lado ao longo de v_dir
        for (int i = 0; i < 4; i++) {
            waypoints_side1_[i] = Eigen::Vector3d(
                side1_base.x() + v_dir.x() * lambdas[i],
                side1_base.y() + v_dir.y() * lambdas[i],
                center_z_);
            waypoints_side2_[i] = Eigen::Vector3d(
                side2_base.x() + v_dir.x() * lambdas[i],
                side2_base.y() + v_dir.y() * lambdas[i],
                center_z_);
        }

        side1_pos_ = Eigen::Vector3d(side1_base.x(), side1_base.y(), center_z_);
        side2_pos_ = Eigen::Vector3d(side2_base.x(), side2_base.y(), center_z_);
        home_pos_  = Eigen::Vector3d(0.0, 0.0, center_z_);

        phase_                 = Phase::MOVE_TO_SIDE1;
        waypoint_idx_          = 0;
        dwell_counter_         = 0;
        base_detection_counter_ = 0;
        base_miss_counter_     = 0;

        drone_->log("yaw=" + std::to_string(yaw)
            + " v_dir=(" + std::to_string(v_dir.x()) + "," + std::to_string(v_dir.y()) + ")"
            + " dist_perp=" + std::to_string(dist_perp)
            + " dwell=" + std::to_string(dwell_ticks_) + " ticks");
    }

    std::string act(fsm::Blackboard &blackboard) override {
        if (!drone_) return "ERROR";

        // ── Persistence filter (ativo em todas as fases) ──────────────────────
        bool is_detected = blackboard.contains("target_base_in_sight")
            && *blackboard.get<bool>("target_base_in_sight");

        if (is_detected) {
            base_miss_counter_ = 0;
            if (++base_detection_counter_ >= min_base_detections_) {
                // Congela posição ao confirmar
                auto cur = drone_->getLocalPosition();
                drone_->setLocalPosition(
                    static_cast<float>(cur.x()),
                    static_cast<float>(cur.y()),
                    center_z_,
                    static_cast<float>(drone_->getOrientation()[2]));
                drone_->log("BASE FOUND! (fase=" + phase_name()
                    + " waypoint=" + std::to_string(waypoint_idx_) + ")");
                return "BASE_FOUND";
            }
        } else {
            if (++base_miss_counter_ >= kBaseMissTol)
                base_detection_counter_ = 0;
        }

        // ── Phase dispatch ────────────────────────────────────────────────────
        switch (phase_) {

        case Phase::MOVE_TO_SIDE1:
            if (move_local_by_waypoint(drone_, side1_pos_, 0.5f, 0.20f)) {
                drone_->log("Lado 1 alcançado. Iniciando busca em waypoints.");
                phase_                 = Phase::SEARCH_SIDE1;
                waypoint_idx_          = 0;
                dwell_counter_         = 0;
                base_detection_counter_ = 0;
            }
            break;

        case Phase::SEARCH_SIDE1:
            if (search_step(waypoints_side1_)) {
                drone_->log("Lado 1 esgotado. Retornando à origem.");
                phase_                 = Phase::RETURN_HOME;
                base_detection_counter_ = 0;
            }
            break;

        case Phase::RETURN_HOME:
            if (move_local_by_waypoint(drone_, home_pos_, 0.5f, 0.20f)) {
                drone_->log("Retornou à origem. Movendo para lado 2.");
                phase_                 = Phase::MOVE_TO_SIDE2;
                base_detection_counter_ = 0;
            }
            break;

        case Phase::MOVE_TO_SIDE2:
            if (move_local_by_waypoint(drone_, side2_pos_, 0.5f, 0.20f)) {
                drone_->log("Lado 2 alcançado. Iniciando busca em waypoints.");
                phase_                 = Phase::SEARCH_SIDE2;
                waypoint_idx_          = 0;
                dwell_counter_         = 0;
                base_detection_counter_ = 0;
            }
            break;

        case Phase::SEARCH_SIDE2:
            if (search_step(waypoints_side2_)) {
                drone_->log("Lado 2 esgotado. Base não encontrada.");
                phase_ = Phase::DONE;
            }
            break;

        case Phase::DONE:
            return "SEARCH_FAILED";
        }

        return "";
    }

private:
    // Avança waypoints com dwell. Retorna true quando todos os 4 foram varridos.
    bool search_step(const std::array<Eigen::Vector3d, 4>& waypoints) {
        Eigen::Vector3d target = waypoints[waypoint_idx_];

        bool at_wp = move_local_by_waypoint(drone_, target, 0.4f, 0.20f);
        if (at_wp) {
            if (dwell_counter_ % 20 == 0) {
                drone_->log("H-search waypoint " + std::to_string(waypoint_idx_)
                    + "/4 dwell=" + std::to_string(dwell_counter_)
                    + "/" + std::to_string(dwell_ticks_)
                    + " fase=" + phase_name());
            }
            if (++dwell_counter_ >= dwell_ticks_) {
                dwell_counter_ = 0;
                base_detection_counter_ = 0;
                if (++waypoint_idx_ >= 4) {
                    waypoint_idx_ = 0;
                    return true;  // todos os waypoints varridos
                }
            }
        }
        return false;
    }

    std::string phase_name() const {
        switch (phase_) {
            case Phase::MOVE_TO_SIDE1: return "MOVE_TO_SIDE1";
            case Phase::SEARCH_SIDE1:  return "SEARCH_SIDE1";
            case Phase::RETURN_HOME:   return "RETURN_HOME";
            case Phase::MOVE_TO_SIDE2: return "MOVE_TO_SIDE2";
            case Phase::SEARCH_SIDE2:  return "SEARCH_SIDE2";
            case Phase::DONE:          return "DONE";
        }
        return "?";
    }

    enum class Phase { MOVE_TO_SIDE1, SEARCH_SIDE1, RETURN_HOME, MOVE_TO_SIDE2, SEARCH_SIDE2, DONE };

    std::shared_ptr<Drone> drone_;

    float center_z_;
    int   min_base_detections_;
    int   dwell_ticks_;

    int   base_detection_counter_;
    int   base_miss_counter_;
    static constexpr int kBaseMissTol = 3;

    Phase phase_;
    int   waypoint_idx_;
    int   dwell_counter_;

    std::array<Eigen::Vector3d, 4> waypoints_side1_;
    std::array<Eigen::Vector3d, 4> waypoints_side2_;
    Eigen::Vector3d side1_pos_;
    Eigen::Vector3d side2_pos_;
    Eigen::Vector3d home_pos_;
};
