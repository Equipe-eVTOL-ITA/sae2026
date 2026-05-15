#pragma once

#include <Eigen/Eigen>
#include <cmath>
#include <memory>
#include <string>

#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "drone/movement.hpp"

/**
 * Retorna à base inicial com perfil de velocidade trapezoidal, depois paira
 * por `hover_before_landing_ticks` ticks antes de transitar para ARRIVED.
 *
 * Parâmetros blackboard:
 *   target_x/y/z/yaw             — destino
 *   position_tolerance           — tolerância de chegada (m)
 *   max_vel_goto                  — velocidade de cruceiro (m/s, padrão 0.5)
 *   goto_acceleration             — aceleração/desaceleração (m/s², padrão 0.5)
 *   hover_before_landing_ticks   — ticks de hover antes de pousar (padrão 40)
 */
class GoToBaseState : public fsm::State {
public:
    GoToBaseState() : fsm::State(),
        target_x_(0.0f), target_y_(0.0f), target_z_(-1.5f), target_yaw_(0.0f),
        tolerance_(0.15f), max_vel_(0.5f), a_max_(0.5f),
        hover_ticks_(40), hover_counter_(0) {}

    void on_enter(fsm::Blackboard &blackboard) override {
        drone_ = *blackboard.get<std::shared_ptr<Drone>>("drone");
        if (!drone_) return;

        drone_->log("");
        drone_->log("STATE: GOTO_BASE");

        target_x_  = blackboard.contains("target_x")   ? *blackboard.get<float>("target_x")  : 0.0f;
        target_y_  = blackboard.contains("target_y")   ? *blackboard.get<float>("target_y")  : 0.0f;
        target_z_  = blackboard.contains("target_z")   ? *blackboard.get<float>("target_z")  : -1.5f;
        target_yaw_= blackboard.contains("target_yaw") ? *blackboard.get<float>("target_yaw"): 0.0f;
        tolerance_ = blackboard.contains("position_tolerance")
            ? *blackboard.get<float>("position_tolerance") : 0.15f;
        max_vel_   = blackboard.contains("max_vel_goto")
            ? *blackboard.get<float>("max_vel_goto") : 0.5f;
        a_max_     = blackboard.contains("goto_acceleration")
            ? *blackboard.get<float>("goto_acceleration") : 0.5f;
        hover_ticks_ = blackboard.contains("hover_before_landing_ticks")
            ? static_cast<int>(*blackboard.get<float>("hover_before_landing_ticks")) : 40;

        hover_counter_ = 0;
        mover_.reset();

        drone_->log("Returning to (" + std::to_string(target_x_) + "," +
                    std::to_string(target_y_) + "," + std::to_string(target_z_) + ")" +
                    " v_max=" + std::to_string(max_vel_) +
                    " a_max=" + std::to_string(a_max_) +
                    " hover=" + std::to_string(hover_ticks_) + " ticks");
    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void)blackboard;
        if (!drone_) return "ERROR";

        Eigen::Vector3d tgt(target_x_, target_y_, target_z_);
        bool at_target = mover_.update(drone_, tgt, max_vel_, a_max_, tolerance_, target_yaw_);

        if (at_target) {
            if (++hover_counter_ >= hover_ticks_) {
                drone_->log("Hover complete — transitioning to LANDING");
                return "ARRIVED";
            }
            if (hover_counter_ % 20 == 0)
                drone_->log("Hovering before landing (" +
                            std::to_string(hover_counter_) + "/" +
                            std::to_string(hover_ticks_) + " ticks)");
        } else {
            hover_counter_ = 0;
        }

        return "";
    }

private:
    std::shared_ptr<Drone> drone_;
    float target_x_, target_y_, target_z_, target_yaw_;
    float tolerance_, max_vel_, a_max_;
    int   hover_ticks_, hover_counter_;
    TrapezoidalMover mover_;
};
