#pragma once

#include <Eigen/Eigen>
#include <cstdint>
#include <memory>
#include <string>
#include <map>

#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"

class GoToTesteState : public fsm::State {
public:
    GoToTesteState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {
        drone_ = *blackboard.get<std::shared_ptr<Drone>>("drone");
        if (drone_ == nullptr) return;

        drone_->log("");
        drone_->log("STATE: GOTOTESTE");   

        max_horizontal_velocity_ = *blackboard.get<float>("max_horizontal_velocity");
        position_tolerance_mov_ = *blackboard.get<float>("position_tolerance_mov");

        // definir coordenadas
        x = *blackboard.get<float>("x");
        y = *blackboard.get<float>("y");
        z = *blackboard.get<float>("z");

        

        pos_ = drone_->getLocalPosition();
        initial_yaw_ = drone_->getOrientation()[2];
        goal_ = Eigen::Vector3d(x,y,z);

        if (drone_->getArmingState() != DronePX4::ARMING_STATE::ARMED) {
            drone_->toOffboardSync();
            drone_->armSync();
        }

        drone_->log("Initial Yaw: " + std::to_string(initial_yaw_));
    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void)blackboard;

        if (drone_ == nullptr) return "ERROR";
        else{

            // Re-arm if the drone somehow disarmed
            if (drone_->getArmingState() != DronePX4::ARMING_STATE::ARMED) {
                drone_->log("Drone is not armed, arming again.");
                drone_->toOffboardSync();
                drone_->armSync();
            }

            pos_ = drone_->getLocalPosition();
            Eigen::Vector3d diff = goal_ - pos_;

            if (diff.norm() < position_tolerance_mov_) {
                drone_->log("target point reached");
                return "ON TARGET";
            }
            // Move toward goal with velocity clamping
            Eigen::Vector3d little_goal = pos_ + (diff.norm() > max_horizontal_velocity_
                                                ? diff.normalized() * max_horizontal_velocity_
                                                : diff);

            drone_->setLocalPosition(
                little_goal.x(),
                little_goal.y(),
                little_goal.z(),
                initial_yaw_);

            return "";
        }
    }
    void on_exit(fsm::Blackboard &blackboard) override {
        (void)blackboard;
    }

private:
    std::shared_ptr<Drone> drone_;
    Eigen::Vector3d pos_, goal_;
    float initial_yaw_;
    float position_tolerance_mov_;
    float max_horizontal_velocity_;
    float x,y,z;

};