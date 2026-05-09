#pragma once

#include <memory>  
#include <map>      
#include <variant>  
#include <string>
#include <Eigen/Eigen>
#include <cstdint>
  
#include "fsm/fsm.hpp" 
#include "drone/Drone.hpp"          

class AlignState : public fsm::State {
public:
    AlignState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {
        drone_ = *blackboard.get<std::shared_ptr<Drone>>("drone");
        if (drone_ == nullptr) return;

        drone_->log("");
        drone_->log("STATE: ALIGN");

        max_horizontal_velocity_align_ =*blackboard.get<float>("max_horizontal_velocity_align");
        position_tolerance_align_ = *blackboard.get<float>("position_tolerance_align");

        pos_a_ = drone_->getLocalPosition();
        initial_yaw_ = drone_->getOrientation()[2];
        manometer_h_ = -1.7;
        fx_=1554.95;
        fy_=1545.72;

        if (drone_->getArmingState() != DronePX4::ARMING_STATE::ARMED) {
            drone_->toOffboardSync();
            drone_->armSync();
        }

        drone_->log("Initial Yaw: " + std::to_string(initial_yaw_));
    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void)blackboard;

        if (drone_ == nullptr) return "ERROR";
       
        // Re-arm if the drone somehow disarmed
        
        if (drone_->getArmingState() != DronePX4::ARMING_STATE::ARMED) {
            drone_->log("Drone is not armed, arming again.");
            drone_->toOffboardSync();
            drone_->armSync();
        }

        pos_a_ = drone_->getLocalPosition();
        
        error_x_= *blackboard.get<float>("error_x");
        error_y_= *blackboard.get<float>("error_y");

        erro_x_camera_ = error_x_*std::abs(pos_a_.z()-manometer_h_)/fx_;
        erro_y_camera_ = error_y_*std::abs(pos_a_.z()-manometer_h_)/fy_;


        float current_yaw = drone_->getOrientation()[2];
        float erro_mundo_x = -(erro_x_camera_ * sin(current_yaw)) - (erro_y_camera_ * cos(current_yaw));
        float erro_mundo_y = (erro_x_camera_ * cos(current_yaw)) - (erro_y_camera_ * sin(current_yaw));


        Eigen::Vector3d diff (erro_mundo_x,erro_mundo_y,0.0);

        if (diff.norm() < position_tolerance_align_) {
            drone_->log("manometro alinhado");
            return "ALIGNED";
        }
        // Move toward goal with velocity clamping
        Eigen::Vector3d little_goal = pos_a_ + (diff.norm() > max_horizontal_velocity_align_
                                              ? diff.normalized() * max_horizontal_velocity_align_
                                              : diff);

        drone_->setLocalPosition(
            little_goal.x(),
            little_goal.y(),
            little_goal.z(),
            initial_yaw_);

        return "";
    }

private:
    std::shared_ptr<Drone> drone_;
    Eigen::Vector3d pos_a_, goal_a_, erro_;
    float initial_yaw_;
    float position_tolerance_align_;
    float max_horizontal_velocity_align_;
    float error_x_;
    float error_y_;
    float manometer_h_;
    float fx_,fy_;
    float erro_x_camera_,erro_y_camera_;
};