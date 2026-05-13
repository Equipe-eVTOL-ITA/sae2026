#pragma once

#include <Eigen/Eigen>
#include <cstdint>
#include <memory>
#include <string>
#include <map>

#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"

class SearchState : public fsm::State {
public:
    SearchState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {
        drone_ = *blackboard.get<std::shared_ptr<Drone>>("drone");
        if (drone_ == nullptr) return;

        drone_->log("");
        drone_->log("STATE: SEARCH");

        search_step_ = *blackboard.get<float>("step");

        max_horizontal_velocity_search_ = *blackboard.get<float>("max_horizontal_velocity_search");
        position_tolerance_search_ = *blackboard.get<float>("position_tolerance_search");
        contador_ = 0;
        limite_ticks_ = *blackboard.get<int>("limite_ticks_");
        ticks_=0;

        pos_ = drone_->getLocalPosition();
        initial_yaw_ = drone_->getOrientation()[2];

        search_waypoints_[0] = Eigen::Vector3d(pos_.x() + search_step_,pos_.y()+search_step_,pos_.z());
        search_waypoints_[1] = Eigen::Vector3d(pos_.x() - search_step_,pos_.y()+search_step_,pos_.z());
        search_waypoints_[2] = Eigen::Vector3d(pos_.x() - search_step_,pos_.y()-search_step_,pos_.z());
        search_waypoints_[3] = Eigen::Vector3d(pos_.x() + search_step_,pos_.y()-search_step_,pos_.z());
        search_waypoints_[4] = Eigen::Vector3d(pos_.x() + search_step_,pos_.y()+search_step_,pos_.z());
        search_waypoints_[5] = pos_;

        if (drone_->getArmingState() != DronePX4::ARMING_STATE::ARMED) {
            drone_->toOffboardSync();
            drone_->armSync();
        }

        drone_->log("Initial Yaw: " + std::to_string(initial_yaw_));
    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void)blackboard;

        if (!drone_) return "ERROR";

            
        float error_x = *blackboard.get<float>("error_x");
        float error_y = *blackboard.get<float>("error_y");

        if(contador_>=6){
            contador_ = 0;
        }

        if((!std::isnan(error_x)) && (!std::isnan(error_y))){
            ticks_++;
            if(ticks_ >= limite_ticks_){
                return "FOUND IT";
            }
            pos_=drone_->getLocalPosition();
            drone_->setLocalPosition(pos_.x(),pos_.y(),pos_.z(),initial_yaw_);
            return "";
        }
        else{
            ticks_ = 0;
        }

        pos_ = drone_->getLocalPosition();
        goal_ = search_waypoints_[contador_];

        Eigen::Vector3d diff = goal_ - pos_;

        if (diff.norm() < position_tolerance_search_) {
            drone_->log("contador ++");
            contador_++;
            return "";
        }

        // Move toward goal with velocity clamping
        Eigen::Vector3d little_goal = pos_ + (diff.norm() > max_horizontal_velocity_search_
                                            ? diff.normalized() * max_horizontal_velocity_search_
                                            : diff);

        drone_->setLocalPosition(
            little_goal.x(),
            little_goal.y(),
            little_goal.z(),
            initial_yaw_);

        return "";
        
    }
    void on_exit(fsm::Blackboard &blackboard) override {
        (void)blackboard;
    }

private:
    std::array<Eigen::Vector3d, 6> search_waypoints_;
    std::shared_ptr<Drone> drone_;
    Eigen::Vector3d pos_, goal_;
    float initial_yaw_;
    float position_tolerance_search_;
    float max_horizontal_velocity_search_;
    float search_step_;
    int contador_;
    int limite_ticks_;
    int ticks_;
};