#pragma once

#include <Eigen/Eigen>
#include <cstdint>
#include <memory>
#include <string>
#include <map>

#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"

class PhotoState : public fsm::State{
public:
    PhotoState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {
        drone_ = *blackboard.get<std::shared_ptr<Drone>>("drone");
        if (drone_==nullptr) return;
        
        position_photo_= drone_->getLocalPosition();
        photo_yaw_ = drone_->getOrientation()[2];
        limit_value_ = *blackboard.get<float>("limit_value");
        publish_analysis_=*blackboard.get<std::function<void(std::string)>>("publish_pressure_analysis");
        measured_pressure_ = *blackboard.get<float>("measured_pressure");

        drone_->log("");
        drone_->log("STATE: PHOTO");
    }
    std::string act(fsm::Blackboard &blackboard) override {
        (void)blackboard;

        if (drone_ == nullptr) return "ERROR";
        
        
        drone_->setLocalPosition(position_photo_.x(),position_photo_.y(),position_photo_.y(),photo_yaw_);

        measured_pressure_ = *blackboard.get<float>("measured_pressure");
        
        if(measured_pressure_==-1){
            return "";
        }
        else{
            if(measured_pressure_>limit_value_){
                publish_analysis_("pressure above limit");
            }
            else{
                publish_analysis_("pressure below limit");
            }

            blackboard.set("measured_pressure",-1.0);

            return "PHOTO TAKEN";
        }
        

    }
    
private:
std::shared_ptr<Drone> drone_;
bool has_pressure_;
float measured_pressure_;
Eigen::Vector3d position_photo_;
float photo_yaw_;
float limit_value_;
std::function<void(std::string)> publish_analysis_;

};


