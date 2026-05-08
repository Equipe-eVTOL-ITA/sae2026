#pragma once

#include <cstdlib>
#include <memory>
#include <string>

#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"

class DropTheHookState : public fsm::State {
public:
    DropTheHookState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {
        drone_ = *blackboard.get<std::shared_ptr<Drone>>("drone");
        if (drone_ == nullptr) return;

        drone_->log("");
        drone_->log("STATE: DROP_THE_HOOK");
        
        hook_script_path_ = *blackboard.get<std::string>("hook_script_path");
        executed_ = false;
    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void)blackboard;
        if (drone_ == nullptr) return "ERROR";

        if (!executed_) {
            // Build the command
            std::string command = "python3 " + hook_script_path_;
            drone_->log("Executing command: " + command);
            
            int result = std::system(command.c_str());
            
            if (result == 0) {
                drone_->log("Hook dropped successfully.");
            } else {
                drone_->log("Warning: Hook drop script returned non-zero exit code.");
            }
            executed_ = true;

            // Update target_z so the drone returns at the current altitude
            float current_z = drone_->getLocalPosition().z();
            blackboard.set<float>("target_z", current_z);

            return "HOOK_DROPPED";
        }

        return "";
    }

private:
    std::shared_ptr<Drone> drone_;
    std::string hook_script_path_;
    bool executed_;
};
