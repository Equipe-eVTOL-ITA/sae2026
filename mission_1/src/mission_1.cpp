#include <memory>
#include <iostream>
#include <map>
#include <string>
#include <variant>

#include <rclcpp/rclcpp.hpp>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"

// Standard states from stdstates
#include "stdstates/arming_state.hpp"
#include "stdstates/takeoff_state.hpp"
#include "stdstates/landing_state.hpp"

// Mission-specific states 
// ...


/**
 * @brief Mission 1 FSM — defines states and transitions.
 *
 * This class sets up the finite state machine for Mission 1.
 */
class Mission1FSM : public fsm::FSM {
public:
    Mission1FSM(
        std::shared_ptr<Drone> drone,
        const std::map<std::string, std::variant<double, std::string>>& params
    ) : fsm::FSM({"ERROR", "FINISHED"}) {

        // Store drone in the blackboard (accessible by all states)
        this->blackboard_set<std::shared_ptr<Drone>>("drone", drone);

        // Load ROS2 parameters into the blackboard
        for (const auto& [key, value] : params) {
            if (std::holds_alternative<double>(value)) {
                this->blackboard_set<float>(key, static_cast<float>(std::get<double>(value)));
            } else if (std::holds_alternative<std::string>(value)) {
                this->blackboard_set<std::string>(key, std::get<std::string>(value));
            }
        }

        // ===================== STATES =====================
        this->add_state("ARMING", std::make_unique<ArmingState>());
        this->add_state("TAKEOFF", std::make_unique<TakeoffState>());
        this->add_state("LANDING", std::make_unique<LandingState>());

        // ================== TRANSITIONS ===================
        // Define transitions: {outcome, next_state}
        this->add_transitions("ARMING", {
            {"ARMED", "TAKEOFF"},
            {"ERROR", "ERROR"}
        });

        this->add_transitions("TAKEOFF", {
            {"TAKEOFF COMPLETED", "LANDING"},
            {"ERROR", "ERROR"}
        });

        this->add_transitions("LANDING", {
            {"LANDED", "FINISHED"},
            {"ERROR", "ERROR"}
        });

        this->set_initial_state("TAKEOFF");
    }
};


/**
 * @brief ROS2 Node that runs the Mission 1 FSM.
 *
 * Declares ROS2 parameters (loaded from config/simulation.yaml),
 * passes them to the FSM via the blackboard, and executes the FSM
 * at a fixed rate (50ms / 20Hz).
 */
class Mission1Node : public rclcpp::Node {
public:
    Mission1Node(std::shared_ptr<Drone> drone)
        : rclcpp::Node("mission_1_node"), drone_(drone) {

        // Declare default parameters — these are overridden by config YAML files
        // when launching via: ros2 launch mission_1 simulation.launch.py
        std::map<std::string, std::variant<double, std::string>> default_params = {
            // Takeoff
            {"takeoff_height",         -2.5},
            {"max_vertical_velocity",  1.2},
            {"position_tolerance",     0.15},

            // Landing
            {"landing_velocity_max",   0.5},
            {"landing_velocity_min",   0.15},
            {"max_base_height",        0.5},
            {"landing_timeout",        5.0},

            // Movement
            {"max_horizontal_velocity", 1.5},
        };

        auto params = declareAndGetParameters(default_params);

        // Create the FSM
        fsm_ = std::make_unique<Mission1FSM>(drone_, params);

        // Run FSM at 20Hz (50ms period)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&Mission1Node::executeFSM, this)
        );

        RCLCPP_INFO(this->get_logger(), "Mission 1 FSM started");
    }

private:
    void executeFSM() {
        if (rclcpp::ok() && !fsm_->is_finished()) {
            fsm_->execute();
        } else {
            RCLCPP_INFO(this->get_logger(), "FSM finished with outcome: %s",
                        fsm_->get_fsm_outcome().c_str());
            rclcpp::shutdown();
        }
    }

    /**
     * @brief Declares ROS2 parameters with defaults and reads their values.
     * Parameters can be overridden via YAML config files at launch time.
     */
    std::map<std::string, std::variant<double, std::string>> declareAndGetParameters(
        const std::map<std::string, std::variant<double, std::string>>& defaults) {

        std::map<std::string, std::variant<double, std::string>> result;

        for (const auto& [name, default_value] : defaults) {
            if (std::holds_alternative<double>(default_value)) {
                this->declare_parameter(name, std::get<double>(default_value));
                result[name] = this->get_parameter(name).as_double();
            } else if (std::holds_alternative<std::string>(default_value)) {
                this->declare_parameter(name, std::get<std::string>(default_value));
                result[name] = this->get_parameter(name).as_string();
            }
        }

        return result;
    }

    std::shared_ptr<Drone> drone_;
    std::unique_ptr<Mission1FSM> fsm_;
    rclcpp::TimerBase::SharedPtr timer_;
};


/**
 * @brief Main entry point.
 *
 * Creates the Drone node and Mission FSM node, then spins them
 * in a multi-threaded executor.
 */
int main(int argc, const char *argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::executors::MultiThreadedExecutor executor;

    auto drone = std::make_shared<Drone>();
    auto mission_node = std::make_shared<Mission1Node>(drone);

    executor.add_node(drone);
    executor.add_node(mission_node);

    executor.spin();

    rclcpp::shutdown();
    return 0;
}
