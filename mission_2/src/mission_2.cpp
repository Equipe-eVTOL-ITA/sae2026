#include <memory>
#include <iostream>
#include <map>
#include <string>
#include <variant>

#include <rclcpp/rclcpp.hpp>
#include <custom_msgs/msg/ball_detection.hpp>

#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"

// Standard states from stdstates
#include "stdstates/arming_state.hpp"
#include "stdstates/takeoff_state.hpp"
#include "stdstates/landing_state.hpp"
#include "stdstates/goto_state.hpp"
#include "stdstates/align_state.hpp"

// Mission-specific states 
#include "mission_2/states/search_ball_state.hpp"
#include "mission_2/states/goto_ball_state.hpp"
#include "mission_2/states/rise_state.hpp"
#include "mission_2/states/drop_the_hook_state.hpp"

class Mission2FSM : public fsm::FSM {
public:
    Mission2FSM(
        std::shared_ptr<Drone> drone,
        const std::map<std::string, std::variant<double, std::string>>& params
    ) : fsm::FSM({"ERROR", "FINISHED"}) {

        // objeto drone na blackboard
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
        this->add_state("SEARCH_BALL", std::make_unique<SearchBallState>());
        this->add_state("GOTO_BALL", std::make_unique<GoToBallState>());
        this->add_state("RISE", std::make_unique<RiseState>());
        this->add_state("ALIGN", std::make_unique<AlignState>(true, true)); // Alinha centro e yaw
        this->add_state("DROP_HOOK", std::make_unique<DropTheHookState>());
        this->add_state("GOTO_BASE", std::make_unique<GoToState>());
        this->add_state("LANDING", std::make_unique<LandingState>());

        // ================== TRANSITIONS ===================
        // Define transitions: {outcome, next_state}
        this->add_transitions("ARMING", {
            {"ARMED", "TAKEOFF"},
            {"ERROR", "ERROR"}
        });

        this->add_transitions("TAKEOFF", {
            {"TAKEOFF COMPLETED", "SEARCH_BALL"},
            {"ERROR", "ERROR"}
        });

        this->add_transitions("SEARCH_BALL", {
            {"BALL_FOUND", "GOTO_BALL"},
            {"ERROR", "ERROR"}
        });

        this->add_transitions("GOTO_BALL", {
            {"REACHED", "RISE"},
            {"BALL_LOST", "SEARCH_BALL"},
            {"ERROR", "ERROR"}
        });

        this->add_transitions("RISE", {
            {"RISE_COMPLETED", "ALIGN"},
            {"ERROR", "ERROR"}
        });

        this->add_transitions("ALIGN", {
            {"ALIGNED", "DROP_HOOK"},
            {"ERROR", "ERROR"}
        });

        this->add_transitions("DROP_HOOK", {
            {"HOOK_DROPPED", "GOTO_BASE"},
            {"ERROR", "ERROR"}
        });

        this->add_transitions("GOTO_BASE", {
            {"ARRIVED", "LANDING"},
            {"ERROR", "ERROR"}
        });

        this->add_transitions("LANDING", {
            {"LANDED", "FINISHED"},
            {"ERROR", "ERROR"}
        });

        this->set_initial_state("TAKEOFF");
    }
};

class Mission2Node : public rclcpp::Node {
public:
    Mission2Node(std::shared_ptr<Drone> drone)
        : rclcpp::Node("mission_2_node"), drone_(drone) {

        // Declare default parameters
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

            // Mission 2 Parameters
            {"search_yaw_rate",        0.5},
            {"search_radius",          3.0},
            {"ball_trigger_score",     5000.0},
            {"ball_kp_x", 0.5}, {"ball_ki_x", 0.0}, {"ball_kd_x", 0.1},
            {"ball_kp_y", 0.5}, {"ball_ki_y", 0.0}, {"ball_kd_y", 0.1},
            {"rise_delta_z", 2.0},
            {"align_tolerance_y",      0.1},
            {"align_tolerance_yaw",    0.05},
            {"align_kp_y", 0.5}, {"align_ki_y", 0.0}, {"align_kd_y", 0.1},
            {"align_kp_yaw", 0.5}, {"align_ki_yaw", 0.0}, {"align_kd_yaw", 0.1},
            {"hook_script_path",       std::string("/home/marconipavan/evtol/dev/scripts/drop_hook.py")},
            {"target_x", 0.0},
            {"target_y", 0.0},
            {"target_z", -2.0}, // Flight altitude to return
            {"target_yaw", 0.0},
        };

        auto params = declareAndGetParameters(default_params);

        // objeto da FSM
        fsm_ = std::make_unique<Mission2FSM>(drone_, params);

        // subscriber para as mensagens de detecção da bola
        ball_sub_ = this->create_subscription<custom_msgs::msg::BallDetection>(
            "ball_detection", 10,
            [this](const custom_msgs::msg::BallDetection::SharedPtr msg) {
                fsm_->blackboard_set<bool>("ball_is_detected", msg->is_detected);
                fsm_->blackboard_set<float>("ball_target_score", msg->target_score);
                fsm_->blackboard_set<float>("ball_x_error", msg->x_error);
                fsm_->blackboard_set<float>("ball_y_error", msg->y_error);
            }
        );

        // rodar a FSM a 20Hz (periodo de 50ms)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&Mission2Node::executeFSM, this)
        );

        RCLCPP_INFO(this->get_logger(), "Mission 2 FSM started");
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
    std::unique_ptr<Mission2FSM> fsm_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<custom_msgs::msg::BallDetection>::SharedPtr ball_sub_;
};


int main(int argc, const char *argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::executors::MultiThreadedExecutor executor;

    auto drone = std::make_shared<Drone>();
    auto mission_node = std::make_shared<Mission2Node>(drone);

    executor.add_node(drone);
    executor.add_node(mission_node);

    executor.spin();

    rclcpp::shutdown();
    return 0;
}
