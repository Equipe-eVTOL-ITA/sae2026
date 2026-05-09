#include <memory>
#include <iostream>
#include <map>
#include <string>
#include <variant>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <custom_msgs/msg/base_detection.hpp>
#include <geometry_msgs/msg/point.hpp>

// Standard states from stdstates
#include "stdstates/arming_state.hpp"
#include "stdstates/takeoff_state.hpp"
#include "stdstates/landing_state.hpp"
#include "mission_3/align_state.hpp"
#include "mission_3/goto_state.hpp"
#include "mission_3/photo_state.hpp"



//defines the states and transitions
class Fase3FSM : public fsm::FSM {
public:
    Fase3FSM(std::shared_ptr<Drone> drone,
        const std::map<std::string, std::variant<double, std::string>> &params
    ) : fsm::FSM({"ERROR","FINISHED"}){


        for (const auto& [key, value] : params) {
            if (std::holds_alternative<double>(value)) {
                this->blackboard_set<float>(key, static_cast<float>(std::get<double>(value)));
            } else if (std::holds_alternative<std::string>(value)) {
                this->blackboard_set<std::string>(key, std::get<std::string>(value));
            }
        }

        // blackboard
        this->blackboard_set<std::shared_ptr<Drone>>("drone", drone);
        this->blackboard_set<int>("waypoint_index", 1);
        this->blackboard_set<bool>("manometro_alinhado",false);
        this->blackboard_set<float>("measured_pressure", -1.0);
        // ... outras variáveis do blackboard ...
        this->blackboard_set<float>("error_x", NAN);
        this->blackboard_set<float>("error_y", NAN);
        

        // states
        
        
        this->add_state("ARMING", std::make_unique<ArmingState>());
        this->add_state("TAKEOFF", std::make_unique<TakeoffState>());
        this->add_state("GOTO",std::make_unique<GoToState>());
        this->add_state("ALIGN",std::make_unique<AlignState>());
        this->add_state("PHOTO",std::make_unique<PhotoState>());
        this->add_state("LANDING", std::make_unique<LandingState>());

        // transitions

        this->add_transitions("ARMING", {
            {"ARMED", "TAKEOFF"},
            {"ERROR", "ERROR"}
        });

        this->add_transitions("TAKEOFF", {
            {"TAKEOFF COMPLETED", "GOTO"},
            {"ERROR", "ERROR"}
        });

        this->add_transitions("GOTO", {
            {"ON TARGET", "ALIGN"},
            {"TRAJECTORY COMPLETED", "LANDING"},
            {"ERROR","ERROR"}
        });

        this->add_transitions("ALIGN", {
            {"ALIGNED", "PHOTO"},
            {"ERROR","ERROR"}
        });

        this->add_transitions("PHOTO", {
            {"PHOTO TAKEN", "GOTO"},
            //{"NOT ALIGNED", "ALIGN"},
            {"ERROR","ERROR"}
        });

        this->add_transitions("LANDING", {
            {"LANDED", "FINISHED"},
            {"ERROR", "ERROR"}
        });

        this->set_initial_state("ARMING");
    }
    
};

class Fase3Node : public rclcpp::Node {
public:
    Fase3Node(std::shared_ptr<Drone> drone)
        : rclcpp::Node("fase_3_node"), drone_(drone) {

        // Declare default parameters — these are overridden by config YMAL files
        // when launching via: ros2 launch mission_1 simulation.launch.py
        std::map<std::string, std::variant<double, std::string>> default_params = {
            // Takeoff
            {"takeoff_height",         -2.0},
            {"max_vertical_velocity",  1.2},
            {"position_tolerance",     0.15},

            // Landing
            {"landing_velocity_max",   0.5},
            {"landing_velocity_min",   0.15},
            {"max_base_height",        0.5},
            {"landing_timeout",        5.0},

            // Movement
            {"max_horizontal_velocity", 0.5},
            {"position_tolerance_mov", 0.1},

            //align
            {"position_tolerance_align",    0.10},
            {"max_horizontal_velocity_align",0.4},

            //limit
            {"limit_value",50.0},

            //Waypoint 1 
            {"x1", 2.0},
            {"y1", -6.0},
            {"z1", -2.0},
                          
            //Waypoint 2 
            {"x2", -2.5},
            {"y2", 0.0},
            {"z2", -2.0},

            //Waypoint 3 
            {"x3", 5.0},
            {"y3", 2.5},
            {"z3", -2.0},

            //Landing point
            {"x4", 5.0},
            {"y4", 2.5},
            {"z4", -2.0}

        };

        auto params = declareAndGetParameters(default_params);

        // Create the FSM
        fsm_ = std::make_unique<Fase3FSM>(drone_, params);

        //subscriber leitura do manometro
        pressure_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/measured_pressure",
            10,
            [this](const std_msgs::msg::Float32::SharedPtr msg) {
                if(this->fsm_) {
                    this->fsm_->blackboard_set<float>("measured_pressure",msg->data);
                }
            }

        );

        //subscriber erro da posição em relação ao manometro

        pos_manometer_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/manometer_error",
            10,
            [this](const geometry_msgs::msg::Point::SharedPtr msg){
                if(this->fsm_){
                    this->fsm_->blackboard_set<float>("error_x",msg->x);
                    this->fsm_->blackboard_set<float>("error_y",msg->y);
                }
            }
        );

        pressure_analysis_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/pressure_analysis",
            10
        );

        this->fsm_->blackboard_set<std::function<void(std::string)>>(
            "publish_pressure_analysis",
            [this](std::string press_analysis){
                std_msgs::msg::String msg;
                msg.data=press_analysis;
                this->pressure_analysis_pub_->publish(msg);
            }
        );


        // Run FSM at 20Hz (50ms period)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&Fase3Node::executeFSM, this)
        );

        RCLCPP_INFO(this->get_logger(), "Fase 3 FSM started");
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
    std::unique_ptr<Fase3FSM> fsm_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr pressure_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr pos_manometer_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pressure_analysis_pub_;
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
    auto mission_node = std::make_shared<Fase3Node>(drone);

    executor.add_node(drone);
    executor.add_node(mission_node);

    executor.spin();

    rclcpp::shutdown();
    return 0;
}





//arming state
//takeoff state
//go to state
//alinhar
//tirar foto
//se tem manometro -> volta pro go to
//se nao -> landing em qualquer lugar