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
#include <std_srvs/srv/set_bool.hpp>
#include <custom_msgs/msg/base_detection.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

// Standard states from stdstates
#include "stdstates/takeoff_state.hpp"
#include "stdstates/landing_state.hpp"
#include "mission_3/align_state.hpp"
#include "mission_3/approach_state.hpp"
#include "mission_3/goto_state.hpp"
#include "mission_3/photo_state.hpp"
#include "mission_3/search_state.hpp"
#include "mission_3/termination_state.hpp"



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
        this->blackboard_set<int>("limite_ticks_",20);
        this->blackboard_set<int>("limit_miss_align",100);
        

        // states
        
        
        this->add_state("TAKEOFF",  std::make_unique<TakeoffState>());
        this->add_state("GOTO",     std::make_unique<GoToState>());
        this->add_state("ALIGN",    std::make_unique<AlignState>());
        this->add_state("SEARCH",   std::make_unique<SearchState>());
        this->add_state("APPROACH", std::make_unique<ApproachState>());
        this->add_state("PHOTO",    std::make_unique<PhotoState>());
        this->add_state("LANDING",  std::make_unique<LandingState>());
        this->add_state("TERMINATION", std::make_unique<TerminationState>());

        // transitions

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
            {"ALIGNED", "APPROACH"},
            {"LOST TARGET", "SEARCH"},
            {"ERROR",   "ERROR"}
        });

        this->add_transitions("SEARCH", {
            {"FOUND IT", "ALIGN"},
            {"FAILED","TERMINATION"},
            {"ERROR", "ERROR"}
        });

        this->add_transitions("APPROACH", {
            {"AT_ALTITUDE", "PHOTO"},
            {"ERROR",       "ERROR"}
        });

        this->add_transitions("PHOTO", {
            {"PHOTO TAKEN", "GOTO"},
            {"ERROR","ERROR"}
        });

        this->add_transitions("LANDING", {
            {"LANDED", "FINISHED"},
            {"ERROR", "ERROR"}
        });

        this->add_transitions("TERMINATION", {
            {"ON LANDING POINT", "LANDING"},
            {"ERROR","ERROR"}
        });

        this->set_initial_state("TAKEOFF");
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

            // ALIGN
            {"position_tolerance_align",     0.10},
            {"max_horizontal_velocity_align", 0.3},
            {"align_kp",                     0.7},
            {"align_kd",                     0.05},
            {"align_min_detections",         10.0},

            // SEARCH
            {"max_horizontal_velocity_search", 0.3},
            {"position_tolerance_search",      0.1},
            {"step",                           0.7},

            // APPROACH
            {"manometer_approach_altitude",  -1.0},
            {"approach_velocity",            0.3},
            {"approach_hold_ticks",          20.0},
            {"approach_settle_ticks",        40.0},

            //limit
            {"limit_value",50.0},

            //Waypoint 1
            {"x1", -6.0},
            {"y1", -2.0},
            {"z1", -2.2},

            //Waypoint 2
            {"x2", -2.5},
            {"y2", 0.0},
            {"z2", -2.2},

            //Waypoint 3
            {"x3", 5.0},
            {"y3", 2.5},
            {"z3", -2.2},

            // Return-to-home point (takeoff origin)
            {"x4", 0.0},
            {"y4", 0.0},
            {"z4", -2.2}

        };

        auto params = declareAndGetParameters(default_params);

        // Create the FSM
        fsm_ = std::make_unique<Fase3FSM>(drone_, params);

        // Callback groups: timer runs separately from CV subscribers so a slow
        // detector callback can't block the 20Hz FSM tick that sends offboard
        // control mode to PX4.
        timer_cb_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        cv_cb_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

        rclcpp::SubscriptionOptions cv_opts;
        cv_opts.callback_group = cv_cb_group_;

        //subscriber leitura do manometro
        pressure_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/measured_pressure",
            10,
            [this](const std_msgs::msg::Float32::SharedPtr msg) {
                if(this->fsm_) {
                    this->fsm_->blackboard_set<float>("measured_pressure",msg->data);
                }
            },
            cv_opts
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
            },
            cv_opts
        );

        play_audio_client_ = this->create_client<std_srvs::srv::SetBool>("play_audio");

        pressure_analysis_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/pressure_analysis", 10);

        this->fsm_->blackboard_set<std::function<void(std::string)>>(
            "publish_pressure_analysis",
            [this](std::string press_analysis){
                // Publish topic for manometro_detector frame saving
                std_msgs::msg::String topic_msg;
                topic_msg.data = press_analysis;
                this->pressure_analysis_pub_->publish(topic_msg);

                // Call audio service (guards against overlapping playback)
                if (!play_audio_client_->service_is_ready()) {
                    RCLCPP_WARN(this->get_logger(), "play_audio service not available");
                    return;
                }
                auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
                req->data = (press_analysis.find("above") != std::string::npos);
                play_audio_client_->async_send_request(req,
                    [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture fut){
                        auto res = fut.get();
                        RCLCPP_INFO(this->get_logger(), "play_audio: %s (%s)",
                            res->success ? "ok" : "skip", res->message.c_str());
                    });
            }
        );


        this->declare_parameter("enable_trajectory_viz", false);
        enable_trajectory_viz_ = this->get_parameter("enable_trajectory_viz").as_bool();

        // Trajectory publisher for RViz (nav_msgs/Path in ENU frame)
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/drone_trajectory", 10);
        trajectory_.header.frame_id = "map";

        // Run FSM at 20Hz (50ms period) — dedicated callback group, see above
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&Fase3Node::executeFSM, this),
            timer_cb_group_
        );

        RCLCPP_INFO(this->get_logger(), "Fase 3 FSM started");
    }



private:

    void executeFSM() {
        // Append position to trajectory and publish (NED → ENU for RViz)
        auto pos = drone_->getLocalPosition();
        geometry_msgs::msg::PoseStamped ps;
        ps.header.stamp    = this->now();
        ps.header.frame_id = "map";
        ps.pose.position.x =  (float)pos.y();   // East  = NED y
        ps.pose.position.y =  (float)pos.x();   // North = NED x
        ps.pose.position.z = -(float)pos.z();   // Up    = -NED z
        ps.pose.orientation.w = 1.0;
        if (enable_trajectory_viz_) {
            trajectory_.poses.push_back(ps);
            if (trajectory_.poses.size() > 500)
                trajectory_.poses.erase(trajectory_.poses.begin());
            trajectory_.header.stamp = ps.header.stamp;
            path_pub_->publish(trajectory_);
        }

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
    rclcpp::CallbackGroup::SharedPtr timer_cb_group_;
    rclcpp::CallbackGroup::SharedPtr cv_cb_group_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr pressure_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr pos_manometer_sub_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr play_audio_client_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pressure_analysis_pub_;
    bool enable_trajectory_viz_ = false;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    nav_msgs::msg::Path trajectory_;
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

    // The Drone owns its own MultiThreadedExecutor + spin thread internally,
    // so it is intentionally NOT added to this outer executor.
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