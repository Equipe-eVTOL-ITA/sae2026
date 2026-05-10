#include <memory>
#include <iostream>
#include <map>
#include <string>
#include <variant>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "custom_msgs/msg/bouncing_detection.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/string.hpp"

// Standard states from stdstates
#include "stdstates/arming_state.hpp"
#include "stdstates/takeoff_state.hpp"
#include "stdstates/landing_state.hpp"

// Mission-specific states 
#include "mission_1/states/initial_aruco_search_state.hpp"
#include "mission_1/states/search_aruco_state.hpp"
#include "mission_1/states/go_to_aruco_state.hpp"
#include "mission_1/states/search_base_state.hpp"
#include "mission_1/states/go_to_base_state.hpp"
#include "mission_1/states/descend_for_shape_state.hpp"


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
        this->add_state("INITIAL_ARUCO_SEARCH", std::make_unique<InitialArucoSearchState>());
        this->add_state("SEARCH_ARUCO", std::make_unique<SearchArucoState>());
        this->add_state("GO_TO_ARUCO", std::make_unique<GoToArucoState>());
        this->add_state("DESCEND_FOR_SHAPE", std::make_unique<DescendForShapeState>());
        this->add_state("SEARCH_BASE", std::make_unique<SearchBaseState>());
        this->add_state("GO_TO_BASE", std::make_unique<GoToBaseState>());
        this->add_state("LANDING", std::make_unique<LandingState>());

        // ================== TRANSITIONS ===================
        this->add_transitions("ARMING", {
            {"ARMED", "TAKEOFF"},
            {"ERROR", "ERROR"}
        });

        this->add_transitions("TAKEOFF", {
            {"TAKEOFF COMPLETED", "INITIAL_ARUCO_SEARCH"},
            {"ERROR", "ERROR"}
        });

        this->add_transitions("INITIAL_ARUCO_SEARCH", {
            {"ARUCO_FOUND", "GO_TO_ARUCO"},
            {"MAX_ALTITUDE_REACHED", "SEARCH_ARUCO"},
            {"ERROR", "ERROR"}
        });

        this->add_transitions("SEARCH_ARUCO", {
            {"ARUCO_FOUND", "GO_TO_ARUCO"},
            {"ERROR", "ERROR"}
        });

        this->add_transitions("GO_TO_ARUCO", {
            {"ARUCO_LOST", "SEARCH_ARUCO"},
            {"SHAPE_UNKNOWN", "DESCEND_FOR_SHAPE"},
            {"UNKNOWN_BASE", "SEARCH_BASE"},
            {"KNOWN_BASE", "GO_TO_BASE"},
            {"ERROR", "ERROR"}
        });

        this->add_transitions("DESCEND_FOR_SHAPE", {
            {"SHAPE_FOUND", "GO_TO_ARUCO"},
            {"ARUCO_LOST", "SEARCH_ARUCO"},
            {"ERROR", "ERROR"}
        });

        this->add_transitions("SEARCH_BASE", {
            {"BASE_FOUND", "GO_TO_BASE"},
            {"ERROR", "ERROR"}
        });

        this->add_transitions("GO_TO_BASE", {
            {"ALIGNED", "LANDING"},
            {"BASE_LOST", "SEARCH_BASE"},
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

            // Mission 1 Parameters
            {"z_max_search", -2.5},
            {"aruco_spiral_step", 2.5},
            {"search_aruco_velocity", 0.4},
            {"aruco_persistence_frames", 3.0},
            {"aruco_tolerance", 0.15},
            {"aruco_kp_x", 0.6},
            {"aruco_kp_y", 0.6},
            {"base_spiral_step", 1.0},
            {"base_tolerance", 0.10},
            {"base_kp_x", 0.5},
            {"base_kp_y", 0.5},
            {"shape_id_altitude", -1.0},
            {"shape_id_velocity", 0.3},
            {"base_persistence_frames", 3.0},
            {"base_cam_scale", 0.7},
            {"base_max_err_radius", 0.7},
            {"aruco_align_frames", 5.0},
        };

        auto params = declareAndGetParameters(default_params);

        // Create the FSM
        fsm_ = std::make_unique<Mission1FSM>(drone_, params);

        // Run FSM at 20Hz (50ms period)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&Mission1Node::executeFSM, this)
        );

        // Trajectory publisher for RViz (nav_msgs/Path in ENU frame)
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/drone_trajectory", 10);

        // Publishes whenever a base is seen for the first time (goes to rosbag)
        discovered_bases_pub_ = this->create_publisher<std_msgs::msg::String>("/discovered_bases", 10);
        trajectory_.header.frame_id = "map";

        // Subscribes to the computer vision node
        cv_sub_ = this->create_subscription<custom_msgs::msg::BouncingDetection>(
            "bouncing_detection", 10,
            std::bind(&Mission1Node::cv_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "Mission 1 FSM started");
    }

private:
    void cv_callback(const custom_msgs::msg::BouncingDetection::SharedPtr msg) {
        // ArUco info
        fsm_->blackboard_set<bool>("aruco_detected", msg->aruco_detected);
        if (msg->aruco_detected) {
            fsm_->blackboard_set<int>("aruco_id", msg->aruco_id);
            fsm_->blackboard_set<float>("aruco_x_error", msg->aruco_x_error);
            fsm_->blackboard_set<float>("aruco_y_error", msg->aruco_y_error);
            fsm_->blackboard_set<std::string>("aruco_shape", msg->aruco_shape);
        }
        if (msg->aruco_detected != prev_aruco_detected_) {
            if (msg->aruco_detected)
                RCLCPP_INFO(this->get_logger(), "[CV] ArUco DETECTED id=%d shape=%s",
                    msg->aruco_id, msg->aruco_shape.c_str());
            else
                RCLCPP_INFO(this->get_logger(), "[CV] ArUco LOST");
            prev_aruco_detected_ = msg->aruco_detected;
        }

        // Target Base info — latched: once identified, never auto-reset
        // (the shape + divisibility result is deterministic for a fixed ArUco ID)
        if (msg->target_calculated) {
            fsm_->blackboard_set<bool>("target_calculated", true);
            fsm_->blackboard_set<std::string>("target_base", msg->target_base);
        }
        if (msg->target_calculated && !prev_target_calculated_) {
            RCLCPP_INFO(this->get_logger(), "[CV] Target identified: %s",
                msg->target_base.c_str());
            prev_target_calculated_ = true;
        }

        // Visible Bases
        fsm_->blackboard_set<bool>("target_base_in_sight", msg->target_base_in_sight);
        if (msg->target_base_in_sight) {
            fsm_->blackboard_set<float>("target_base_x_error", msg->target_base_x_error);
            fsm_->blackboard_set<float>("target_base_y_error", msg->target_base_y_error);
        }
        if (msg->target_base_in_sight != prev_target_base_in_sight_) {
            if (msg->target_base_in_sight)
                RCLCPP_INFO(this->get_logger(), "[CV] Target base IN SIGHT (err=%.2f,%.2f)",
                    msg->target_base_x_error, msg->target_base_y_error);
            else
                RCLCPP_INFO(this->get_logger(), "[CV] Target base LOST");
            prev_target_base_in_sight_ = msg->target_base_in_sight;
        }

        // Base position cache: estimate world position from camera error + altitude.
        // Camera is downward-facing (FRD): err_y<0=forward(+X body), err_x>0=right(+Y body).
        // Only update if the new estimate has smaller error radius (more confident).
        {
            float* cam_scale_ptr = fsm_->blackboard_get<float>("base_cam_scale");
            float cam_scale = cam_scale_ptr ? *cam_scale_ptr : 0.7f;
            float alt = -static_cast<float>(drone_->getLocalPosition().z());
            float yaw = static_cast<float>(drone_->getOrientation()[2]);
            auto  dpos = drone_->getLocalPosition();

            float* max_err_ptr = fsm_->blackboard_get<float>("base_max_err_radius");
            float max_err_radius = max_err_ptr ? *max_err_ptr : 0.7f;

            auto store_base_pos = [&](const std::string& label, float ex, float ey) {
                if (label.empty()) return;
                // Reject bases whose centre is too close to the frame edge — partial
                // detections (base cut off at the border) produce unreliable positions.
                if (std::hypot(ex, ey) > max_err_radius) return;
                float ldx = -ey * alt * cam_scale;
                float ldy =  ex * alt * cam_scale;
                float wx  = static_cast<float>(dpos.x())
                            + ldx * std::cos(yaw) - ldy * std::sin(yaw);
                float wy  = static_cast<float>(dpos.y())
                            + ldx * std::sin(yaw) + ldy * std::cos(yaw);
                float r   = std::max(0.5f, std::hypot(ex, ey) * alt * cam_scale + 0.5f);
                std::string rk = "known_base_" + label + "_r";
                float* prev_r = fsm_->blackboard_get<float>(rk);
                bool is_new = (prev_r == nullptr);
                if (!prev_r || r < *prev_r) {
                    fsm_->blackboard_set<float>("known_base_" + label + "_x", wx);
                    fsm_->blackboard_set<float>("known_base_" + label + "_y", wy);
                    fsm_->blackboard_set<float>(rk, r);
                }
                if (is_new) {
                    std::string discovery = "[DISCOVERY] " + label +
                        " @ (" + std::to_string(wx) + ", " + std::to_string(wy) +
                        ") r=" + std::to_string(r);
                    RCLCPP_INFO(this->get_logger(), "%s", discovery.c_str());
                    std_msgs::msg::String pub_msg;
                    pub_msg.data = discovery;
                    discovered_bases_pub_->publish(pub_msg);
                }
            };

            for (size_t i = 0; i < msg->visible_bases.size(); ++i)
                store_base_pos(msg->visible_bases[i],
                               msg->visible_bases_x_error[i],
                               msg->visible_bases_y_error[i]);
            if (msg->target_base_in_sight)
                store_base_pos(msg->target_base,
                               msg->target_base_x_error,
                               msg->target_base_y_error);
        }
    }

    void executeFSM() {
        auto pos    = drone_->getLocalPosition();
        auto orient = drone_->getOrientation();

        // Append position to trajectory and publish (NED → ENU for RViz)
        geometry_msgs::msg::PoseStamped ps;
        ps.header.stamp      = this->now();
        ps.header.frame_id   = "map";
        ps.pose.position.x   =  (float)pos.y();   // East  = NED y
        ps.pose.position.y   =  (float)pos.x();   // North = NED x
        ps.pose.position.z   = -(float)pos.z();   // Up    = -NED z
        ps.pose.orientation.w = 1.0;
        trajectory_.header.stamp = ps.header.stamp;
        trajectory_.poses.push_back(ps);
        path_pub_->publish(trajectory_);

        // State + position log every 2 s (40 ticks at 20 Hz)
        if (pos_log_counter_++ % 40 == 0) {
            RCLCPP_INFO(this->get_logger(), "[%s] pos=(%.2f,%.2f,%.2f) yaw=%.2f rad",
                fsm_->get_current_state().c_str(),
                (float)pos.x(), (float)pos.y(), (float)pos.z(), (float)orient[2]);
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
    std::unique_ptr<Mission1FSM> fsm_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<custom_msgs::msg::BouncingDetection>::SharedPtr cv_sub_;

    bool prev_aruco_detected_       = false;
    bool prev_target_base_in_sight_ = false;
    bool prev_target_calculated_    = false;
    int  pos_log_counter_           = 0;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr discovered_bases_pub_;
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
    auto mission_node = std::make_shared<Mission1Node>(drone);

    executor.add_node(drone);
    executor.add_node(mission_node);

    executor.spin();

    rclcpp::shutdown();
    return 0;
}
