#include <memory>
#include <iostream>
#include <map>
#include <string>
#include <variant>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <custom_msgs/msg/ball_detection.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <nav_msgs/msg/path.hpp>

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
#include "mission_2/states/approach_hose_state.hpp"
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
        this->add_state("ARMING",         std::make_unique<ArmingState>());
        this->add_state("TAKEOFF",        std::make_unique<TakeoffState>());
        this->add_state("SEARCH_BALL",    std::make_unique<SearchBallState>());
        this->add_state("GOTO_BALL",      std::make_unique<GoToBallState>());
        this->add_state("RISE",           std::make_unique<RiseState>());
        this->add_state("APPROACH_HOSE",  std::make_unique<ApproachHoseState>());
        this->add_state("ALIGN",          std::make_unique<AlignState>(true, true));
        this->add_state("DROP_HOOK",      std::make_unique<DropTheHookState>());
        this->add_state("GOTO_BASE",      std::make_unique<GoToState>());
        this->add_state("LANDING",        std::make_unique<LandingState>());

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
            {"RISE_COMPLETED", "APPROACH_HOSE"},
            {"ERROR", "ERROR"}
        });

        this->add_transitions("APPROACH_HOSE", {
            {"HOSE_FOUND", "ALIGN"},
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
            {"search_speed",           0.5},
            {"search_radius",          3.0},
            {"ball_trigger_score",     5000.0},
            {"ball_approach_velocity", 0.2},
            {"ball_distance_scale",    100.0},
            {"ball_min_area",          100.0},
            {"ball_max_area",        30000.0},
            {"ball_confirm_frames",      3.0},
            {"ball_miss_frames",        10.0},
            {"ball_centering_threshold", 0.15},
            {"ball_centering_kp",        0.8},
            {"ball_centering_frames",    5.0},
            {"ball_centering_miss_tol",  5.0},
            {"ball_kp_x", 0.5}, {"ball_ki_x", 0.0}, {"ball_kd_x", 0.1},
            {"ball_kp_y", 0.5}, {"ball_ki_y", 0.0}, {"ball_kd_y", 0.1},
            {"rise_target_z", -4.5},
            {"align_tolerance_y",      0.1},
            {"align_tolerance_yaw",    0.05},
            {"align_kp_y", 0.5}, {"align_ki_y", 0.0}, {"align_kd_y", 0.1},
            {"align_kp_yaw", 0.5}, {"align_ki_yaw", 0.0}, {"align_kd_yaw", 0.1},
            {"hose_approach_speed",    0.2},
            {"hook_script_path",       std::string("~/evtol/dev/scripts/drop_hook.py")},
            {"target_x", 0.0},
            {"target_y", 0.0},
            {"target_z", -2.0}, // Flight altitude to return
            {"target_yaw", 0.0},
        };

        auto params = declareAndGetParameters(default_params);

        // objeto da FSM
        fsm_ = std::make_unique<Mission2FSM>(drone_, params);

        // subscriber para as mensagens de detecção da bola
        // Persistence filter: N frames confirmados para detectar, M frames para declarar perdida.
        // Filtra também áreas pequenas (false positives de ruído).
        ball_sub_ = this->create_subscription<custom_msgs::msg::BallDetection>(
            "ball_detection", 10,
            [this](const custom_msgs::msg::BallDetection::SharedPtr msg) {
                float* min_area_ptr = fsm_->blackboard_get<float>("ball_min_area");
                float min_area = min_area_ptr ? *min_area_ptr : 100.0f;

                float* max_area_ptr = fsm_->blackboard_get<float>("ball_max_area");
                float max_area = max_area_ptr ? *max_area_ptr : 30000.0f;

                float* cf_ptr = fsm_->blackboard_get<float>("ball_confirm_frames");
                int confirm_frames = cf_ptr ? static_cast<int>(*cf_ptr) : 3;

                float* mf_ptr = fsm_->blackboard_get<float>("ball_miss_frames");
                int miss_frames = mf_ptr ? static_cast<int>(*mf_ptr) : 10;

                bool raw_ok = msg->is_detected
                    && (msg->target_score >= min_area)
                    && (msg->target_score <= max_area);

                if (raw_ok) {
                    ball_miss_counter_ = 0;
                    ball_detect_counter_++;

                    // Atualiza valores apenas quando confiáveis
                    fsm_->blackboard_set<float>("ball_target_score", msg->target_score);
                    fsm_->blackboard_set<float>("ball_x_error",      msg->x_error);
                    fsm_->blackboard_set<float>("ball_y_error",      msg->y_error);

                    if (ball_detect_counter_ >= confirm_frames) {
                        fsm_->blackboard_set<bool>("ball_is_detected", true);
                    }
                } else {
                    ball_detect_counter_ = 0;
                    ball_miss_counter_++;
                    if (ball_miss_counter_ >= miss_frames) {
                        fsm_->blackboard_set<bool>("ball_is_detected", false);
                    }
                }

                // Log periódico (a cada 10 callbacks com detecção válida)
                if (raw_ok) {
                    float* k_ptr = fsm_->blackboard_get<float>("ball_distance_scale");
                    float k_ball = k_ptr ? *k_ptr : 100.0f;
                    float area   = msg->target_score;
                    float est_dist = (area > 0.0f) ? (k_ball / std::sqrt(area)) : -1.0f;
                    if (ball_log_counter_++ % 10 == 0) {
                        RCLCPP_INFO(this->get_logger(),
                            "[BALL] err=(%.2f,%.2f) area=%.0f est_dist≈%.2fm confirmed=%d",
                            msg->x_error, msg->y_error, area, est_dist, ball_detect_counter_);
                    }
                } else {
                    ball_log_counter_ = 0;
                }
            }
        );

        // Timer that resets hose_in_sight to False after 500ms of no detection.
        // Each mangueira/position callback resets this timer.
        hose_timeout_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            [this]() {
                fsm_->blackboard_set<bool>("hose_in_sight", false);
                // Reset stale offsets so AlignState doesn't chase a ghost position.
                // AlignState will see offset=0 (within tolerance) and proceed.
                fsm_->blackboard_set<float>("hose_offset_y",    0.0f);
                fsm_->blackboard_set<float>("hose_angle_error", 0.0f);
            }
        );
        fsm_->blackboard_set<bool>("hose_in_sight", false);

        // subscriber para a detecção da mangueira (posição normalizada e ângulo)
        hose_pos_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/mangueira/position", 10,
            [this](const geometry_msgs::msg::PointStamped::SharedPtr msg) {
                // Expect normalized coordinates [0,1]; convert to offset centered at 0.0
                float offset_y = static_cast<float>(msg->point.y) - 0.5f;
                fsm_->blackboard_set<float>("hose_offset_y", offset_y);
                // Mark hose as in sight; timer resets to 500ms before going False
                fsm_->blackboard_set<bool>("hose_in_sight", true);
                hose_timeout_->reset();
            }
        );

        hose_angle_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/mangueira/angle", 10,
            [this](const std_msgs::msg::Float64::SharedPtr msg) {
                // Angle is expected in radians ([-pi/2, pi/2])
                float angle = static_cast<float>(msg->data);
                fsm_->blackboard_set<float>("hose_angle_error", angle);
            }
        );

        // Trajectory publisher for RViz (nav_msgs/Path in ENU frame)
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/drone_trajectory", 10);
        trajectory_.header.frame_id = "map";

        // rodar a FSM a 20Hz (periodo de 50ms)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&Mission2Node::executeFSM, this)
        );

        RCLCPP_INFO(this->get_logger(), "Mission 2 FSM started");
    }

private:
    void executeFSM() {
        // Append position to trajectory and publish (NED → ENU for RViz)
        auto pos = drone_->getLocalPosition();
        geometry_msgs::msg::PoseStamped ps;
        ps.header.stamp    = this->now();
        ps.header.frame_id = "map";
        ps.pose.position.x =  (float)pos.y();
        ps.pose.position.y =  (float)pos.x();
        ps.pose.position.z = -(float)pos.z();
        ps.pose.orientation.w = 1.0;
        trajectory_.header.stamp = ps.header.stamp;
        trajectory_.poses.push_back(ps);
        path_pub_->publish(trajectory_);

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
    rclcpp::TimerBase::SharedPtr hose_timeout_;
    rclcpp::Subscription<custom_msgs::msg::BallDetection>::SharedPtr ball_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr hose_pos_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr hose_angle_sub_;
    int ball_log_counter_    = 0;
    int ball_detect_counter_ = 0;
    int ball_miss_counter_   = 0;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    nav_msgs::msg::Path trajectory_;
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
