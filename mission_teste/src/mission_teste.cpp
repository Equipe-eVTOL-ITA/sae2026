#include <memory>
#include <map>
#include <string>
#include <variant>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <custom_msgs/msg/bouncing_detection.hpp>

#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"

#include <stdstates/arming_state.hpp>
#include <stdstates/takeoff_state.hpp>
#include <stdstates/landing_state.hpp>
#include <mission_teste/go_to_test.hpp>
#include <mission_teste/align_state.hpp>

class TesteFSM : public fsm::FSM {
public:
    TesteFSM(
        std::shared_ptr<Drone> drone,
        const std::map<std::string, std::variant<double, std::string>>& params
    ) : fsm::FSM({"ERROR", "FINISHED"}) {

        this->blackboard_set<std::shared_ptr<Drone>>("drone", drone);

        for (const auto& [key, value] : params) {
            if (std::holds_alternative<double>(value)) {
                this->blackboard_set<float>(key, static_cast<float>(std::get<double>(value)));
            } else if (std::holds_alternative<std::string>(value)) {
                this->blackboard_set<std::string>(key, std::get<std::string>(value));
            }
        }

        // ── States ──────────────────────────────────────────────────────────
        this->add_state("ARMING",    std::make_unique<ArmingState>());
        this->add_state("TAKEOFF",   std::make_unique<TakeoffState>());
        this->add_state("GOTOTESTE", std::make_unique<GoToTesteState>());
        this->add_state("ALIGN",     std::make_unique<AlignState>());
        this->add_state("LANDING",   std::make_unique<LandingState>());

        // ── Transitions ─────────────────────────────────────────────────────
        this->add_transitions("ARMING", {
            {"ARMED", "TAKEOFF"},
            {"ERROR", "ERROR"}
        });

        this->add_transitions("TAKEOFF", {
            {"TAKEOFF COMPLETED", "GOTOTESTE"},
            {"ERROR", "ERROR"}
        });

        this->add_transitions("GOTOTESTE", {
            {"ON TARGET", "ALIGN"},
            {"ERROR",     "ERROR"}
        });

        this->add_transitions("ALIGN", {
            {"ALIGNED",   "LANDING"},
            {"TIMEOUT",   "LANDING"},     // fallback seguro
            {"ERROR",     "ERROR"}
        });

        this->add_transitions("LANDING", {
            {"LANDED", "FINISHED"},
            {"ERROR",  "ERROR"}
        });

        this->set_initial_state("ARMING");
    }
};

class TesteNode : public rclcpp::Node {
public:
    explicit TesteNode(std::shared_ptr<Drone> drone)
        : rclcpp::Node("teste_node"), drone_(drone) {

        std::map<std::string, std::variant<double, std::string>> default_params = {
            // Takeoff
            {"takeoff_height",        -2.0},
            {"max_vertical_velocity",  1.2},
            {"position_tolerance",     0.15},

            // Landing
            {"landing_velocity_max",   0.5},
            {"landing_velocity_min",   0.15},
            {"max_base_height",        0.5},
            {"landing_timeout",        5.0},

            // GoToTeste
            {"max_horizontal_velocity",  0.5},
            {"position_tolerance_mov",   0.15},
            {"x",  -4.2},
            {"y",  -2.1},
            {"z", -2.0},

            // AlignState
            {"base_tolerance",    0.10},   // metros
            {"base_max_velocity", 0.30},   // m/s
            {"base_kp_x",         0.70},
            {"base_kp_y",         0.70},
            {"base_kd_x",         0.05},
            {"base_kd_y",         0.05},
            {"cam_scale",         0.70},   // tan(FOV/2), FOV≈70°
            {"align_frames",     10.0},    // frames consecutivos para confirmar
            {"align_timeout",    30.0},    // timeout global em segundos
        };

        auto params = declareAndGetParameters(default_params);
        fsm_ = std::make_unique<TesteFSM>(drone_, params);

        // Subscriber: detecção da base pelo bouncing_detector
        rclcpp::QoS qos(10);
        qos.reliable();
        bouncing_sub_ = this->create_subscription<custom_msgs::msg::BouncingDetection>(
            "/bouncing_detection", qos,
            [this](const custom_msgs::msg::BouncingDetection::SharedPtr msg) {
                // EMA suaviza posição da base antes de chegar ao AlignState
                float raw_x = msg->target_base_x_error;
                float raw_y = msg->target_base_y_error;

                if (msg->target_base_in_sight) {
                    if (ema_first_) {
                        ema_x_ = raw_x;
                        ema_y_ = raw_y;
                        ema_first_ = false;
                    } else {
                        ema_x_ = kEmaAlpha * raw_x + (1.0f - kEmaAlpha) * ema_x_;
                        ema_y_ = kEmaAlpha * raw_y + (1.0f - kEmaAlpha) * ema_y_;
                    }
                } else {
                    ema_first_ = true;  // reset ao perder a base
                }

                fsm_->blackboard_set<bool> ("target_base_in_sight",  msg->target_base_in_sight);
                fsm_->blackboard_set<float>("target_base_x_error",   ema_x_);
                fsm_->blackboard_set<float>("target_base_y_error",   ema_y_);
            }
        );

        // Trajetória para RViz (NED → ENU)
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/drone_trajectory", 10);
        trajectory_.header.frame_id = "map";

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&TesteNode::executeFSM, this));

        RCLCPP_INFO(this->get_logger(), "Mission Teste FSM started");
    }

private:
    void executeFSM() {
        auto pos = drone_->getLocalPosition();
        geometry_msgs::msg::PoseStamped ps;
        ps.header.stamp    = this->now();
        ps.header.frame_id = "map";
        ps.pose.position.x =  static_cast<float>(pos.y());
        ps.pose.position.y =  static_cast<float>(pos.x());
        ps.pose.position.z = -static_cast<float>(pos.z());
        ps.pose.orientation.w = 1.0;
        trajectory_.header.stamp = ps.header.stamp;
        trajectory_.poses.push_back(ps);
        path_pub_->publish(trajectory_);

        if (rclcpp::ok() && !fsm_->is_finished()) {
            fsm_->execute();
        } else {
            RCLCPP_INFO(this->get_logger(), "FSM finished: %s",
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
    std::unique_ptr<TesteFSM> fsm_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<custom_msgs::msg::BouncingDetection>::SharedPtr bouncing_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    nav_msgs::msg::Path trajectory_;

    // EMA para erros de posição da base
    float ema_x_    = 0.0f;
    float ema_y_    = 0.0f;
    bool  ema_first_ = true;
    static constexpr float kEmaAlpha = 0.45f;
};

int main(int argc, const char *argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::executors::MultiThreadedExecutor executor;
    auto drone        = std::make_shared<Drone>();
    auto mission_node = std::make_shared<TesteNode>(drone);

    executor.add_node(drone);
    executor.add_node(mission_node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
