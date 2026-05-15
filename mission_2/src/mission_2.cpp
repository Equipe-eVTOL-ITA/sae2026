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
#include "mission_2/states/lookat_ball_state.hpp"
#include "mission_2/states/goto_ball_state.hpp"
#include "mission_2/states/rise_state.hpp"
#include "mission_2/states/approach_hose_state.hpp"
#include "mission_2/states/mangueira_align_state.hpp"
#include "mission_2/states/drop_the_hook_state.hpp"
#include "mission_2/states/goto_base_state.hpp"

// ── Bayesian 2D spatial filter for ball detection ────────────────────────
// Maintains a log-probability grid over image space [-1,1]x[-1,1].
//
// Key property (Bayesian multiplication):
//   P(ball_pos | z1, z2, ...) ∝ Π P(zi | ball_pos)
//
// When false positives appear at different locations, their Gaussian
// likelihoods are peaked at different cells → the product is near-zero
// everywhere → ratio stays low → no confirmation.
//
// When the real ball is consistently at the same position, likelihoods
// stack at those cells → high peak/mean ratio → confirmation.
class BallBayesFilter {
public:
    static constexpr int   N       = 10;     // 10x10 grid, cell size = 0.2 norm. units
    static constexpr float SIGMA   = 0.35f;  // obs. noise (norm. image units)
    static constexpr float DECAY   = 0.88f;  // log-prior decay per miss (→ more uniform)
    static constexpr float CONFIRM = 8.0f;   // peak/mean ratio threshold

    BallBayesFilter() { reset(); }

    void reset() {
        for (int i = 0; i < N; i++)
            for (int j = 0; j < N; j++)
                g_[i][j] = 0.0f;
    }

    // Feed a detection at (x, y) ∈ [-1,1]
    void observe(float x, float y) {
        for (int i = 0; i < N; i++) {
            float cx = cell_c(i);
            for (int j = 0; j < N; j++) {
                float d2 = (x - cx)*(x - cx) + (y - cell_c(j))*(y - cell_c(j));
                g_[i][j] += -0.5f * d2 / (SIGMA * SIGMA);
            }
        }
        clamp();  // subtract max so overflow never occurs
    }

    // Call each miss tick; decays toward uniform
    void miss() {
        for (int i = 0; i < N; i++)
            for (int j = 0; j < N; j++)
                g_[i][j] *= DECAY;
    }

    struct Result { bool detected; float x, y, ratio; };

    Result query() const {
        float sum = 0.0f, sx = 0.0f, sy = 0.0f;
        for (int i = 0; i < N; i++) {
            float cx = cell_c(i);
            for (int j = 0; j < N; j++) {
                float p = std::exp(g_[i][j]);  // g_ ≤ 0 → p ≤ 1
                sum += p;
                sx  += p * cx;
                sy  += p * cell_c(j);
            }
        }
        // After clamp(), max(g_) = 0, so peak_p = exp(0) = 1.
        // ratio = peak / mean = N*N / sum_exp.
        float ratio = (sum > 1e-9f) ? (float(N * N) / sum) : 1.0f;
        return {ratio >= CONFIRM,
                sum > 0 ? sx / sum : 0.0f,
                sum > 0 ? sy / sum : 0.0f,
                ratio};
    }

private:
    static float cell_c(int i) { return -1.0f + (2.0f * i + 1.0f) / N; }

    void clamp() {
        float mx = -1e10f;
        for (int i = 0; i < N; i++)
            for (int j = 0; j < N; j++)
                mx = std::max(mx, g_[i][j]);
        for (int i = 0; i < N; i++)
            for (int j = 0; j < N; j++)
                g_[i][j] = std::max(g_[i][j] - mx, -50.0f);
    }

    float g_[N][N];
};

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
        this->add_state("LOOKAT_BALL",    std::make_unique<LookAtBallState>());
        this->add_state("GOTO_BALL",      std::make_unique<GoToBallState>());
        this->add_state("RISE",           std::make_unique<RiseState>());
        this->add_state("APPROACH_HOSE",  std::make_unique<ApproachHoseState>());
        this->add_state("MANGUEIRA_ALIGN", std::make_unique<MangueiraAlignState>(true, true));
        this->add_state("DROP_HOOK",      std::make_unique<DropTheHookState>());
        this->add_state("GOTO_BASE",      std::make_unique<GoToBaseState>());
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
            {"BALL_FOUND", "LOOKAT_BALL"},
            {"ERROR", "ERROR"}
        });

        this->add_transitions("LOOKAT_BALL", {
            {"BALL_ALIGNED", "GOTO_BALL"},
            {"BALL_LOST", "SEARCH_BALL"},
            {"ERROR", "ERROR"}
        });

        this->add_transitions("GOTO_BALL", {
            {"REACHED", "RISE"},
            {"BALL_LOST", "SEARCH_BALL"},
            {"ERROR", "ERROR"}
        });

        this->add_transitions("RISE", {
            {"HOSE_DETECTED", "MANGUEIRA_ALIGN"},
            {"RISE_COMPLETED", "APPROACH_HOSE"},
            {"ERROR", "ERROR"}
        });

        this->add_transitions("APPROACH_HOSE", {
            {"HOSE_FOUND", "MANGUEIRA_ALIGN"},
            {"ERROR", "ERROR"}
        });

        this->add_transitions("MANGUEIRA_ALIGN", {
            {"HOSE_LOST", "RISE"},
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
            {"ball_trigger_score",      10000.0}, // área (px²) para transitar GOTO_BALL → RISE
            {"ball_diameter_m",         0.25},  // diâmetro real da bola em metros
            {"ball_cam_scale",          0.7},   // tan(FOV/2) da câmera horizontal
            {"ball_resize_width",       600.0}, // largura de resize do ball_detector
            {"ball_approach_velocity", 0.12},
            {"ball_distance_scale",    100.0},  // mantido para compatibilidade de log
            {"ball_min_area",          100.0},
            {"ball_max_area",        30000.0},
            {"ball_confirm_frames",      3.0},
            {"ball_miss_frames",        10.0},
            {"ball_centering_threshold", 0.15},
            {"ball_centering_kp",        0.8},
            {"ball_centering_frames",    5.0},
            {"ball_centering_miss_tol",  5.0},
            {"ball_lookat_x_tolerance",  0.12},
            {"ball_lookat_y_tolerance",  0.10},
            {"ball_lookat_frames",       5.0},
            {"ball_lookat_yaw_kp",       0.45},
            {"ball_lookat_alt_kp",       0.25},
            {"ball_lookat_max_yaw_step",  0.12},
            {"ball_lookat_max_vertical_step", 0.08},
            {"ball_kp_x", 0.5}, {"ball_ki_x", 0.0}, {"ball_kd_x", 0.1},
            {"ball_kp_y", 0.5}, {"ball_ki_y", 0.0}, {"ball_kd_y", 0.1},
            {"rise_target_z",            -2.0},
            {"rise_climb_rate",           0.3},
            {"rise_timeout_s",           12.0},
            {"rise_hose_confirm_frames",  3.0},
            {"align_tolerance_y",          0.1},
            {"align_tolerance_yaw",        0.15},
            // XY PD gains used by MangueiraAlignState
            {"align_kp",                   0.4},
            {"align_kd",                   0.10},
            {"max_horizontal_velocity_align", 0.20},
            {"align_min_detections",       8.0},
            // Phase 1: translate to hose centre (loose tolerance)
            {"align_translate_tolerance",  0.25},
            {"align_translate_frames",     5.0},
            // Phase 2: yaw alignment
            {"align_yaw_frames",           5.0},
            // Phase 3: fine centring (tight tolerance)
            {"align_fine_tolerance",       0.10},
            {"align_fine_frames",          8.0},
            // Yaw PID
            {"align_kp_y", 0.5}, {"align_ki_y", 0.0}, {"align_kd_y", 0.1},
            {"align_kp_yaw", 0.15}, {"align_ki_yaw", 0.0}, {"align_kd_yaw", 0.05},
            {"hose_approach_speed",          0.2},
            {"hook_script_path",       std::string("~/evtol/dev/scripts/drop_hook.py")},
            // Return to base
            {"ball_lost_frames",            15.0},
            {"max_vel_goto",                 0.5},
            {"hover_before_landing_ticks",  40.0},
            {"target_x",   0.0},
            {"target_y",   0.0},
            {"target_z",  -1.5},
            {"target_yaw", 0.0},
        };

        auto params = declareAndGetParameters(default_params);

        // objeto da FSM
        fsm_ = std::make_unique<Mission2FSM>(drone_, params);

        // Ball detection via Bayesian 2D spatial filter.
        // False positives at scattered locations cancel each other (product of
        // Gaussians peaked at different spots → near-zero everywhere).
        // Real ball at consistent location accumulates probability → confirms.
        ball_sub_ = this->create_subscription<custom_msgs::msg::BallDetection>(
            "ball_detection", 10,
            [this](const custom_msgs::msg::BallDetection::SharedPtr msg) {
                float* min_area_ptr = fsm_->blackboard_get<float>("ball_min_area");
                float min_area = min_area_ptr ? *min_area_ptr : 100.0f;

                float* max_area_ptr = fsm_->blackboard_get<float>("ball_max_area");
                float max_area = max_area_ptr ? *max_area_ptr : 30000.0f;

                bool raw_ok = msg->is_detected
                    && (msg->target_score >= min_area)
                    && (msg->target_score <= max_area);

                if (raw_ok) {
                    ball_bayes_.observe(msg->x_error, msg->y_error);
                    ball_last_score_ = msg->target_score;
                    ball_miss_count_ = 0;
                } else {
                    ball_bayes_.miss();
                    if (++ball_miss_count_ >= 30) {
                        ball_bayes_.reset();  // flush stale belief after ~1.5s of no detection
                        ball_miss_count_ = 0;
                    }
                }

                // Estimativa de distância por câmera pinhole + diâmetro conhecido:
                //   r_px = sqrt(area / π)
                //   dist = D · W / (4 · r_px · cam_scale)
                // onde D = diâmetro real da bola, W = largura da imagem em pixels
                float ball_diam  = fsm_->blackboard_get<float>("ball_diameter_m")
                    ? *fsm_->blackboard_get<float>("ball_diameter_m")  : 0.25f;
                float cam_scale  = fsm_->blackboard_get<float>("ball_cam_scale")
                    ? *fsm_->blackboard_get<float>("ball_cam_scale")   : 0.7f;
                float resize_w   = fsm_->blackboard_get<float>("ball_resize_width")
                    ? *fsm_->blackboard_get<float>("ball_resize_width"): 600.0f;

                float r_px   = (ball_last_score_ > 0.0f)
                    ? std::sqrt(ball_last_score_ / M_PI) : 0.0f;
                float dist_m = (r_px > 0.5f)
                    ? (ball_diam * resize_w / (4.0f * r_px * cam_scale))
                    : 99.0f;
                fsm_->blackboard_set<float>("ball_distance_m", dist_m);

                auto r = ball_bayes_.query();
                fsm_->blackboard_set<bool>("ball_is_detected", r.detected);
                if (r.detected) {
                    fsm_->blackboard_set<float>("ball_x_error",      r.x);
                    fsm_->blackboard_set<float>("ball_y_error",      r.y);
                    fsm_->blackboard_set<float>("ball_target_score", ball_last_score_);
                }

                // Log periódico (a cada 10 callbacks com detecção bruta válida)
                if (raw_ok) {
                    if (ball_log_counter_++ % 10 == 0) {
                        RCLCPP_INFO(this->get_logger(),
                            "[BALL] raw=(%.2f,%.2f) area=%.0f dist≈%.2fm ratio=%.1f det=%s",
                            msg->x_error, msg->y_error, ball_last_score_,
                            dist_m, r.ratio, r.detected ? "YES" : "no");
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
                // Reset stale offsets so the hose alignment state doesn't chase a ghost position.
                fsm_->blackboard_set<float>("hose_offset_x",    0.0f);
                fsm_->blackboard_set<float>("hose_offset_y",    0.0f);
                fsm_->blackboard_set<float>("hose_center_x",    0.0f);
                fsm_->blackboard_set<float>("hose_center_y",    0.0f);
                fsm_->blackboard_set<float>("hose_angle_error", 0.0f);
                hose_ema_first_ = true;  // reset EMA so next detection seeds fresh
            }
        );
        fsm_->blackboard_set<bool>("hose_in_sight", false);

        // subscriber para a detecção da mangueira (posição normalizada e ângulo)
        hose_pos_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/mangueira/position", 10,
            [this](const geometry_msgs::msg::PointStamped::SharedPtr msg) {
                float raw_x = static_cast<float>(msg->point.x);
                float raw_y = static_cast<float>(msg->point.y);

                // EMA suaviza a posição da mangueira (alpha=0.55: rápido mas sem jitter)
                if (hose_ema_first_) {
                    hose_ema_x_ = raw_x;
                    hose_ema_y_ = raw_y;
                    hose_ema_first_ = false;
                } else {
                    hose_ema_x_ = kHoseEmaAlpha * raw_x + (1.0f - kHoseEmaAlpha) * hose_ema_x_;
                    hose_ema_y_ = kHoseEmaAlpha * raw_y + (1.0f - kHoseEmaAlpha) * hose_ema_y_;
                }

                // offset_x/y mantém o raw para debug/display; center_x/y usa EMA para controle
                fsm_->blackboard_set<float>("hose_offset_x", raw_x);
                fsm_->blackboard_set<float>("hose_offset_y", raw_y);
                fsm_->blackboard_set<float>("hose_center_x", hose_ema_x_);
                fsm_->blackboard_set<float>("hose_center_y", hose_ema_y_);
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
    int   ball_log_counter_ = 0;
    int   ball_miss_count_  = 0;
    float ball_last_score_  = 0.0f;
    BallBayesFilter ball_bayes_;

    // Hose position EMA
    float hose_ema_x_   = 0.0f;
    float hose_ema_y_   = 0.0f;
    bool  hose_ema_first_ = true;
    static constexpr float kHoseEmaAlpha = 0.55f;
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
