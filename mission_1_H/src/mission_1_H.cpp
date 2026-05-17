#include <memory>
#include <iostream>
#include <map>
#include <set>
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
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"

// Standard states from stdstates
#include "stdstates/arming_state.hpp"
#include "stdstates/takeoff_state.hpp"
#include "stdstates/landing_state.hpp"

// Mission-specific states
#include "mission_1/states/initial_aruco_search_state.hpp"
#include "mission_1/states/search_aruco_state.hpp"
#include "mission_1/states/go_to_aruco_state.hpp"
#include "mission_1/states/go_to_base_state.hpp"
#include "mission_1/states/descend_for_shape_state.hpp"
#include "mission_1/states/h_search_base_state.hpp"


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
        this->add_state("H_SEARCH_BASE", std::make_unique<HSearchBaseState>());
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
            {"UNKNOWN_BASE", "H_SEARCH_BASE"},
            {"KNOWN_BASE", "GO_TO_BASE"},
            {"ERROR", "ERROR"}
        });

        this->add_transitions("DESCEND_FOR_SHAPE", {
            {"SHAPE_FOUND", "GO_TO_ARUCO"},
            {"ARUCO_LOST", "SEARCH_ARUCO"},
            {"ERROR", "ERROR"}
        });

        this->add_transitions("H_SEARCH_BASE", {
            {"BASE_FOUND",    "GO_TO_BASE"},
            {"SEARCH_FAILED", "LANDING"},   // ambos os lados esgotados → pousa
            {"ERROR",         "ERROR"}
        });

        this->add_transitions("GO_TO_BASE", {
            {"ALIGNED",   "LANDING"},
            {"BASE_LOST", "H_SEARCH_BASE"},  // relança busca em H ao perder a base
            {"ERROR",     "ERROR"}
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
        : rclcpp::Node("mission_1_H_node"), drone_(drone) {

        // Declare default parameters — these are overridden by config YAML files
        // when launching via: ros2 launch mission_1_H simulation.launch.py
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
            {"base_dedup_radius", 1.5},
            {"aruco_exclusion_radius", 0.5},
            {"aruco_kd_x", 0.05},
            {"aruco_kd_y", 0.05},
            {"base_kd_x",  0.03},
            {"base_kd_y",  0.03},
            {"aruco_align_frames",   5.0},
            {"base_align_frames",   10.0},
            {"base_max_miss_ticks", 60.0},
            // H-search parameters
            {"search_base_altitude",              -2.5},
            {"distancia_percorrida_perpendicular", 2.5},
            {"lambda_1",  0.5},
            {"lambda_2",  1.0},
            {"lambda_3", -0.5},
            {"lambda_4", -1.0},
            {"h_search_dwell_ticks", 40.0},
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

        // Marker array: visualise discovered base positions + labels in RViz2
        base_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/mission_1_H/base_markers", 10);
        // Publish an empty array immediately so the topic is visible in RViz2 from startup
        base_markers_pub_->publish(visualization_msgs::msg::MarkerArray{});
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
            fsm_->blackboard_set<std::string>("aruco_shape", msg->aruco_shape);

            // EMA filter on ArUco position: smooths single-frame noise and
            // suppresses the derivative spike that occurs on flicker re-detection.
            // alpha=1 → raw (no smoothing); alpha→0 → maximum smoothing + lag.
            if (aruco_first_detection_) {
                aruco_smoothed_x_ = msg->aruco_x_error;
                aruco_smoothed_y_ = msg->aruco_y_error;
                aruco_first_detection_ = false;
            } else {
                constexpr float kAlpha = 0.45f;
                aruco_smoothed_x_ = kAlpha * msg->aruco_x_error
                                  + (1.0f - kAlpha) * aruco_smoothed_x_;
                aruco_smoothed_y_ = kAlpha * msg->aruco_y_error
                                  + (1.0f - kAlpha) * aruco_smoothed_y_;
            }
            fsm_->blackboard_set<float>("aruco_x_error", aruco_smoothed_x_);
            fsm_->blackboard_set<float>("aruco_y_error", aruco_smoothed_y_);
        } else {
            // Reset on loss so the filter seeds fresh on re-acquisition
            aruco_first_detection_ = true;
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

        // Base position cache with spatial deduplication and confidence scoring.
        {
            float* cam_scale_ptr = fsm_->blackboard_get<float>("base_cam_scale");
            float cam_scale = cam_scale_ptr ? *cam_scale_ptr : 0.7f;
            float alt = -static_cast<float>(drone_->getLocalPosition().z());
            float yaw = static_cast<float>(drone_->getOrientation()[2]);
            auto  dpos = drone_->getLocalPosition();

            float* max_err_ptr = fsm_->blackboard_get<float>("base_max_err_radius");
            float max_err_radius = max_err_ptr ? *max_err_ptr : 0.7f;

            float* dedup_ptr = fsm_->blackboard_get<float>("base_dedup_radius");
            float dedup_r = dedup_ptr ? *dedup_ptr : 1.5f;

            // Lock ArUco world position only once the target has been identified,
            // meaning the drone was aligned (errors ≈ 0) → accurate estimate.
            // Updating during the far-away spiral produces large positional errors
            // that would incorrectly mask real base detections.
            if (msg->aruco_detected && !aruco_world_valid_) {
                bool* tc_ptr = fsm_->blackboard_get<bool>("target_calculated");
                if (tc_ptr && *tc_ptr) {
                    float ldx_a = -msg->aruco_y_error * alt * cam_scale;
                    float ldy_a =  msg->aruco_x_error * alt * cam_scale;
                    aruco_world_x_ = static_cast<float>(dpos.x())
                                     + ldx_a * std::cos(yaw) - ldy_a * std::sin(yaw);
                    aruco_world_y_ = static_cast<float>(dpos.y())
                                     + ldx_a * std::sin(yaw) + ldy_a * std::cos(yaw);
                    aruco_world_valid_ = true;
                    // Expõe ao blackboard para que HSearchBaseState acesse
                    fsm_->blackboard_set<float>("aruco_world_x", aruco_world_x_);
                    fsm_->blackboard_set<float>("aruco_world_y", aruco_world_y_);
                    RCLCPP_INFO(this->get_logger(),
                        "[ARUCO_POS] Locked ArUco world pos @ (%.2f, %.2f)",
                        aruco_world_x_, aruco_world_y_);
                }
            }

            float* excl_ptr = fsm_->blackboard_get<float>("aruco_exclusion_radius");
            float aruco_excl_r = excl_ptr ? *excl_ptr : 0.5f;

            auto store_base_pos = [&](const std::string& label, float ex, float ey, float det_conf) {
                if (label.empty()) return;
                if (std::hypot(ex, ey) > max_err_radius) return;

                float ldx = -ey * alt * cam_scale;
                float ldy =  ex * alt * cam_scale;
                float wx  = static_cast<float>(dpos.x())
                            + ldx * std::cos(yaw) - ldy * std::sin(yaw);
                float wy  = static_cast<float>(dpos.y())
                            + ldx * std::sin(yaw) + ldy * std::cos(yaw);
                float r   = std::max(0.5f, std::hypot(ex, ey) * alt * cam_scale + 0.5f);

                // Ignore detections projected near the launch pad origin (false positives
                // during takeoff when the downward camera sees the launch base).
                if (std::hypot(wx, wy) < 0.30f) {
                    RCLCPP_DEBUG(this->get_logger(),
                        "[IGNORE_LAUNCHPAD] %s @ (%.2f, %.2f) — too close to origin",
                        label.c_str(), wx, wy);
                    return;
                }

                // Ignore detections that fall on top of the ArUco marker — the vision
                // node can confuse the marker pattern for a numbered base.
                if (aruco_world_valid_) {
                    float d = std::hypot(wx - aruco_world_x_, wy - aruco_world_y_);
                    if (d < aruco_excl_r) {
                        RCLCPP_DEBUG(this->get_logger(),
                            "[IGNORE_ARUCO] %s @ (%.2f, %.2f) — %.2f m from ArUco (excl=%.2f)",
                            label.c_str(), wx, wy, d, aruco_excl_r);
                        return;
                    }
                }

                // Blended confidence: detection quality × positional reliability
                float pos_q = 1.0f - std::min(1.0f, std::hypot(ex, ey));
                float conf  = det_conf * pos_q;

                // ── Spatial deduplication ───────────────────────────────────
                // Find all known bases within dedup_r of this new detection.
                std::string closest_label;
                float closest_dist = dedup_r + 1.0f;
                std::string weakest_nearby_label;
                float weakest_conf = 1.1f;
                int nearby_count = 0;

                for (const auto& lbl : known_base_labels_) {
                    float* kx = fsm_->blackboard_get<float>("known_base_" + lbl + "_x");
                    float* ky = fsm_->blackboard_get<float>("known_base_" + lbl + "_y");
                    if (!kx || !ky) continue;
                    float dist = std::hypot(wx - *kx, wy - *ky);
                    if (dist < dedup_r) {
                        nearby_count++;
                        if (dist < closest_dist) {
                            closest_dist = dist;
                            closest_label = lbl;
                        }
                        float* kc = fsm_->blackboard_get<float>("known_base_" + lbl + "_conf");
                        float known_c = kc ? *kc : 0.0f;
                        if (known_c < weakest_conf) {
                            weakest_conf = known_c;
                            weakest_nearby_label = lbl;
                        }
                    }
                }

                // Helper: store or update an entry under a specific label key
                auto write_entry = [&](const std::string& key_label) {
                    fsm_->blackboard_set<float>("known_base_" + key_label + "_x", wx);
                    fsm_->blackboard_set<float>("known_base_" + key_label + "_y", wy);
                    fsm_->blackboard_set<float>("known_base_" + key_label + "_r", r);
                    fsm_->blackboard_set<float>("known_base_" + key_label + "_conf", conf);
                    known_base_labels_.insert(key_label);
                };

                if (nearby_count == 0) {
                    // Case A: genuinely new base
                    write_entry(label);
                    std::string discovery = "[DISCOVERY] " + label +
                        " @ (" + std::to_string(wx) + ", " + std::to_string(wy) +
                        ") r=" + std::to_string(r) + " conf=" + std::to_string(conf);
                    RCLCPP_INFO(this->get_logger(), "%s", discovery.c_str());
                    std_msgs::msg::String pub_msg;
                    pub_msg.data = discovery;
                    discovered_bases_pub_->publish(pub_msg);

                } else if (nearby_count == 1) {
                    // Case B: same physical base seen again
                    float* kc = fsm_->blackboard_get<float>("known_base_" + closest_label + "_conf");
                    float known_c = kc ? *kc : 0.0f;

                    if (label == closest_label) {
                        // Same label: update position/confidence if more confident
                        if (conf > known_c) {
                            write_entry(label);
                            RCLCPP_INFO(this->get_logger(),
                                "[UPDATE] %s conf %.2f→%.2f", label.c_str(), known_c, conf);
                        }
                    } else {
                        // Different label at same location: prefer higher confidence label.
                        // Add the new label as a separate entry; don't contaminate old one.
                        if (conf > known_c) {
                            write_entry(label);
                            RCLCPP_INFO(this->get_logger(),
                                "[NEW_CONF] %s (conf %.2f) near %s (conf %.2f) — added as new",
                                label.c_str(), conf, closest_label.c_str(), known_c);
                        }
                    }

                } else {
                    // Case C: multiple nearby — only act if same label with lower confidence
                    if (conf > weakest_conf && label == weakest_nearby_label) {
                        write_entry(label);
                        RCLCPP_INFO(this->get_logger(),
                            "[UPDATE_MULTI] %s conf %.2f→%.2f",
                            label.c_str(), weakest_conf, conf);
                    } else if (conf > weakest_conf && label != weakest_nearby_label) {
                        // New label is more confident than weakest — add it separately
                        write_entry(label);
                        RCLCPP_INFO(this->get_logger(),
                            "[NEW_CONF_MULTI] %s (conf %.2f) added near cluster",
                            label.c_str(), conf);
                    }
                }
            };

            size_t n = msg->visible_bases.size();
            for (size_t i = 0; i < n; ++i) {
                float det_conf = (i < msg->visible_bases_confidence.size())
                    ? msg->visible_bases_confidence[i] : 0.3f;
                store_base_pos(msg->visible_bases[i],
                               msg->visible_bases_x_error[i],
                               msg->visible_bases_y_error[i],
                               det_conf);
            }
            if (msg->target_base_in_sight)
                store_base_pos(msg->target_base,
                               msg->target_base_x_error,
                               msg->target_base_y_error,
                               0.7f);  // target is always confident (exact match)
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

        // ── Base markers for RViz2 — republish every 2s (empty array if no bases yet) ──
        if (pos_log_counter_ % 40 == 0) {
            visualization_msgs::msg::MarkerArray ma;
            std::string* target_label = fsm_->blackboard_get<std::string>("target_base");
            int mid = 0;
            for (const auto& lbl : known_base_labels_) {
                float* kx = fsm_->blackboard_get<float>("known_base_" + lbl + "_x");
                float* ky = fsm_->blackboard_get<float>("known_base_" + lbl + "_y");
                float* kc = fsm_->blackboard_get<float>("known_base_" + lbl + "_conf");
                if (!kx || !ky) continue;
                bool is_target = target_label && (*target_label == lbl);

                // Pick colour by shape prefix
                float cr = 0.5f, cg = 0.5f, cb = 1.0f;  // default blue
                if (lbl.find("TRIANGULO") != std::string::npos) { cr=1.0f; cg=0.2f; cb=0.2f; }
                else if (lbl.find("ESTRELA")   != std::string::npos) { cr=1.0f; cg=0.9f; cb=0.0f; }
                if (is_target) { cr=0.0f; cg=1.0f; cb=0.3f; }  // target = green

                // Sphere at world XY, z slightly above ground (NED→ENU: swap x/y, flip z)
                visualization_msgs::msg::Marker sphere;
                sphere.header.frame_id = "map";
                sphere.header.stamp    = this->now();
                sphere.ns = "base_spheres";
                sphere.id = mid++;
                sphere.type   = visualization_msgs::msg::Marker::SPHERE;
                sphere.action = visualization_msgs::msg::Marker::ADD;
                sphere.pose.position.x = *ky;   // ENU East  = NED y
                sphere.pose.position.y = *kx;   // ENU North = NED x
                sphere.pose.position.z = 0.05f;
                sphere.pose.orientation.w = 1.0;
                sphere.scale.x = sphere.scale.y = sphere.scale.z = is_target ? 0.4f : 0.25f;
                sphere.color.r = cr; sphere.color.g = cg; sphere.color.b = cb;
                sphere.color.a = kc ? (0.4f + *kc * 0.6f) : 0.7f;
                sphere.lifetime = rclcpp::Duration(0, 0);
                ma.markers.push_back(sphere);

                // Text label above the sphere
                visualization_msgs::msg::Marker txt;
                txt.header = sphere.header;
                txt.ns = "base_labels";
                txt.id = mid++;
                txt.type   = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
                txt.action = visualization_msgs::msg::Marker::ADD;
                txt.pose.position.x = sphere.pose.position.x;
                txt.pose.position.y = sphere.pose.position.y;
                txt.pose.position.z = 0.40f;
                txt.pose.orientation.w = 1.0;
                txt.scale.z = 0.20f;
                txt.color.r = 1.0f; txt.color.g = 1.0f; txt.color.b = 1.0f; txt.color.a = 1.0f;
                txt.text = lbl + (kc ? (" c=" + std::to_string(*kc).substr(0,4)) : "");
                txt.lifetime = rclcpp::Duration(0, 0);
                ma.markers.push_back(txt);
            }
            base_markers_pub_->publish(ma);
        }

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

    bool  prev_aruco_detected_       = false;
    bool  prev_target_base_in_sight_ = false;
    bool  prev_target_calculated_    = false;
    float aruco_smoothed_x_          = 0.0f;
    float aruco_smoothed_y_          = 0.0f;
    bool  aruco_first_detection_     = true;
    float aruco_world_x_             = 0.0f;
    float aruco_world_y_             = 0.0f;
    bool  aruco_world_valid_         = false;
    int  pos_log_counter_           = 0;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr discovered_bases_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr base_markers_pub_;
    nav_msgs::msg::Path trajectory_;
    std::set<std::string> known_base_labels_;
    int marker_seq_ = 0;
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

    executor.add_node(mission_node);

    executor.spin();

    rclcpp::shutdown();
    return 0;
}
