#pragma once

#include <memory>
#include <string>
#include <cmath>

#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "drone/movement.hpp"

/**
 * Searches for the ArUco marker using a square (outward) spiral pattern.
 *
 * Movement strategy (mirrors SearchBaseState):
 *   - Starts at the local origin (0, 0) at the current altitude.
 *   - Legs grow in pairs: two legs of length L, then L += step_size_.
 *   - Directions cycle: +X, +Y, -X, -Y (right → up → left → down).
 *
 * Detection uses a persistence + miss-tolerance filter inherited from the
 * previous Archimedean implementation:
 *   - Requires `aruco_persistence_frames` consecutive detections of the SAME
 *     ArUco ID before transitioning.
 *   - Tolerates up to `max_misses_` (3) consecutive non-detections before
 *     the counter is reset, preventing false resets on single dropped frames.
 *
 * Blackboard reads:
 *   "aruco_spiral_step"        (float) — leg growth per pair of legs (default 2.5 m)
 *   "search_aruco_velocity"    (float) — movement speed m/s (default 0.5)
 *   "aruco_persistence_frames" (float) — frames needed to confirm ArUco (default 3)
 *
 * Returns: "ARUCO_FOUND"
 */
class SearchArucoState : public fsm::State {
public:
    SearchArucoState() : fsm::State(),
        detection_counter_(0), miss_counter_(0), last_detected_id_(-1),
        min_detections_(3), max_misses_(3),
        current_leg_length_(0.0f), current_leg_(0), steps_in_current_length_(0) {}

    void on_enter(fsm::Blackboard &blackboard) override {
        drone_ = *blackboard.get<std::shared_ptr<Drone>>("drone");
        if (drone_ == nullptr) return;

        drone_->log("");
        drone_->log("STATE: SEARCH_ARUCO (Square Spiral)");

        step_size_ = 2.5f;
        if (blackboard.contains("aruco_spiral_step"))
            step_size_ = *blackboard.get<float>("aruco_spiral_step");

        velocity_ = 0.5f;
        if (blackboard.contains("search_aruco_velocity"))
            velocity_ = *blackboard.get<float>("search_aruco_velocity");

        if (blackboard.contains("aruco_persistence_frames"))
            min_detections_ = static_cast<int>(*blackboard.get<float>("aruco_persistence_frames"));

        detection_counter_ = 0;
        miss_counter_       = 0;
        last_detected_id_   = -1;

        // Use z_max_search from config — not the current drone altitude, which may have
        // drifted during GO_TO_ARUCO miss ticks before transitioning back here.
        center_z_ = blackboard.contains("z_max_search")
            ? *blackboard.get<float>("z_max_search")
            : static_cast<float>(drone_->getLocalPosition().z());

        // Square spiral starts at the local origin.
        target_pos_ = Eigen::Vector3d(0.0, 0.0, center_z_);

        current_leg_length_      = step_size_;
        current_leg_             = 0;
        steps_in_current_length_ = 0;

        // Cardinal directions: right, forward, left, back (NED: +X=N, +Y=E)
        directions_[0] = Eigen::Vector2d( 1.0,  0.0);
        directions_[1] = Eigen::Vector2d( 0.0,  1.0);
        directions_[2] = Eigen::Vector2d(-1.0,  0.0);
        directions_[3] = Eigen::Vector2d( 0.0, -1.0);

        update_target();
    }

    std::string act(fsm::Blackboard &blackboard) override {
        if (drone_ == nullptr) return "ERROR";

        // ── Persistence filter with miss tolerance ─────────────────────────
        bool is_detected = false;
        if (blackboard.contains("aruco_detected"))
            is_detected = *blackboard.get<bool>("aruco_detected");

        if (is_detected) {
            miss_counter_ = 0;
            int current_id = 0;
            if (blackboard.contains("aruco_id"))
                current_id = *blackboard.get<int>("aruco_id");

            if (current_id == last_detected_id_) {
                detection_counter_++;
            } else {
                detection_counter_ = 1;
                last_detected_id_  = current_id;
            }

            if (detection_counter_ >= min_detections_) {
                blackboard.set<int>("confirmed_aruco_id", last_detected_id_);
                move_local_by_waypoint(drone_, drone_->getLocalPosition(), 0.0f);
                drone_->log("ArUco ID " + std::to_string(last_detected_id_) +
                            " confirmed after " + std::to_string(detection_counter_) +
                            " frames. Transitioning.");
                return "ARUCO_FOUND";
            }
        } else {
            // Only reset after max_misses_ consecutive non-detections to
            // tolerate single dropped camera frames (~100 ms at 10 Hz).
            miss_counter_++;
            if (miss_counter_ >= max_misses_) {
                detection_counter_ = 0;
                last_detected_id_  = -1;
            }
        }

        // ── Square spiral movement ─────────────────────────────────────────
        Eigen::Vector3d cur = drone_->getLocalPosition();
        float dist = std::hypot(cur.x() - target_pos_.x(),
                                cur.y() - target_pos_.y());

        if (dist < 0.15f) {
            current_leg_ = (current_leg_ + 1) % 4;
            steps_in_current_length_++;

            if (steps_in_current_length_ >= 2) {
                current_leg_length_      += step_size_;
                steps_in_current_length_  = 0;
            }

            update_target();
        }

        move_local_by_waypoint(drone_, target_pos_, velocity_);
        return "";
    }

private:
    void update_target() {
        target_pos_.x() += directions_[current_leg_].x() * current_leg_length_;
        target_pos_.y() += directions_[current_leg_].y() * current_leg_length_;
        target_pos_.z()  = center_z_;
    }

    std::shared_ptr<Drone> drone_;
    Eigen::Vector3d target_pos_;

    float step_size_;
    float velocity_;
    float center_z_;

    // Persistence filter
    int detection_counter_;
    int miss_counter_;
    int last_detected_id_;
    int min_detections_;
    int max_misses_;

    // Square spiral state
    float              current_leg_length_;
    int                current_leg_;
    int                steps_in_current_length_;
    Eigen::Vector2d    directions_[4];
};
