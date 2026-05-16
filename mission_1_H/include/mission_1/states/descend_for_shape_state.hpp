#pragma once

#include <memory>
#include <string>

#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "drone/movement.hpp"

/**
 * Entered when the drone is centred over the ArUco but the surrounding shape
 * (gabarito) could not yet be identified at the current altitude.
 *
 * Sequence:
 *   1. DESCEND  — move straight down to shape_id_altitude while holding XY.
 *   2. WAIT     — hover and wait until the CV node sets target_calculated=true.
 *   3. ASCEND   — return to the original altitude before handing control back.
 *
 * Blackboard reads:
 *   "shape_id_altitude"  (float) — NED z to descend to (default -1.0 m, i.e. 1 m AGL)
 *   "shape_id_velocity"  (float) — descent/ascent speed (default 0.3 m/s)
 *   "position_tolerance" (float) — waypoint acceptance radius
 *   "aruco_detected"     (bool)  — abort if lost for too long
 *   "target_calculated"  (bool)  — shape has been identified
 *
 * Returns:
 *   "SHAPE_FOUND"  — shape identified, drone back at original altitude
 *   "ARUCO_LOST"   — ArUco disappeared during the manoeuvre
 *   "ERROR"
 */
class DescendForShapeState : public fsm::State {
public:
    DescendForShapeState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {
        drone_ = *blackboard.get<std::shared_ptr<Drone>>("drone");
        if (!drone_) return;

        drone_->log("");
        drone_->log("STATE: DESCEND_FOR_SHAPE");

        shape_id_altitude_ = -1.0f;
        if (blackboard.contains("shape_id_altitude"))
            shape_id_altitude_ = *blackboard.get<float>("shape_id_altitude");

        velocity_ = 0.3f;
        if (blackboard.contains("shape_id_velocity"))
            velocity_ = *blackboard.get<float>("shape_id_velocity");

        tolerance_ = 0.15f;
        if (blackboard.contains("position_tolerance"))
            tolerance_ = *blackboard.get<float>("position_tolerance");

        Eigen::Vector3d pos = drone_->getLocalPosition();
        hold_x_          = static_cast<float>(pos.x());
        hold_y_          = static_cast<float>(pos.y());
        return_altitude_ = static_cast<float>(pos.z());

        drone_->log("Descending to z=" + std::to_string(shape_id_altitude_)
                    + " for shape ID (will return to z=" + std::to_string(return_altitude_) + ")");

        phase_        = Phase::DESCEND;
        miss_counter_ = 0;
        descend_mover_.reset();
    }

    std::string act(fsm::Blackboard &blackboard) override {
        if (!drone_) return "ERROR";

        // Abort if ArUco disappears for too long
        bool aruco_ok = false;
        if (blackboard.contains("aruco_detected"))
            aruco_ok = *blackboard.get<bool>("aruco_detected");

        if (!aruco_ok) {
            if (++miss_counter_ > 10) {
                drone_->log("ArUco lost during shape ID descent. Aborting.");
                return "ARUCO_LOST";
            }
        } else {
            miss_counter_ = 0;
        }

        switch (phase_) {

        case Phase::DESCEND: {
            Eigen::Vector3d tgt(hold_x_, hold_y_, shape_id_altitude_);
            if (descend_mover_.update(drone_, tgt, velocity_, 0.5f, tolerance_)) {
                drone_->log("At shape ID altitude. Waiting for shape detection...");
                phase_ = Phase::WAIT;
            }
            break;
        }

        case Phase::WAIT: {
            Eigen::Vector3d tgt(hold_x_, hold_y_, shape_id_altitude_);
            move_local_constant_step(drone_, tgt, 0.0f, tolerance_);

            bool shape_ok = false;
            if (blackboard.contains("target_calculated"))
                shape_ok = *blackboard.get<bool>("target_calculated");

            if (shape_ok) {
                drone_->log("Shape identified! Target: "
                            + *blackboard.get<std::string>("target_base")
                            + ". Ascending back.");
                ascend_mover_.reset();
                phase_ = Phase::ASCEND;
            }
            break;
        }

        case Phase::ASCEND: {
            Eigen::Vector3d tgt(hold_x_, hold_y_, return_altitude_);
            if (ascend_mover_.update(drone_, tgt, velocity_, 0.5f, tolerance_)) {
                drone_->log("Back at search altitude. Shape confirmed.");
                return "SHAPE_FOUND";
            }
            break;
        }
        }

        return "";
    }

private:
    enum class Phase { DESCEND, WAIT, ASCEND };

    std::shared_ptr<Drone> drone_;
    float shape_id_altitude_;
    float velocity_;
    float tolerance_;
    float hold_x_, hold_y_;
    float return_altitude_;
    Phase phase_;
    int   miss_counter_;
    TrapezoidalMover descend_mover_;
    TrapezoidalMover ascend_mover_;
};
