// RB850.cpp

#include "RB/RB850.hpp"
#include <iostream>
#include <stdexcept>
#include <chrono>
#include <thread>

// -----------------------------------------------------------------
// 1. 좌표 변환 함수 구현 (Static, 유지)
// -----------------------------------------------------------------
Eigen::Vector4f RB850::cam_to_robot(const Eigen::Vector4f &camera_centroid)
{
    // ... (로직 유지) ...
    Eigen::Vector4f robot_centroid = Eigen::Vector4f::Zero();
    robot_centroid[0] = camera_centroid[1] + T_X_OFFSET;
    robot_centroid[1] = camera_centroid[0] - T_Y_OFFSET;
    robot_centroid[2] = -camera_centroid[2] + T_Z_OFFSET;
    robot_centroid[3] = 1.0; 
    std::cout << "[TRANSFORM] RealSense Centroid (m): X=" << camera_centroid[0] << ", Y=" << camera_centroid[1] << ", Z=" << camera_centroid[2] << std::endl;
    std::cout << "[TRANSFORM] Robot Base Centroid (m) (ESTIMATED): X=" << robot_centroid[0] << ", Y=" << robot_centroid[1] << ", Z=" << robot_centroid[2] << std::endl;
    return robot_centroid;
}

// -----------------------------------------------------------------
// 2. 툴 제어 함수 구현 (Static, robot/rc 인수로 변경)
// -----------------------------------------------------------------
void RB850::gripper_open(RBCobot &robot, ResponseCollector &rc, int delay_ms)
{
    std::cout << "[GRIPPER] Opening Gripper (Tool Signal 0 = 1)..." << std::endl;

    // 🌟 robot, rc 인수로 변경
    robot.set_tool_out(rc, 24, 1, 0);
    robot.flush(rc);
    rc.error().throw_if_not_empty();

    std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
}

void RB850::gripper_close(RBCobot &robot, ResponseCollector &rc, int delay_ms)
{
    std::cout << "[GRIPPER] Closing Gripper (Tool Signal 0 = 0)..." << std::endl;

    // 🌟 robot, rc 인수로 변경
    robot.set_tool_out(rc, 24, 0, 1);
    robot.flush(rc);
    rc.error().throw_if_not_empty();

    std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
}

// -----------------------------------------------------------------
// 3. 모션 제어 함수 구현 (Static, robot/rc 인수로 변경)
// -----------------------------------------------------------------
void RB850::moveL(RBCobot &robot, ResponseCollector &rc, const RobotPose &pose, double speed, double acc, const std::string &msg)
{
    std::cout << "[MOVE] " << msg << " - Pose: X=" << pose[0] << ", Y=" << pose[1] << ", Z=" << pose[2] << " (mm)" << std::endl;

    // 🌟 robot, rc 인수로 변경
    robot.move_l(rc, pose, speed, acc);
    robot.flush(rc);
    rc.error().throw_if_not_empty();

    if (!robot.wait_for_move_started(rc, 1.0).is_success())
    {
        rc.error().throw_if_not_empty();
        throw std::runtime_error("[MOVE FAIL] " + msg + " : 모션 시작 실패 (타임아웃/오류).");
    }

    robot.wait_for_move_finished(rc);
    rc.error().throw_if_not_empty();

    std::cout << "[MOVE OK] " << msg << " 완료." << std::endl;
}

void RB850::moveJ(RBCobot &robot, ResponseCollector &rc, const RobotPose &joint_angles, double speed, double acc, const std::string &msg)
{
    std::cout << "[MOVEJ] " << msg << " - Joint Angles (deg): J1=" << joint_angles[0]
              << ", J2=" << joint_angles[1] << ", J3=" << joint_angles[2]
              << ", J4=" << joint_angles[3] << ", J5=" << joint_angles[4]
              << ", J6=" << joint_angles[5] << std::endl;

    // 🌟 robot, rc 인수로 변경
    robot.move_j(rc, joint_angles, speed, acc);
    robot.flush(rc);
    rc.error().throw_if_not_empty();

    if (!robot.wait_for_move_started(rc, 1.0).is_success())
    {
        rc.error().throw_if_not_empty();
        throw std::runtime_error("[MOVEJ FAIL] " + msg + " : 모션 시작 실패 (타임아웃/오류).");
    }

    robot.wait_for_move_finished(rc);
    rc.error().throw_if_not_empty();

    std::cout << "[MOVEJ OK] " << msg << " 완료." << std::endl;
}