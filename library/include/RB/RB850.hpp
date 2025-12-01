#pragma once

#include "rbpodo.hpp" // 로봇 SDK 헤더 파일 포함
#include <iostream>
#include <chrono>
#include <thread>
#include <stdexcept>
#include <Eigen/Dense>

using RBCobot = rb::podo::Cobot<rb::podo::StandardVector>;
using ResponseCollector = rb::podo::ResponseCollector;
using RobotPose = RBCobot::Point;

#define T_X_OFFSET 0.0  
#define T_Y_OFFSET 0.845  
#define T_Z_OFFSET 0.99

class RB850
{
public:
    static void moveL(RBCobot &robot, ResponseCollector &rc, const RobotPose &pose, double speed, double acc, const std::string &msg);
    static void moveJ(RBCobot &robot, ResponseCollector &rc, const RobotPose &joint_angles, double speed, double acc, const std::string &msg);
    
    static Eigen::Vector4f cam_to_robot(const Eigen::Vector4f &camera_centroid);

    static void gripper_open(RBCobot &robot, ResponseCollector &rc, int delay_ms);
    static void gripper_close(RBCobot &robot, ResponseCollector &rc, int delay_ms);
    
};