#include <iostream>
#include <array>
#include <chrono>
#include <thread>
#include <cmath>
#include <filesystem>
#include <mutex>
#include <atomic>
#include <stdexcept>
#include <algorithm>
#include <future>
#include <vector>
#include <map>

// RealSense
#include <librealsense2/rs.hpp>

// PCL
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/centroid.h>
#include <pcl/common/pca.h>
#include <pcl/common/common.h>

// Eigen
#include <Eigen/Dense>

// OpenCV
#include <opencv2/opencv.hpp>

// Robot SDK
#include "rbpodo.hpp"
#include "error_code.hpp"
#include "RB/RB850.hpp"

// HTTP Client
#include "client.h"
#include <curl/curl.h>

using namespace rb;
using namespace rb::podo;
namespace fs = std::filesystem;

// =========================================================================
// 1. 타입 별칭 및 상수 정의
// =========================================================================

using RBCobot = rb::podo::Cobot<rb::podo::StandardVector>;
using RobotPose = RBCobot::Point;
using PointCloudPtr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

namespace Timing
{
    using Clock = std::chrono::high_resolution_clock;
    using TimePoint = Clock::time_point;
    using DurationMs = std::chrono::duration<double, std::milli>;
}
struct StepTiming
{
    std::string description;
    Timing::DurationMs duration;
};

namespace VisionParams
{
    const int ROI_X = 260;
    const int ROI_Y = 170;
    const int ROI_WIDTH = 150;
    const int ROI_HEIGHT = 180;

    const float Z_MIN = 0.2f;
    const float Z_MAX = 3.0f;
    const float X_MIN = -0.2f;
    const float X_MAX = 0.2f;
    const float Y_MIN = -0.2f;
    const float Y_MAX = 0.2f;
    const float VOXEL_LEAF_SIZE = 0.005f;

    const double RANSAC_DISTANCE_THRESHOLD = 0.015;
}

// --- 로봇 동작 상수 ---
const double APPROACH_HEIGHT_M = 0.100;
const double MIN_LIFT_HEIGHT_M = 0.300;
const double SPEED_APPROACH = 1500.0;
const double ACC_APPROACH = 1500.0;
const double SPEED_GRASP = 500.0;
const double ACC_GRASP = 500.0;

// --- 전역 상태 및 데이터 ---
std::mutex cloud_mutex;
std::atomic<int> processing_step(0);
std::atomic<bool> running(true);
std::atomic<bool> pick_trigger(false);

PointCloudPtr current_cloud(new pcl::PointCloud<pcl::PointXYZ>);
PointCloudPtr processed_cloud(new pcl::PointCloud<pcl::PointXYZ>);

Eigen::Vector4f g_centroid = Eigen::Vector4f::Zero();
double g_grasp_yaw_deg = 0.0;
Client g_detection_client("http://0.0.0.0:0/predict");
std::string g_detected_object_name = "unknown";

long long g_time_step1 = 0;
long long g_time_step2 = 0;
long long g_time_step3 = 0;

const std::array<double, 6> home_q = {-70, 0, 90, 0, 90, 0};

// =========================================================================
// 2. 로봇 제어 함수
// =========================================================================
void performRobotPickAndPlace(RBCobot &robot, rb::podo::ResponseCollector &rc,
                              const Eigen::Vector4f &centroid, double grasp_yaw_deg,
                              const std::string &object_name)
{
    double target_x_m = centroid[0];
    double target_y_m = centroid[1];
    double target_z_m = centroid[2];

    if (target_z_m < -0.2 || target_z_m > 1.5)
    {
        std::cerr << "[WARN] 객체 Z 좌표(" << target_z_m << "m)가 비정상적입니다. 임의 값으로 강제 보정합니다." << std::endl;
        target_z_m = 0.20;
    }

    double target_x_mm = target_x_m * 1000.0;
    double target_y_mm = target_y_m * 1000.0;
    double grasp_z_mm = (target_z_m) * 1000.0;
    double approach_z_mm = (target_z_m + APPROACH_HEIGHT_M) * 1000.0;
    double lift_z_mm = std::max(MIN_LIFT_HEIGHT_M * 1000.0, target_z_m * 1000.0 + APPROACH_HEIGHT_M * 1000.0);

    RobotPose approach_pose = {target_x_mm, target_y_mm, approach_z_mm, 90.0, 0.0, grasp_yaw_deg};
    RobotPose grasp_pose = {target_x_mm, target_y_mm, grasp_z_mm, 90.0, 0.0, grasp_yaw_deg};
    RobotPose lift_pose = {target_x_mm, target_y_mm, lift_z_mm, 90.0, 0.0, grasp_yaw_deg};
    RobotPose safe_start_pose = {target_x_mm, target_y_mm, lift_z_mm, 90.0, 0.0, 0.0};

    // --- 시간 측정용 변수 선언 ---
    Timing::TimePoint start_time;
    Timing::TimePoint end_time;

    // ⭐ 모든 단계별 시간을 순서대로 저장할 벡터 ⭐
    std::vector<StepTiming> timings_log;

    std::cout << "\n--- ROBOT PICK SEQUENCE START ---" << std::endl;

    // P1: 안전한 시작 위치로 이동
    start_time = Timing::Clock::now();
    RB850::moveL(robot, rc, safe_start_pose, SPEED_APPROACH, ACC_APPROACH, "안전한 시작 위치로 이동");
    end_time = Timing::Clock::now();
    timings_log.push_back({"P1: 안전한 시작 위치로 이동", end_time - start_time});

    // P2: Approach Position으로 직선 이동
    start_time = Timing::Clock::now();
    RB850::moveL(robot, rc, approach_pose, SPEED_APPROACH, ACC_APPROACH, "Approach Position으로 직선 이동");
    end_time = Timing::Clock::now();
    timings_log.push_back({"P2: Approach Position으로 직선 이동", end_time - start_time});

    // P3: Grasp Position으로 직선 하강
    start_time = Timing::Clock::now();
    RB850::moveL(robot, rc, grasp_pose, SPEED_GRASP, ACC_GRASP, "Grasp Position으로 직선 하강");
    end_time = Timing::Clock::now();
    timings_log.push_back({"P3: Grasp Position으로 직선 하강", end_time - start_time});

    // P4: 그리퍼 닫기
    start_time = Timing::Clock::now();
    RB850::gripper_close(robot, rc, 4000);
    end_time = Timing::Clock::now();
    timings_log.push_back({"P4: 그리퍼 닫기 (4000ms 대기)", end_time - start_time});

    // P5: Lift Position으로 직선 상승
    start_time = Timing::Clock::now();
    RB850::moveL(robot, rc, lift_pose, SPEED_APPROACH, ACC_APPROACH, "Lift Position으로 직선 상승");
    end_time = Timing::Clock::now();
    timings_log.push_back({"P5: Lift Position으로 직선 상승", end_time - start_time});

    std::cout << "--- PICK SEQUENCE 완료 ---" << std::endl;
    std::map<std::string, std::pair<double, double>> place_coords = {
        {"banana", {200.0, -350.0}},    // 바나나
        {"egg-plant", {100.0, -350.0}}, // 가지
        {"carrot", {0.0, -350.0}},      // 당근
        {"peach", {-100.0, -350.0}},    // 복숭아
        {"mushroom", {-200.0, -350.0}}, // 버섯
        {"unknown", {0.0, -500.0}}      // Unknown
    };

    double BOX_X_MM, BOX_Y_MM;
    auto it = place_coords.find(object_name);
    if (it != place_coords.end())
    {
        BOX_X_MM = it->second.first;
        BOX_Y_MM = it->second.second;
    }
    else
    {
        BOX_X_MM = place_coords["unknown"].first;
        BOX_Y_MM = place_coords["unknown"].second;
        std::cout << "[PLACE INFO] Object '" << object_name << "' not found. Using default coords." << std::endl;
    }

    const double BOX_Z_MM = 205.0;
    const double PLACE_Z_OFFSET_MM = 100.0;

    RobotPose place_approach_pose = {BOX_X_MM, BOX_Y_MM, BOX_Z_MM + PLACE_Z_OFFSET_MM, 90.0, 0.0, 90.0};
    RobotPose place_pose = {BOX_X_MM, BOX_Y_MM, BOX_Z_MM, 90.0, 0.0, 90.0};

    std::cout << "\n--- ⭐ ROBOT PLACE SEQUENCE START ⭐ ---" << std::endl;

    // L1: Place Approach Position으로 직선 이동
    start_time = Timing::Clock::now();
    RB850::moveL(robot, rc, place_approach_pose, SPEED_APPROACH, ACC_APPROACH, "Place Approach Position으로 직선 이동");
    end_time = Timing::Clock::now();
    timings_log.push_back({"L1: Place Approach Position으로 직선 이동", end_time - start_time});

    // L2: Place Position으로 직선 하강
    start_time = Timing::Clock::now();
    RB850::moveL(robot, rc, place_pose, SPEED_GRASP, ACC_GRASP, "Place Position으로 직선 하강");
    end_time = Timing::Clock::now();
    timings_log.push_back({"L2: Place Position으로 직선 하강", end_time - start_time});

    // L3: 그리퍼 열기
    start_time = Timing::Clock::now();
    RB850::gripper_open(robot, rc, 2000);
    end_time = Timing::Clock::now();
    timings_log.push_back({"L3: 그리퍼 열기 (2000ms 대기)", end_time - start_time});

    // L4: Place Approach Position으로 직선 상승
    start_time = Timing::Clock::now();
    RB850::moveL(robot, rc, place_approach_pose, SPEED_APPROACH, ACC_APPROACH, "Place Approach Position으로 직선 상승");
    end_time = Timing::Clock::now();
    timings_log.push_back({"L4: Place Approach Position으로 직선 상승", end_time - start_time});

    std::cout << "--- PLACE SEQUENCE 완료 ---" << std::endl;

    // ----------------------------------------------------------------------
    // --- ⭐ 최종 결과 출력 (전체 동작 완료 후) ⭐ ---
    // ----------------------------------------------------------------------

    std::cout << "\n\n=========================================================" << std::endl;
    std::cout << "✅ 로봇 PICK & PLACE 전체 단계별 소요 시간" << std::endl;
    std::cout << "=========================================================" << std::endl;

    // 로그에 기록된 모든 단계 시간을 순서대로 출력
    for (size_t i = 0; i < timings_log.size(); ++i)
    {
        const auto &step = timings_log[i];
        std::cout << "[" << i + 1 << "] " << step.description << ": "
                  << step.duration.count() << "ms" << std::endl;
    }
    std::cout << "=========================================================" << std::endl;
}

// =========================================================================
// 3. PCL/RealSense/Server 함수
// =========================================================================
void keyboardCallback(const pcl::visualization::KeyboardEvent &event, void *)
{
    if (event.keyDown())
    {
        if (event.getKeySym() == "0")
        {
            std::cout << "\n[KEY] '0' pressed -> Starting Step 7: Go to Home Position 🏠" << std::endl;
            processing_step = 7;
        }
        else if (event.getKeySym() == "1")
        {
            processing_step = 1;
        }
        else if (event.getKeySym() == "2")
        {
            if (current_cloud->size() > 100)
                processing_step = 2;
            else
                std::cerr << "[WARN] 유효한 포인트 클라우드 데이터를 기다리는 중입니다. 다시 시도하십시오." << std::endl;
        }
        else if (event.getKeySym() == "3")
        {
            if (processed_cloud->size() > 10)
                processing_step = 3;
            else
                std::cerr << "[WARN] 평면 제거(키 '2')를 먼저 수행하여 객체 클라우드를 분리해야 합니다." << std::endl;
        }
        else if (event.getKeySym() == "4")
        {
            if (g_centroid.allFinite() && g_centroid[2] > -0.1 && g_detected_object_name != "unknown")
            {
                std::cout << "\n[KEY] '4' pressed -> Starting Step 4: Robot Pick and Place Sequence 🤖" << std::endl;
                pick_trigger = true;
            }
            else
                std::cerr << "[WARN] PCA 분석(키 '3')과 물체 인식(키 '1')이 모두 완료되어야 합니다." << std::endl;
        }
        else if (event.getKeySym() == "5")
            processing_step = 5;
    }
}

PointCloudPtr convertToPCL(const rs2::points &points)
{
    auto sp = points.get_vertices();
    PointCloudPtr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->points.reserve(points.size()); // **최적화: 메모리 미리 할당**
    for (int i = 0; i < points.size(); ++i)
    {
        if (sp[i].z)
        {
            cloud->points.emplace_back(sp[i].x, sp[i].y, sp[i].z);
        }
    }
    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = false;
    return cloud;
}

PointCloudPtr filterPointCloud(const PointCloudPtr &input)
{
    PointCloudPtr filtered(new pcl::PointCloud<pcl::PointXYZ>);
    PointCloudPtr temp_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    // 1. Z축 필터링 (깊이) - VisionParams 사용
    pcl::PassThrough<pcl::PointXYZ> pass_z;
    pass_z.setInputCloud(input);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(VisionParams::Z_MIN, VisionParams::Z_MAX);
    pass_z.filter(*temp_filtered);

    // 2. X축 필터링 (작업 영역) - VisionParams 사용
    pcl::PassThrough<pcl::PointXYZ> pass_x;
    pass_x.setInputCloud(temp_filtered);
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(VisionParams::X_MIN, VisionParams::X_MAX);
    pass_x.filter(*temp_filtered);

    // 3. Y축 필터링 (작업 영역) - VisionParams 사용
    pcl::PassThrough<pcl::PointXYZ> pass_y;
    pass_y.setInputCloud(temp_filtered);
    pass_y.setFilterFieldName("y");
    pass_y.setFilterLimits(VisionParams::Y_MIN, VisionParams::Y_MAX);
    pass_y.filter(*temp_filtered);

    // 4. Voxel Grid 다운샘플링 - VisionParams 사용
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(temp_filtered);
    voxel.setLeafSize(VisionParams::VOXEL_LEAF_SIZE, VisionParams::VOXEL_LEAF_SIZE, VisionParams::VOXEL_LEAF_SIZE);
    voxel.filter(*filtered);

    return filtered;
}

PointCloudPtr removeFloorPlane(const PointCloudPtr &input,
                               pcl::visualization::PCLVisualizer::Ptr &viewer)
{
    for (int i = 0; i < 3; ++i)
    {
        viewer->removeShape("pca_arrow_" + std::to_string(i));
    }

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(VisionParams::RANSAC_DISTANCE_THRESHOLD);
    seg.setInputCloud(input);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.empty())
    {
        std::cerr << "[ERROR] 바닥 평면을 찾을 수 없습니다! 원본 클라우드를 반환합니다." << std::endl;
        return input;
    }

    pcl::ExtractIndices<pcl::PointXYZ> extract_floor;
    extract_floor.setInputCloud(input);
    extract_floor.setIndices(inliers);
    extract_floor.setNegative(false);
    PointCloudPtr floor_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    extract_floor.filter(*floor_cloud);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> floor_color_handler(floor_cloud, 255, 0, 0); // RGB: Red
    viewer->removePointCloud("floor_cloud");
    viewer->addPointCloud<pcl::PointXYZ>(floor_cloud, floor_color_handler, "floor_cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "floor_cloud");

    pcl::ExtractIndices<pcl::PointXYZ> extract_object;
    extract_object.setInputCloud(input);
    extract_object.setIndices(inliers);
    extract_object.setNegative(true);
    PointCloudPtr no_floor(new pcl::PointCloud<pcl::PointXYZ>);
    extract_object.filter(*no_floor);

    std::cout << "[INFO] 바닥 평면 제거 완료. " << inliers->indices.size() << "개 포인트 제거됨. 남은 포인트: " << no_floor->size() << std::endl;

    return no_floor;
}

void performPCAAndVisualize(pcl::visualization::PCLVisualizer::Ptr &viewer,
                            const PointCloudPtr &cloud,
                            const std::string &arrow_prefix = "pca_arrow")
{
    if (cloud->empty() || cloud->size() < 10)
    {
        std::cerr << "[WARN] PCA 시도했으나 유효한 객체 포인트가 너무 적습니다.\n";
        g_centroid = Eigen::Vector4f::Zero();
        return;
    }

    Eigen::Vector4f centroid_cam;
    pcl::compute3DCentroid(*cloud, centroid_cam);

    Eigen::Matrix3f covariance;
    pcl::computeCovarianceMatrixNormalized(*cloud, centroid_cam, covariance);

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eig(covariance);
    Eigen::Matrix3f eig_vectors = eig.eigenvectors();
    Eigen::Vector3f eig_values = eig.eigenvalues();

    int major_axis_index = 0;
    if (eig_values[1] > eig_values[0])
        major_axis_index = 1;
    if (eig_values[2] > eig_values[major_axis_index])
        major_axis_index = 2;

    Eigen::Vector3f major_axis = eig_vectors.col(major_axis_index);

    Eigen::Vector2f projected_dir(major_axis[0], major_axis[1]);
    projected_dir.normalize();

    float yaw_rad = std::atan2(projected_dir[0], projected_dir[1]);
    float yaw_deg = yaw_rad * 180.0 / M_PI;

    Eigen::Vector4f centroid_cam_aligned;
    centroid_cam_aligned << centroid_cam[0], centroid_cam[1], centroid_cam[2], 1.0;
    Eigen::Vector4f centroid_robot = RB850::cam_to_robot(centroid_cam_aligned);

    g_centroid = centroid_robot;
    g_grasp_yaw_deg = yaw_deg;
    pick_trigger = false;

    float scale = 0.1;
    Eigen::Vector3f center(centroid_cam[0], centroid_cam[1], centroid_cam[2]);

    for (int i = 0; i < 3; ++i)
    {
        viewer->removeShape(arrow_prefix + "_" + std::to_string(i));
        Eigen::Vector3f dir = eig_vectors.col(2 - i).normalized();

        pcl::PointXYZ pt1(center[0], center[1], center[2]);
        pcl::PointXYZ pt2(center[0] + scale * dir[0], center[1] + scale * dir[1], center[2] + scale * dir[2]);

        std::string name = arrow_prefix + "_" + std::to_string(i);
        float r = (i == 0) ? 1.0 : 0.5;
        float g = (i == 1) ? 1.0 : 0.5;
        float b = (i == 2) ? 1.0 : 0.5;
        viewer->addArrow(pt2, pt1, r, g, b, false, name);
    }

    std::cout << "\n============================" << std::endl;
    std::cout << "[ROBOT] Grasp Target Info (PCA)" << std::endl;
    std::cout << "Robot Base Centroid (m): X=" << centroid_robot[0] << ", Y=" << centroid_robot[1] << ", Z=" << centroid_robot[2] << std::endl;
    std::cout << "[ROBOT] Object Orientation (Yaw): " << yaw_deg << " degrees" << std::endl;
    std::cout << "============================\n"
              << std::endl;
}

std::string parseObjectNameFromResponse(const std::string &response)
{
    size_t start_key = response.find("\"class_name\":");
    if (start_key == std::string::npos)
        return "unknown";
    size_t start_quote = response.find("\"", start_key + std::string("\"class_name\":").length()); // 길이 오프셋 대략 계산
    if (start_quote == std::string::npos)
        return "unknown";
    size_t end_quote = response.find("\"", start_quote + 1);
    if (end_quote == std::string::npos)
        return "unknown";

    return response.substr(start_quote + 1, end_quote - start_quote - 1);
}

void captureAndSendRGB(rs2::frameset &frames)
{
    rs2::video_frame color_frame = frames.get_color_frame();
    if (!color_frame)
    {
        std::cerr << "[RGB CAPTURE] Color frame not available." << std::endl;
        return;
    }

    cv::Mat bgr_mat(cv::Size(color_frame.get_width(), color_frame.get_height()),
                    CV_8UC3, (void *)color_frame.get_data(), cv::Mat::AUTO_STEP);

    // 2. x, y축으로 불필요한 부분 잘라내기 (ROI 설정) - VisionParams 사용
    cv::Rect roi(VisionParams::ROI_X, VisionParams::ROI_Y, VisionParams::ROI_WIDTH, VisionParams::ROI_HEIGHT);

    if (roi.x + roi.width > bgr_mat.cols || roi.y + roi.height > bgr_mat.rows)
    {
        std::cerr << "[RGB CAPTURE] ROI 설정이 이미지 크기를 초과합니다. 조정이 필요합니다." << std::endl;
        return;
    }

    cv::Mat cropped_image = bgr_mat(roi).clone();

    // 2-1. 저장 경로
    std::filesystem::path save_path = "D:/manipulator/DATA/RGB/captured_rgb.jpg";
    try
    {
        std::filesystem::create_directories(save_path.parent_path());
    }
    catch (const fs::filesystem_error &e)
    {
        std::cerr << "[FS ERROR] Could not create directory: " << e.what() << std::endl;
        return;
    }

    std::vector<int> compression_params;
    compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
    compression_params.push_back(80);
    cv::imwrite(save_path.string(), cropped_image, compression_params);
    std::cout << "[RGB CAPTURE] Cropped RGB image saved to: " << save_path << std::endl;

    // 3. 저장한 것을 서버로 보낸다.
    std::cout << "[SERVER] Sending image to detection server..." << std::endl;
    std::string response = g_detection_client.sendImageForDetection(save_path.string());

    if (!response.empty())
    {
        g_detected_object_name = parseObjectNameFromResponse(response);
        std::cout << "[DETECTION] Object detected: " << g_detected_object_name << std::endl;
    }
    else
    {
        g_detected_object_name = "unknown";
        std::cerr << "[SERVER FAIL] Detection failed or server error. Object set to 'unknown'." << std::endl;
    }

    // OpenCV 시각화
    cv::Mat display_mat = bgr_mat.clone();
    cv::rectangle(display_mat, roi, cv::Scalar(0, 255, 0), 2); // BGR 순서 (Green)
    cv::putText(display_mat, "Detected: " + g_detected_object_name, cv::Point(roi.x, roi.y - 10),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
    cv::imshow("RGB Capture (Green box is ROI)", display_mat);
    cv::waitKey(1);
}

void realTimeCapture(rs2::pipeline &pipe, rs2::pointcloud &pc, rs2::align &align_to_depth)
{
    while (running)
    {
        rs2::frameset frames = pipe.wait_for_frames();
        frames = align_to_depth.process(frames);

        rs2::depth_frame depth = frames.get_depth_frame();
        rs2::points points = pc.calculate(depth);

        auto new_cloud = convertToPCL(points);
        auto filtered = filterPointCloud(new_cloud);

        {
            // 최적화: 잠금 범위 최소화 (포인터 업데이트만 잠금)
            std::lock_guard<std::mutex> lock(cloud_mutex);
            *current_cloud = *filtered;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }
}

// =========================================================================
// 4. 메인 함수
// =========================================================================

int main()
{
    curl_global_init(CURL_GLOBAL_ALL);
    cv::namedWindow("RGB Capture (Green box is ROI)", cv::WINDOW_NORMAL);

    try
    {
        //* 1-1. RealSense 연결 및 설정
        rs2::pipeline pipe;
        rs2::config cfg;
        cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
        cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);

        try
        {
            pipe.start(cfg);
            for (int i = 0; i < 30; ++i)
                pipe.wait_for_frames();
            std::cout << "[RealSense] RealSense 파이프라인 시작." << std::endl;
        }
        catch (const rs2::error &e)
        {
            std::cerr << "[RealSense ERROR] RealSense 오류: " << e.what() << std::endl;
            curl_global_cleanup();
            return 1;
        }

        rs2::pointcloud pc;
        rs2::align align_to_depth(RS2_STREAM_DEPTH);

        // * 1-2. 로봇 연결
        RBCobot robot("10.0.2.7", 5000);
        rb::podo::ResponseCollector rc;
        std::cout << "[ROBOT] 로봇에 연결됨 (IP: 10.0.2.7)." << std::endl;
        robot.set_operation_mode(rc, podo::OperationMode::Real);

        //* 1-2. 초기 이동 명령 전송 (홈 포지션 이동)
        robot.set_speed_bar(rc, 1.0);
        RB850::gripper_open(robot, rc, 1000);
        RB850::moveJ(robot, rc, home_q, 900.0, 900.0, "HOME 이동");

        //* 1-3. 명령 전송 (Flush)
        robot.flush(rc);
        rc.error().throw_if_not_empty();

        auto ack_result = robot.wait_until_ack_message(rc, 5.0, false);

        if (ack_result.is_success())
        {
            std::cout << "[ROBOT] 모션 완료 대기..." << std::endl;
            robot.wait_for_move_finished(rc);
        }
        else if (ack_result.is_timeout())
        {
            std::cerr << "[WARN] MoveJ ACK Timeout, robot may be stuck or already home. Proceeding." << std::endl;
        }
        else
        {
            rc.error().throw_if_not_empty();
            throw std::runtime_error("초기 로봇 이동 명령 실패 (ACK 에러).");
        }
        std::cout << "[ROBOT] 초기 이동 완료" << std::endl;
        rc.error().throw_if_not_empty();

        //* 1-4. PCL 뷰어 설정
        pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("PCL Step-by-Step Viewer"));
        viewer->setBackgroundColor(0, 0, 0);
        viewer->addCoordinateSystem(0.1);
        viewer->initCameraParameters();
        viewer->registerKeyboardCallback(keyboardCallback);
        {
            std::lock_guard<std::mutex> lock(cloud_mutex);
            viewer->addPointCloud<pcl::PointXYZ>(current_cloud, "cloud");
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 1.0, "cloud");
        }

        //* 1-5. 캡처 스레드 시작
        std::thread capture_thread(realTimeCapture, std::ref(pipe), std::ref(pc), std::ref(align_to_depth));
        bool auto_sequence_running = false;

        std::cout << "\n*** PCL Step-by-Step Processing Guide ***" << std::endl;
        std::cout << "1. **'1'** 키: **Step 1: RGB 캡처, 서버 전송 & 물체 인식** " << std::endl;
        std::cout << "2. **'2'** 키: **Step 2: 평면 제거** " << std::endl;
        std::cout << "3. **'3'** 키: **Step 3: PCA 분석**" << std::endl;
        std::cout << "4. **'4'** 키: **Step 4: Robot move** " << std::endl;
        std::cout << "5. **'5'** 키: **Step 5: 자동 실행** " << std::endl;
        std::cout << "*****************************************\n"
                  << std::endl;

        // 5. 메인 루프 (시각화 및 처리 로직)
        while (!viewer->wasStopped())
        {
            if (processing_step == 5)
            {
                std::cout << "\n[AUTO SEQUENCE] 자동 실행..." << std::endl;
                auto_sequence_running = true;
                processing_step = 1;
            }
            else if (processing_step == 1)
            {
                Timing::TimePoint start_time = Timing::Clock::now();
                std::cout << "\n[STEP 1] RGB 캡처 및 서버 전송... (Object: " << g_detected_object_name << ")" << std::endl;

                rs2::frameset frames;
                try
                {
                    frames = pipe.wait_for_frames(1000);
                }
                catch (const rs2::error &e)
                {
                    std::cerr << "[RealSense WARN] RGB Frame wait failed: " << e.what() << std::endl;
                }

                if (frames)
                {
                    captureAndSendRGB(frames);
                }

                Timing::TimePoint end_time = Timing::Clock::now();
                g_time_step1 = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

                if (auto_sequence_running)
                    processing_step = 2;
                else
                    processing_step = 0;
            }
            else if (processing_step == 2)
            {
                Timing::TimePoint start_time = Timing::Clock::now();
                std::cout << "\n[STEP 2] PCD 처리 (RANSAC 평면 제거)..." << std::endl;

                PointCloudPtr snapshot(new pcl::PointCloud<pcl::PointXYZ>);
                {
                    std::lock_guard<std::mutex> lock(cloud_mutex);
                    *snapshot = *current_cloud;
                }

                auto temp_processed = removeFloorPlane(snapshot, viewer);

                {
                    std::lock_guard<std::mutex> lock(cloud_mutex);
                    *processed_cloud = *temp_processed;
                    viewer->removePointCloud("cloud");
                    viewer->addPointCloud<pcl::PointXYZ>(temp_processed, "cloud");
                    *current_cloud = *temp_processed;
                    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "cloud");
                }

                Timing::TimePoint end_time = Timing::Clock::now();
                g_time_step2 = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

                if (auto_sequence_running)
                    processing_step = 3;
                else
                    processing_step = 0;
            }
            else if (processing_step == 3)
            {
                Timing::TimePoint start_time = Timing::Clock::now();
                std::cout << "\n[STEP 3] PCA 분석 및 좌표 계산..." << std::endl;

                PointCloudPtr snapshot(new pcl::PointCloud<pcl::PointXYZ>);
                {
                    std::lock_guard<std::mutex> lock(cloud_mutex);
                    *snapshot = *processed_cloud;
                    viewer->removePointCloud("floor_cloud");
                }

                performPCAAndVisualize(viewer, snapshot);

                Timing::TimePoint end_time = Timing::Clock::now();
                g_time_step3 = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

                if (auto_sequence_running)
                    processing_step = 4;
                else
                    processing_step = 0;
            }
            else if (processing_step == 4)
            {
                if (g_centroid.allFinite() && g_centroid[2] > -0.1 && g_detected_object_name != "unknown")
                {

                    pick_trigger = true;
                    if (auto_sequence_running)
                        processing_step = 1;
                    else
                        processing_step = 0;
                }
                else
                {
                    std::cerr << "[WARN] 로봇 동작 조건 미충족. Pick Sequence 건너뜀." << std::endl;
                    processing_step = 0;
                    auto_sequence_running = false;
                }
            }
            else if (processing_step == 7)
            {
                RB850::moveJ(robot, rc, home_q, 900.0, 900.0, "HOME 이동 명령");
                processing_step = 0;
            }

            // **로봇 동작 실행 (Step 4)**
            if (pick_trigger)
            {
                Timing::TimePoint start_time_robot = Timing::Clock::now();
                try
                {
                    std::cout << "\n[ROBOT] performRobotPickAndPlace 호출..." << std::endl;
                    performRobotPickAndPlace(robot, rc, g_centroid, g_grasp_yaw_deg, g_detected_object_name);

                    pick_trigger = false;
                    RB850::moveJ(robot, rc, home_q, 900.0, 900.0, "HOME 복귀 이동");

                    Timing::TimePoint end_time_robot = Timing::Clock::now();
                    long long duration_ms_robot = std::chrono::duration_cast<std::chrono::milliseconds>(end_time_robot - start_time_robot).count();
                    long long total_time = g_time_step1 + g_time_step2 + g_time_step3 + duration_ms_robot;

                    std::cout << "\n==================================================" << std::endl;
                    std::cout << "        PnP Cycle Time Report (ms)         " << std::endl;
                    std::cout << "--------------------------------------------------" << std::endl;
                    std::cout << "1. RGB Capture & Detection : " << g_time_step1 << " ms" << std::endl;
                    std::cout << "2. PCD Process (RANSAC)    : " << g_time_step2 << " ms" << std::endl;
                    std::cout << "3. PCA & Coordinate Calc   : " << g_time_step3 << " ms" << std::endl;
                    std::cout << "4. Robot Pick & Place Cycle: " << duration_ms_robot << " ms" << std::endl;
                    std::cout << "--------------------------------------------------" << std::endl;
                    std::cout << "   TOTAL PnP TIME          : " << total_time << " ms" << std::endl;
                    std::cout << "==================================================" << std::endl;

                    g_time_step1 = 0;
                    g_time_step2 = 0;
                    g_time_step3 = 0;
                }
                catch (const std::exception &e)
                {
                    std::cerr << "[ROBOT FATAL ERROR] Pick sequence failed: " << e.what() << std::endl;
                    pick_trigger = false;
                    auto_sequence_running = false;
                    processing_step = 0;
                }
            }

            {
                std::lock_guard<std::mutex> lock(cloud_mutex);
                viewer->updatePointCloud<pcl::PointXYZ>(current_cloud, "cloud");
            }

            viewer->spinOnce(30);
            cv::waitKey(1);
            std::this_thread::sleep_for(std::chrono::milliseconds(30));
        }

        // 6. 종료
        running = false;
        if (capture_thread.joinable())
        {
            capture_thread.join();
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "--- 프로그램 종료 오류 ---" << std::endl;
        std::cerr << " 오류 발생: " << e.what() << std::endl;
        curl_global_cleanup();
        return 1;
    }

    curl_global_cleanup();
    return 0;
}