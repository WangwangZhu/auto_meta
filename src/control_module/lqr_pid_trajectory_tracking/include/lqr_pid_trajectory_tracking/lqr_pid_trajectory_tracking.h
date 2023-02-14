#ifndef LQR_PID_TRAJECTORY_TRACKING_H_
#define LQR_PID_TRAJECTORY_TRACKING_H_

#include <vector>
#include <chrono>     // 时间库
#include <string>
#include <memory>
#include <cstdio>
#include <thread>
#include <iomanip>
#include <iostream>
#include <algorithm>
#include <functional> // 函数模板库
#include <eigen3/Eigen/QR>
#include <eigen3/Eigen/Core>
#include <eigen3/unsupported/Eigen/Splines>

#include "rclcpp/rate.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include <visualization_msgs/msg/marker.hpp>

#include "chassis_msg/msg/wvcu_flt_cod.hpp"
#include "chassis_msg/msg/adu_body_cmd.hpp"
#include "chassis_msg/msg/adu_drive_cmd.hpp"
#include "chassis_msg/msg/wvcu_veh_sph_lim.hpp"
#include "chassis_msg/msg/adu_gear_request.hpp"
#include "chassis_msg/msg/wvcu_body_status.hpp"
#include "chassis_msg/msg/wvcu_horizontal_status.hpp"
#include "chassis_msg/msg/wvcu_longitudinal_status.hpp"

#include "matrix_interfaces/msg/msg_to_can.hpp"  

#include <math.h>
#include <tf2/convert.h>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_broadcaster.h>

#include "lqr_pid_trajectory_tracking_controller.h"
#include "common.h"

#include "carla_msgs/msg/carla_ego_vehicle_control.hpp"
#include "carla_msgs/msg/carla_status.hpp"
#include "carla_msgs/msg/carla_vehicle_target_velocity.hpp"
#include "carla_msgs/msg/carla_ego_vehicle_status.hpp"
#include "pid_controller.h"

using namespace std::chrono_literals; // 表示时间长度的命名空间
using std::cout;
using std::endl;
using std::vector;
using std::placeholders::_1;

using CppAD::AD;
using Eigen::VectorXd;

/*'''*****************************************************************************************************
# Class Name  : 
# FileFunction: 
# Comments    :
*****************************************************************************************************'''*/
class LQRPIDTrajectoryTracking : public rclcpp::Node
{
public:
    LQRPIDTrajectoryTracking();
    ~LQRPIDTrajectoryTracking();
    void lqr_pid_tracking_iteration_callback();
    void ins_data_receive_callback(nav_msgs::msg::Odometry::SharedPtr msg); // 后面加 const表示函数不可以修改class的成员
    void global_path_callback(nav_msgs::msg::Path::SharedPtr msg);
    void palnner_frenet_path_receive_callback(nav_msgs::msg::Path::SharedPtr msg);
    void palnner_cartesian_path_receive_callback(visualization_msgs::msg::Marker::SharedPtr msg);
    void vehicle_status_feedback_callback(chassis_msg::msg::WVCULongitudinalStatus::SharedPtr msg);
    void localization_data_callback(nav_msgs::msg::Odometry::SharedPtr msg);
    void localization_data_imu_callback(sensor_msgs::msg::Imu::SharedPtr msg);

    TrajectoryPoint QueryNearestPointByPosition(const double x, const double y){
        double d_min = this->PointDistanceSquare(trajectory_points_.front(), x, y);
        size_t index_min = 0;

        for (size_t i = 1; i < trajectory_points_.size(); ++i) {
            double d_temp = this->PointDistanceSquare(trajectory_points_[i], x, y);
            if (d_temp < d_min) {
                d_min = d_temp;
                index_min = i;
            }
        }
        return trajectory_points_[index_min];
    };

    int QueryNearestPointByPosition(const double x, const double y, std::vector<TrajectoryPoint> current_trajectory ){
        double d_min = this->PointDistanceSquare(current_trajectory.front(), x, y);
        int index_min = 0;

        for (int i = 1; i < current_trajectory.size(); ++i) {
            double d_temp = this->PointDistanceSquare(current_trajectory[i], x, y);
            if (d_temp < d_min) {
                d_min = d_temp;
                index_min = i;
            }
        }
        return index_min;
    };
    double PointDistanceSquare(const TrajectoryPoint &point, const double x, const double y){
        double dx = point.x - x;
        double dy = point.y - y;
        return dx * dx + dy * dy;
    };
    //计算两点之间的距离
    double pointDistance(const TrajectoryPoint &point, const double x, const double y) {
        double dx = point.x - x;
        double dy = point.y - y;
        return sqrt(dx * dx + dy * dy);
    };
    double pointDistance(const double x1, const double y1, const double x, const double y) {
        double dx = x1 - x;
        double dy = y1 - y;
        return sqrt(dx * dx + dy * dy);
    };

    chassis_msg::msg::ADUDriveCmd vehicle_control_gas_brake_steer_msg = chassis_msg::msg::ADUDriveCmd();
    chassis_msg::msg::ADUGearRequest vehicle_control_gear_msg;
    chassis_msg::msg::WVCULongitudinalStatus::SharedPtr vehicle_longitudinal_feedback_msg;
    carla_msgs::msg::CarlaEgoVehicleControl carla_control_cmd;
    carla_msgs::msg::CarlaVehicleTargetVelocity vehicle_control_target_velocity;
    std_msgs::msg::Float32 lqr_pid_iteration_duration_msg = std_msgs::msg::Float32();
    matrix_interfaces::msg::MsgToCan matrix_ad_control_message = matrix_interfaces::msg::MsgToCan();

public:
    rclcpp::Publisher<chassis_msg::msg::ADUDriveCmd>::SharedPtr lqr_pid_control_signals_gas_brake_steer_publisher;
    rclcpp::Publisher<chassis_msg::msg::ADUGearRequest>::SharedPtr lqr_pid_control_signals_gear_publisher;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr lqr_pid_iteration_time_publisher;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr lqr_pid_reference_path_publisher;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr lqr_pid_output_path_publisher;
    rclcpp::Publisher<carla_msgs::msg::CarlaEgoVehicleControl>::SharedPtr carla_vehicle_control_publisher;
    rclcpp::Publisher<carla_msgs::msg::CarlaVehicleTargetVelocity>::SharedPtr vehicle_control_target_velocity_publisher;
    rclcpp::Publisher<matrix_interfaces::msg::MsgToCan>::SharedPtr matrix_ad_control_publisher_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ins_data_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_path_subscription;
    rclcpp::Subscription<chassis_msg::msg::WVCULongitudinalStatus>::SharedPtr vehicle_longitudinal_status_feedback_subscription;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr lqr_pid_planner_frenet_path_subscription;
    rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr lqr_pid_planner_cartesian_path_subscription;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr localization_data_subscription;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr lacalization_data_imu_subscription;

    tf2::Quaternion ins_quaternion_transform;
    rclcpp::TimerBase::SharedPtr lqr_pid_iteration_timer_;
    rclcpp::Time ins_frame_arrive_time;

    int qos_ = 2;
    double ref_v;
    double vehicle_steering_ratio_double;
    int working_mode;
    int reference_path_id = 101;
    int lqr_pid_working_mode;
    double target_v; // km/h
    int with_planner_flag; // lqr_pid做全局纯跟踪还是与规划算法上下贯通的标志 为 0 做纯跟踪，为 1 与规划算法上下贯通
    double lqr_controller_u;
    double lqr_controller_cost_q_1;
    double lqr_controller_cost_q_2;
    double lqr_controller_cost_q_3;
    double lqr_controller_cost_q_4;
    double steering_ratio;
    double ins_delay;
    double car_s;
    double car_d;

    double ins_arrive_at_rs232_buffer;

    bool is_eps_received = false;
    bool is_global_path_received = false;
    bool is_ins_data_received = false;
    bool is_planner_frenet_path_received = false;
    bool is_planner_cartesian_path_received = false;
    bool is_vehicle_longitudinal_received = false;
    bool is_vehicle_horizontal_received = false;

    vector<double> global_path_x; // ptsx
    vector<double> global_path_y; // ptsy

    vector<double> global_path_remap_x;
    vector<double> global_path_remap_y;

    vector<double> planner_path_x;
    vector<double> planner_path_y;

    vector<double> planner_path_s;
    vector<double> planner_path_v;
    
    vector<double> planner_path_remap_x;
    vector<double> planner_path_remap_y;

    double roll_current, pitch_current, heading_current;
    double ins_data_arrive_at_lqr_pid_through_callback;

    double goalTolerance_ = 0.5;                    //到终点的容忍距离
    bool isReachGoal_ = false;
    bool firstRecord_ = true;

    std::vector<std::pair<double, double>> xy_points;
    std::vector<double> v_points;

    TrajectoryData global_reference_trajectory;
    TrajectoryPoint goal_point;
    std::vector<TrajectoryPoint> trajectory_points_;

    TrajectoryData local_reference_trajectory;

    TrajectoryData from_planner_reference_trajectory;

    double speed_P, speed_I, speed_D, target_speed;

    VehicleState vehicleState_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_gps_vehicle;
    std::unique_ptr<zww::control::PIDController> pid_controller_longitudinal;
    std::unique_ptr<zww::control::LqrController> lqr_controller_lateral;
};

#endif