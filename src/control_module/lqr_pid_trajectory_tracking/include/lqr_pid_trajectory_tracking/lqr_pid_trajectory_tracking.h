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
    void eps_feedback_callback(chassis_msg::msg::WVCUHorizontalStatus::SharedPtr msg);
    void global_path_callback(nav_msgs::msg::Path::SharedPtr msg);
    void palnner_frenet_path_receive_callback(nav_msgs::msg::Path::SharedPtr msg);
    void palnner_cartesian_path_receive_callback(visualization_msgs::msg::Marker::SharedPtr msg);

public:
    rclcpp::Publisher<chassis_msg::msg::ADUDriveCmd>::SharedPtr lqr_pid_control_signals_gas_brake_steer_publisher;
    int working_mode;
    chassis_msg::msg::ADUDriveCmd vehicle_control_gas_brake_steer_msg = chassis_msg::msg::ADUDriveCmd();

    rclcpp::Publisher<chassis_msg::msg::ADUGearRequest>::SharedPtr lqr_pid_control_signals_gear_publisher;
    chassis_msg::msg::ADUGearRequest vehicle_control_gear_msg;

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr lqr_pid_iteration_time_publisher;

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr fake_velocity_vis_publisher;
    std_msgs::msg::Float32 fake_velocity;

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr lqr_pid_reference_path_publisher;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr lqr_pid_output_path_publisher;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ins_data_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr velocity_from_csv_subscription;
    rclcpp::Subscription<chassis_msg::msg::WVCUHorizontalStatus>::SharedPtr eps_feedback_subscription;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_path_subscription;

    rclcpp::Subscription<chassis_msg::msg::WVCULongitudinalStatus>::SharedPtr vehicle_longitudinal_status_feedback_subscription;
    void vehicle_status_feedback_callback(chassis_msg::msg::WVCULongitudinalStatus::SharedPtr msg);
    chassis_msg::msg::WVCULongitudinalStatus::SharedPtr vehicle_longitudinal_feedback_msg;
    

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr lqr_pid_planner_frenet_path_subscription;
    rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr lqr_pid_planner_cartesian_path_subscription;

    rclcpp::Publisher<carla_msgs::msg::CarlaEgoVehicleControl>::SharedPtr carla_vehicle_control_publisher;
    carla_msgs::msg::CarlaEgoVehicleControl carla_control_cmd;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr carla_localization_data_subscription;
    void carla_odom_callback(nav_msgs::msg::Odometry::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr carla_lacalization_data_imu_subscription;
    void carla_imu_callback(sensor_msgs::msg::Imu::SharedPtr msg);

    rclcpp::Subscription<carla_msgs::msg::CarlaEgoVehicleStatus>::SharedPtr carla_status_subscription;
    void carla_vehicle_status_callback(carla_msgs::msg::CarlaEgoVehicleStatus::SharedPtr msg);

    rclcpp::Publisher<carla_msgs::msg::CarlaVehicleTargetVelocity>::SharedPtr vehicle_control_target_velocity_publisher;
    carla_msgs::msg::CarlaVehicleTargetVelocity vehicle_control_target_velocity;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr history_path_visualization_publisher;
    nav_msgs::msg::Path history_path;
    geometry_msgs::msg::PoseStamped history_path_points;
    // std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_gps_vehicle;

    geometry_msgs::msg::TransformStamped odom_translation;

    // VehicleState vehicleState_;
    // bool firstRecord_ = true;

    // lqr_pid_controller lqr_pid;
    tf2::Quaternion ins_quaternion_transform;
    rclcpp::TimerBase::SharedPtr lqr_pid_iteration_timer_;

    visualization_msgs::msg::Marker reference_path;
    visualization_msgs::msg::Marker lqr_pid_output_path;

    std_msgs::msg::Float32 lqr_pid_iteration_duration_msg = std_msgs::msg::Float32();

    rclcpp::TimerBase::SharedPtr parameter_reconfigure_timer_;

    double ref_v;

    double vehicle_steering_ratio_double;
    double vehicle_Lf_double;

    int lqr_pid_control_horizon_length_int;
    double lqr_pid_control_step_length_double;

    bool lqr_pid_tracking_enable_bool;

    double old_steer_value;
    double old_throttle_value;

    int lqr_pid_cte_weight_int;
    int lqr_pid_epsi_weight_int;
    int lqr_pid_v_weight_int;
    int lqr_pid_steer_actuator_cost_weight_int;
    int lqr_pid_acc_actuator_cost_weight_int;
    int lqr_pid_change_steer_cost_weight_int;
    int lqr_pid_change_accel_cost_weight_int;

    int reference_path_id = 101;

    int lqr_pid_reference_path_length;
    double lqr_pid_controller_delay_compensation;
    double lqr_pid_point_distance_of_reference_line_visualization;
    int lqr_pid_points_number_of_reference_line_visualization;
    int lqr_pid_working_mode;

    double target_v; // km/h
    int with_planner_flag; // lqr_pid做全局纯跟踪还是与规划算法上下贯通的标志 为 0 做纯跟踪，为 1 与规划算法上下贯通

    // 考虑道路曲率的车辆运动学模型里面可以设置的参数
    double steering_ratio;
    double kinamatic_para_Lf;

    // lqr_pid预测模型里可以调节的参数
    int lqr_pid_control_horizon_length; // lqr_pid求解的时候,一次求解里面 lqr_pid 预测的步数,乘以步长,就是 lqr_pid 一次求解考虑的未来的范围大小.
    double lqr_pid_control_step_length; //Original 0.1 不是控制步长,而是模型离散化后进行预测时的预测步长
    // lqr_pid 目标函数里面可以调节的权重
    int cte_weight; //Original2000
    int epsi_weight;
    int v_weight;
    int steer_actuator_cost_weight; //Original 6000
    int acc_actuator_cost_weight; //Original 6000
    int change_steer_cost_weight;
    int change_accel_cost_weight;

    double ins_delay;

    // lqr_pid 里面拟合前方参考路径的时候使用的路点的数量，这个意义更新为使用的前方多少距离内的点
    double reference_path_length;

    int reference_path_points_number;

    double controller_delay_compensation; // 控制信号延时补偿

    // 在 RVIZ 里面可视化根据参考路径拟合得到的多项式表达的曲线时,显示的点的数量和点的距离参数
    double point_distance_of_reference_line_visualization; // step on x
    int points_number_of_reference_line_visualization;     /* how many point "int the future" to be plotted. */
    bool lqr_pid_enable_signal = true;

    double steer_value;
    double throttle_value;

    double px;
    double py;
    double psi;    
    double car_s;
    double car_d;
    double delta;
    double omega;
    double a_lateral;
    double yaw_rate;
    double v_lateral;
    double v_longitudinal;
    double a_longitudinal;

    int former_point_of_current_position; // 车辆当前定位点，前面的一个地图点的索引号，即距离车辆最近的前方的地图点的索引号。

    rclcpp::Time ins_frame_arrive_time;
    double ins_arrive_at_rs232_buffer;

    bool is_eps_received = false;
    bool is_global_path_received = false;
    bool is_ins_data_received = false;
    bool is_planner_frenet_path_received = false;
    bool is_planner_cartesian_path_received = false;
    bool is_vehicle_longitudinal_received = false;
    bool is_vehicle_horizontal_received = false;

    double max_cte = 0;
    double min_cte = 0;

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

    VectorXd coeffs;
    double cte;

    vector<double> next_x_vals;
    vector<double> next_y_vals;

    int qos_=2;

    // 以下是从lqr移植过来的时候新增的

    //计算两点之间的距离
    double pointDistance(const TrajectoryPoint &point, const double x, const double y) {
        double dx = point.x - x;
        double dy = point.y - y;
        return sqrt(dx * dx + dy * dy);
    }
    double pointDistance(const double x1, const double y1, const double x, const double y) {
        double dx = x1 - x;
        double dy = y1 - y;
        return sqrt(dx * dx + dy * dy);
    }

    TrajectoryPoint QueryNearestPointByPosition(const double x, const double y);
    int QueryNearestPointByPosition(const double x, const double y, std::vector<TrajectoryPoint> current_trajectory );
    double PointDistanceSquare(const TrajectoryPoint &point, const double x, const double y);

       private:
    double targetSpeed_ = 5;
    double controlFrequency_ = 100;                 //控制频率
    double goalTolerance_ = 0.5;                    //到终点的容忍距离
    bool isReachGoal_ = false;
    bool firstRecord_ = true;
    int cnt;

    std::string _line;
    std::vector<std::pair<double, double>> xy_points;
    std::vector<double> v_points;

    TrajectoryData global_reference_trajectory;
    TrajectoryPoint goal_point;
    std::vector<TrajectoryPoint> trajectory_points_;

    TrajectoryData local_reference_trajectory;

    TrajectoryData from_planner_reference_trajectory;

    std::string roadmap_path;
    double speed_P, speed_I, speed_D, target_speed;

    VehicleState vehicleState_;
    // TrajectoryData planningPublishedTrajectory_;    //跟踪的轨迹
    // TrajectoryPoint goalPoint_;                     //终点

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_gps_vehicle;
    std::unique_ptr<zww::control::PIDController> pid_controller_longitudinal;
    std::unique_ptr<zww::control::LqrController> lqr_controller_lateral;


};

#endif