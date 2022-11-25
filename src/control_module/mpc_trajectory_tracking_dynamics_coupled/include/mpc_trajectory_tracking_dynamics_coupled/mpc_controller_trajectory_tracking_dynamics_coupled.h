#ifndef MPC_CONTROLLER_H
#define MPC_CONTROLLER_H

#include <vector>
#include <chrono>     // 时间库
#include <functional> // 函数模板库
#include <string>
#include <memory>
#include "rclcpp/rate.hpp"
#include <cstdio>
#include <thread>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/QR>
#include <eigen3/unsupported/Eigen/Splines>

#include <vector>
#include <iostream>
#include <math.h>
#include <algorithm>
#include <iomanip>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "chassis_msg/msg/adu_drive_cmd.hpp"
#include "chassis_msg/msg/adu_body_cmd.hpp"
#include "chassis_msg/msg/wvcu_body_status.hpp"
#include "chassis_msg/msg/wvcu_flt_cod.hpp"
#include "chassis_msg/msg/wvcu_horizontal_status.hpp"
#include "chassis_msg/msg/wvcu_longitudinal_status.hpp"
#include "chassis_msg/msg/wvcu_veh_sph_lim.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/convert.h>

#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <nav_msgs/msg/path.hpp>

#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>


using namespace std::chrono_literals; // 表示时间长度的命名空间
//调用嵌套空间std::chrono_literals下的函数
using std::cout;
using std::endl;
using std::vector;
using std::placeholders::_1;

using CppAD::AD;
using Eigen::VectorXd;


class mpc_controller
{
private:
    /*Indexes on the 1-D vector for readability，
    These indexes are used for "vars"*/
    size_t x_start;
    size_t y_start;
    size_t psi_start;
    size_t v_longitudinal_start;
    size_t v_lateral_start;
    size_t yaw_rate_start;
    size_t cte_start;
    size_t epsi_start;
    size_t front_wheel_angle_start;
    size_t longitudinal_acceleration_start; //控制时域长度为25的时候，控制量一共有24个。
    size_t front_wheel_angle_increment_start;
    size_t longitudinal_acceleration_increment_start;
    size_t Np;
    size_t Nc;
    double dt;
    double a_lateral;
public:
    mpc_controller(/* args */);
    virtual ~mpc_controller();

    // Solve the model given an initial state and polynomial coefficients
    // Retrun the first actuators
    std::vector<double> Solve(const Eigen::VectorXd &state,
                              const Eigen::VectorXd &coeffs,
                              const double &target_v,
                              const int &cte_weight,
                              const int &epsi_weight,
                              const int &v_weight,
                              const int &steer_actuator_cost_weight,
                              const int &acc_actuator_cost_weight,
                              const int &change_steer_cost_weight,
                              const int &change_accel_cost_weight,
                              const int &mpc_control_horizon_length,
                              const double &mpc_control_step_length,
                              const double &kinamatic_para_Lf,
                              const double &a_lateral,
                              const double &old_steer_value,
                              const double &old_throttle_value,
                              const double &steer_ratio);
};

class FG_eval
{
public:
    /*Define weights for different terms of objective*/
    // double yaw_rate;
    double cte_weight;
    double epsi_weight;
    double v_weight;
    double steer_actuator_cost_weight_fg;
    double acc_actuator_cost_weight_fg;
    double change_steer_cost_weight;
    double change_accel_cost_weight;
    double target_v; // km/h
    size_t mpc_prediction_horizon_length;
    double mpc_control_step_length;
    double kinamatic_para_Lf;
    /*Reference values that will be include in objective function*/
    double ref_cte = 0;
    double ref_epsi = 0;
    double ref_v; // m/s
    /*Set the timestep length and duration*/
    size_t Np;
    size_t Nc;
    double dt;
    double a_lateral;
    double steer_ratio;
    AD<double> old_steer_value;
    AD<double> old_throttle_value;

    /*
    This value assumes the model presented in the classroom is used.
    It was obtained by measuring the radius formed by running the vehicle in the
    simulator around in a circle with a constant steering angle and velocity on
    a flat terrain.
    Lf was tuned until the the radius formed by the simulating the model
    presented in the classroom matched the previous radius.
    This is the length from front to CoG that has a similar radius.
    */
    double Lf;

    /*Indexes on the 1-D vector for readability，
    These indexes are used for "vars"*/
    size_t x_start;
    size_t y_start;
    size_t psi_start;
    size_t v_longitudinal_start;
    size_t v_lateral_start;
    size_t yaw_rate_start;
    size_t cte_start;
    size_t epsi_start;
    size_t front_wheel_angle_start;
    size_t longitudinal_acceleration_start; //控制时域长度为25的时候，控制量一共有24个。

    size_t front_wheel_angle_increment_start; 
    size_t longitudinal_acceleration_increment_start;

    // AD<double> Ccf = 46900; // 前轮轮胎侧偏刚度
    // AD<double> Ccr = 42700; // 后轮轮胎侧偏刚度
    AD<double> Cf = 46000; // 前轮轮胎纵向刚度
    AD<double> Cr = 46000; // 后轮轮胎纵向刚度
    // AD<double> Sf = 0.2;    // 前轮滑移率
    // AD<double> Sr = 0.0;    // 后轮滑移率
    AD<double> m = 1000;
    // AD<double> v_lateral;
    // AD<double> v_longitudinal;
    AD<double> l = 1.60;
    AD<double> I = 3575;
    AD<double> lf = 0.95;
    AD<double> lr = 1.6 - lf;

    double x;
    double y;
    double psi;
    double v_longitudinal;
    double v_lateral;
    double yaw_rate;
    double cte;
    double epsi;



public:
    VectorXd coeffs; // Fitted polynomial coefficients
    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

public:
    /* coeffs are the coefficients of the fitted polynomial, 
       will be used by the cross track error and heading error equations.*/
    FG_eval(const Eigen::VectorXd &state,
            VectorXd coeffs,
            const double &target_v,
            const int &cte_weight,
            const int &epsi_weight,
            const int &v_weight,
            const int &steer_actuator_cost_weight_fg,
            const int &acc_actuator_cost_weight,
            const int &change_steer_cost_weight,
            const int &change_accel_cost_weight,
            const int &mpc_prediction_horizon_length,
            const int &mpc_control_horizon_length,
            const double &mpc_control_step_length,
            const double &kinamatic_para_Lf,
            const double &a_lateral,
            const double &old_steer_value,
            const double &old_throttle_value,
            const double &steer_ratio);
    ~FG_eval();
    // 重载函数调用运算符，创建一个可以传递任意数目参数的运算符函数
    // 函数对象,可以将像函数调用一样使用函数对象
    void operator()(ADvector &fg, ADvector &vars);
};


/*'''*****************************************************************************************************
# Class Name  : 
# FileFunction: 
# Comments    :
*****************************************************************************************************'''*/
class MpcTrajectoryTrackingPublisher : public rclcpp::Node
{
public:
    MpcTrajectoryTrackingPublisher();
    ~MpcTrajectoryTrackingPublisher();
    void mpc_tracking_iteration_callback();
    void ins_data_receive_callback(nav_msgs::msg::Odometry::SharedPtr msg); // 后面加 const表示函数不可以修改class的成员
    void eps_feedback_callback(chassis_msg::msg::WVCUHorizontalStatus::SharedPtr msg);
    void global_path_callback(nav_msgs::msg::Path::SharedPtr msg);
    void parameter_reconfigure_callback();
    void palnner_frenet_path_receive_callback(nav_msgs::msg::Path::SharedPtr msg);
    void palnner_cartesian_path_receive_callback(visualization_msgs::msg::Marker::SharedPtr msg);

public:
    rclcpp::Publisher<chassis_msg::msg::ADUDriveCmd>::SharedPtr mpc_control_signals_publisher;

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr mpc_iteration_time_publisher;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr history_path_visualization_publisher;

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr mpc_reference_path_publisher;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr mpc_output_path_publisher;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ins_data_subscription_;
    rclcpp::Subscription<chassis_msg::msg::WVCUHorizontalStatus>::SharedPtr eps_feedback_subscription;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_path_subscription;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr mpc_planner_frenet_path_subscription;
    rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr mpc_planner_cartesian_path_subscription;

    mpc_controller mpc;
    tf2::Quaternion ins_quaternion_transform;
    rclcpp::TimerBase::SharedPtr mpc_iteration_timer_;

    visualization_msgs::msg::Marker reference_path;
    visualization_msgs::msg::Marker mpc_output_path;
    nav_msgs::msg::Path history_path;
    geometry_msgs::msg::PoseStamped history_path_points;

    std_msgs::msg::Float32 mpc_iteration_duration_msg = std_msgs::msg::Float32();

    long int history_path_id = 103;

    rclcpp::TimerBase::SharedPtr parameter_reconfigure_timer_;

    double ref_v;

    double vehicle_steering_ratio_double;
    double vehicle_Lf_double;

    int mpc_control_horizon_length_int;
    double mpc_control_step_length_double;

    bool mpc_tracking_enable_bool;

    int mpc_cte_weight_int;
    int mpc_epsi_weight_int;
    int mpc_v_weight_int;
    int mpc_steer_actuator_cost_weight_int;
    int mpc_acc_actuator_cost_weight_int;
    int mpc_change_steer_cost_weight_int;
    int mpc_change_accel_cost_weight_int;

    int reference_path_id = 101;

    int mpc_reference_path_length;
    double mpc_controller_delay_compensation;
    double mpc_point_distance_of_reference_line_visualization;
    int mpc_points_number_of_reference_line_visualization;
    int mpc_working_mode;

    double target_v; // km/h
    int with_planner_flag; // MPC做全局纯跟踪还是与规划算法上下贯通的标志 为 0 做纯跟踪，为 1 与规划算法上下贯通

    // 考虑道路曲率的车辆运动学模型里面可以设置的参数
    double steering_ratio;
    double kinamatic_para_Lf;

    // MPC预测模型里可以调节的参数
    int mpc_control_horizon_length; // MPC求解的时候,一次求解里面 MPC 预测的步数,乘以步长,就是 MPC 一次求解考虑的未来的范围大小.
    double mpc_control_step_length; //Original 0.1 不是控制步长,而是模型离散化后进行预测时的预测步长
    // MPC 目标函数里面可以调节的权重
    int cte_weight; //Original2000
    int epsi_weight;
    int v_weight;
    int steer_actuator_cost_weight; //Original 6000
    int acc_actuator_cost_weight; //Original 6000
    int change_steer_cost_weight;
    int change_accel_cost_weight;

    double ins_delay;

    // MPC 里面拟合前方参考路径的时候使用的路点的数量，这个意义更新为使用的前方多少距离内的点
    double reference_path_length;

    int reference_path_points_number;

    double controller_delay_compensation; // 控制信号延时补偿

    // 在 RVIZ 里面可视化根据参考路径拟合得到的多项式表达的曲线时,显示的点的数量和点的距离参数
    double point_distance_of_reference_line_visualization; // step on x
    int points_number_of_reference_line_visualization;     /* how many point "int the future" to be plotted. */
    bool mpc_enable_signal = true;

    int working_mode;
    chassis_msg::msg::ADUDriveCmd vehicle_control_msg = chassis_msg::msg::ADUDriveCmd();

private:
    double steer_value;
    double throttle_value;

    double px;
    double py;
    double psi;
    double yaw_rate;
    double v_longitudinal;
    double v_lateral;

    double delta;
    double a_longitudinal;
    double a_lateral;
    double omega;
    rclcpp::Time ins_frame_arrive_time;
    double ins_arrive_at_rs232_buffer;

    bool is_eps_received = false;
    bool is_global_path_received = false;
    bool is_ins_data_received = false;
    bool is_planner_frenet_path_received = false;
    bool is_planner_cartesian_path_received = false;

    double max_cte = 0;
    double min_cte = 0;

    double car_s;
    double car_d;

    int former_point_of_current_position; // 车辆当前定位点，前面的一个地图点的索引号，即距离车辆最近的前方的地图点的索引号。

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
    double ins_data_arrive_at_mpc_through_callback;

    double old_steer_value;
    double old_throttle_value;

    double steering_damp = 1;

    vector<double> cal_heading_ratio;
    vector<double> cal_heading_ratio_diff;
};

#endif
