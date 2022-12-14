#ifndef LATTICE_PLANNER_H_
#define LATTICE_PLANNER_H_

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
#include <vector>
#include <iostream>
#include <math.h>
#include <algorithm>
#include <iomanip>
#include "lattice_planner/utils.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int16.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/convert.h>

#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>

#include <uWS/uWS.h>
#include <fstream>
#include <string>
#include <algorithm>

#include <eigen3/unsupported/Eigen/Splines>


// #include <boost/math/interpolators/makima.hpp>

//调用嵌套空间std::chrono_literals下的函数
using namespace std::chrono_literals; // 表示时间长度的命名空间

using std::cout;
using std::endl;
using std::vector;
using std::placeholders::_1;
// using boost::math::interpolators::makima;

class LatticePlanner : public rclcpp::Node
{
public:    
    LatticePlanner();
    virtual ~LatticePlanner();

public:
    void ins_data_receive_callback(nav_msgs::msg::Odometry::SharedPtr msg); // 后面加 const表示函数不可以修改class的成员
    void global_path_callback(nav_msgs::msg::Path::SharedPtr msg);
    void planner_tracking_iteration_callback();
    void sensor_fusion_results_bounding_box_callback(visualization_msgs::msg::MarkerArray::SharedPtr msg);
    void sensor_fusion_results_label_callback(visualization_msgs::msg::MarkerArray::SharedPtr msg);
    void fsm_behavior_decision_makeing_callback(std_msgs::msg::Int16::SharedPtr msg);

    
public:
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_path_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ins_data_subscription_;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr fsm_behavior_decision_makeing_subscription;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr planner_iteration_time_publisher;  // 用于统计 planner 求解时间的广播器
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr lattice_planner_path_cartesian_publisher; // 广播规划器求解结果
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr lattice_planner_path_frenet_publisher; // 广播规划器求解结果
    rclcpp::TimerBase::SharedPtr planner_iteration_timer_; // 定时器，定频调用路径规划算法

    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr sensor_fusion_results_bounding_box_subscription_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr sensor_fusion_results_label_subscription_;

    int qos_ = 2; // 定义消息队列大小为2，若超过2，则最先进入的消息将被舍弃，发布与订阅均有队列
    double ins_delay;
    int working_mode;

    std_msgs::msg::Float32 planner_iteration_duration_msg = std_msgs::msg::Float32();

    visualization_msgs::msg::Marker lattice_planner_path_cardesian;
    geometry_msgs::msg::Point highway_with_prediction_planner_point_cartesian;

    nav_msgs::msg::Path lattice_planner_path_frenet;
    geometry_msgs::msg::PoseStamped highway_with_prediction_planner_point_frenet;

    visualization_msgs::msg::MarkerArray sensor_fusion_results_label;
    visualization_msgs::msg::Marker sensor_fusion_single_result_label;

    visualization_msgs::msg::MarkerArray sensor_fusion_results_bounding_box;
    visualization_msgs::msg::Marker sensor_fusion_single_result_bounding_box;

    double ins_data_arrive_at_planner_through_callback;
    double a_longitudinal;
    double a_lateral;
    double px;
    double py;
    double psi;
    double v_longitudinal;
    double v_lateral;
    double yaw_rate;

    rclcpp::Time ins_frame_arrive_time;     
    double ins_arrive_at_rs232_buffer;

    bool is_global_path_received = false;
    bool is_ins_data_received = false;
    bool is_sensor_fusion_results_label_received = false;
    bool is_sensor_fusion_results_bounding_box_reveived = false;
    bool bounding_box_label_same_frame_check_flag = false;

    vector<double> global_path_x; // ptsx
    vector<double> global_path_y; // ptsy
    vector<double> global_path_s; // ptsy

    vector<double> global_path_remap_x;
    vector<double> global_path_remap_y;


    int former_point_of_current_position; // 车辆当前定位点，前面的一个地图点的索引号，即距离车辆最近的前方的地图点的索引号。

    int lane = 0; // start in lane 1
    double ref_vel = 0.0; // have a reference velocity to target mph

    int lattice_planner_path_id = 10;

    double car_s;
    double car_d;

    vector<double> next_x_vals;
    vector<double> next_y_vals;
    vector<double> next_s_vals;
    vector<double> next_v_vals;

    vector<double> next_x_vals_previous;
    vector<double> next_y_vals_previous;
    vector<double> next_s_vals_previous;
    vector<double> next_v_vals_previous;

    vector<double> next_x_vals_previous_remap;
    vector<double> next_y_vals_previous_remap;
    vector<double> next_s_vals_previous_remap;
    vector<double> next_v_vals_previous_remap;

    vector<Object_Around> sensor_fusion;

    VectorXd coeffs;
    double cte;

    int current_velocity_behavior  = 0;
};

#endif