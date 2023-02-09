#ifndef SENSOR_FUSION_H
#define SENSOR_FUSION_H

#include "utils.h"
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

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "derived_object_msgs/msg/object_array.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/convert.h>

#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>

using namespace std::chrono_literals; // 表示时间长度的命名空间
//调用嵌套空间std::chrono_literals下的函数
using std::cout;
using std::endl;
using std::vector;
using std::placeholders::_1;
using std::ostringstream;
using std::string;

using CppAD::AD;
using Eigen::VectorXd;

/*'''*****************************************************************************************************
# Class Name  : 
# FileFunction: 
# Comments    :
*****************************************************************************************************'''*/
class SensorFusion : public rclcpp::Node
{
public:
    SensorFusion();
    ~SensorFusion();
    void sensor_fusion_iteration_callback(derived_object_msgs::msg::ObjectArray::SharedPtr msg); // 被定时器定时回调
    void sensor_fusion_ins_data_receive_callback(nav_msgs::msg::Odometry::SharedPtr msg); 
    void sensor_fusion_global_path_callback(nav_msgs::msg::Path::SharedPtr msg);
    void sensor_fusion_object_pack( double object_s, 
                                    double object_d, 
                                    double object_length, 
                                    double object_width, 
                                    double object_height, 
                                    double object_heading,  
                                    double object_v_X,  
                                    double object_v_Y,  
                                    double object_v_yaw_rate, 
                                    string object_label,  
                                    uint object_id, 
                                    double object_line_sacle, 
                                    double object_label_scale,
                                    double object_color_r, 
                                    double object_color_g, 
                                    double object_color_b, 
                                    double object_color_a,
                                    visualization_msgs::msg::MarkerArray &sensor_fusion_results_bounding_box_msg,
                                    visualization_msgs::msg::MarkerArray &sensor_fusion_results_label_msg,
                                    string object_frame_id = "odom");

public:
    rclcpp::TimerBase::SharedPtr sensor_fusion_iteration_timer_; // 定时器，定频回调，控制输出频率

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sensor_fusion_ins_data_subscription_;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sensor_fusion_global_path_subscription_;

    rclcpp::Subscription<derived_object_msgs::msg::ObjectArray>::SharedPtr sensor_fusion_objects_from_carla;

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr sensor_fusion_iteration_time_publisher; // 广播一次算法迭代耗时

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr sensor_fusion_results_bounding_box_publisher; // 存放环境感知结果
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr sensor_fusion_results_label_publisher; // 存放环境感知结果
    // rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr sensor_fusion_results_property_publisher; // 存放环境感知结果

    std_msgs::msg::Float32 sensor_fusion_iteration_duration_msg; // 存放一次算法迭代耗时

    visualization_msgs::msg::MarkerArray sensor_fusion_results_bounding_box_msg; // 存放环境感知结果
    visualization_msgs::msg::Marker sensor_fusion_single_result_bounding_box_msg;

    visualization_msgs::msg::MarkerArray sensor_fusion_results_label_msg;
    visualization_msgs::msg::Marker sensor_fusion_single_result_label_msg;
    geometry_msgs::msg::Pose label_pose; // label的位姿

    // geometry_msgs::msg::PoseArray sensor_fusion_results_property_msg;     
    // geometry_msgs::msg::Pose sensor_fusion_single_result_property_msg;     


private:

    int qos_ = 2;
    double px;
    double py;
    double psi;
    double yaw_rate;
    double v_longitudinal;
    double v_lateral;
    double a_longitudinal;
    double a_lateral;
    rclcpp::Time ins_frame_arrive_time;
    double ins_arrive_at_rs232_buffer;

    bool is_global_path_received = false;
    bool is_ins_data_received = false;

    vector<double> global_path_x; // ptsx
    vector<double> global_path_y; // ptsy
    vector<double> global_path_s; // ptsy

    double car_s;
    double car_d;

    double ins_delay;
    int working_mode;
    double ins_data_arrive_at_sensor_fusion_through_callback;

    
    bool first_iterion = true;
    double run_start_time_stamp = 0;


};

#endif