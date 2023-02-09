#ifndef VISUALIZATIUON_H_
#define VISUALIZATIUON_H_


#include <cstdio>
#include <chrono> // C++里面处理时间的包
#include <memory>
#include <string>
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <stdio.h>
#include <sstream>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/time_source.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "nav_msgs/msg/odometry.hpp"
#include <tf2/convert.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>

#include "std_msgs/msg/float32.hpp"

#include "visualization/utils.h"


using namespace std::chrono_literals;
using std::cout;
using std::endl;
using std::ifstream;
using std::string;
using std::vector;

/*'''*****************************************************************************************************
# Class Name  : 
# FileFunction: 
# Comments    :
*****************************************************************************************************'''*/
class GlobalPathVisualization : public rclcpp::Node
{
public:
    nav_msgs::msg::Path global_path;
    nav_msgs::msg::Path global_path_offset;
    geometry_msgs::msg::PoseStamped this_pose_stamped;

    vector<double> velocity_curve;

    vector<double> global_path_x; // ptsx
    vector<double> global_path_y; // ptsy
    vector<double> global_path_psi; // ptsy
    vector<double> global_path_s;

    vector<double> global_path_x_down_sample; // ptsx
    vector<double> global_path_y_down_sample; // ptsy
    vector<double> global_path_psi_down_sample; // ptsy
    vector<double> global_path_s_down_sample;

    vector<double> global_path_x_offset; // ptsx
    vector<double> global_path_y_offset; // ptsy

    
    visualization_msgs::msg::MarkerArray global_path_multi_lines;
    

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr global_path_multi_lines_publisher;
    rclcpp::TimerBase::SharedPtr global_path_multi_lines_publisher_timer_;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr global_path_publisher_;
    rclcpp::TimerBase::SharedPtr global_path_publisher_timer_;


    rclcpp::TimerBase::SharedPtr target_velocity_from_csv_timer;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr target_velocity_from_csv_publisher;
    std_msgs::msg::Float32 velocity_target_from_csv;


    string global_path_path;
    string path_configure_parameter;
    string velocity_wyx_curve;
    bool load_map_done_global = false;
    bool load_velocity_curve_done = false;

    int i = 0;

public:
    GlobalPathVisualization();

    ~GlobalPathVisualization() ;

    void global_path_publisher_timer_callback() ;
    void global_path_multi_lines_publisher_timer_callback();

    void generate_road_structure(double lane_d_coordinate, int lane_id, int line_type_flag);

    void target_velocity_publisher_callback();

    template <class Type>
    Type stringToNum(const string &str);
    template <class Type>
    vector<Type> string_split(const string &str, const char &pattern);
    void load_map();
    void load_velocity_curve();
};
#endif