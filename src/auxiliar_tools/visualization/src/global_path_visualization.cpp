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
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <std_msgs/msg/float32.hpp>

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
    geometry_msgs::msg::PoseStamped this_pose_stamped;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr global_path_publisher_;
    rclcpp::TimerBase::SharedPtr global_path_publisher_timer_;

    rclcpp::TimerBase::SharedPtr target_velocity_from_csv_timer;
    rclcpp::Publisher<std_msgs::msg::Float32> target_velocity_from_csv_publisher;

    string global_path_path;
    string path_configure_parameter;
    bool load_map_done = false;

public:
    /*'''**************************************************************************************
    - FunctionName: None
    - Function    : 定时广播全局路径出去
    - Inputs      : None
    - Outputs     : None
    - Comments    : None
    **************************************************************************************'''*/
    GlobalPathVisualization() : Node("goobal_path_visualization")
    {
        RCLCPP_INFO(this->get_logger(), "publishing global path.");
        global_path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("global_path", 10);

        global_path_publisher_timer_ = this->create_wall_timer(1000ms, std::bind(&GlobalPathVisualization::global_path_publisher_timer_callback, this));

        this->declare_parameter<string>("global_map_name_parameter", path_configure_parameter);
    }

    /*'''**************************************************************************************
    - FunctionName: None
    - Function    : None
    - Inputs      : None
    - Outputs     : None
    - Comments    : None
    **************************************************************************************'''*/
    ~GlobalPathVisualization() {}

    /*'''**************************************************************************************
    - FunctionName: None
    - Function    : None
    - Inputs      : None
    - Outputs     : None
    - Comments    : None
    **************************************************************************************'''*/
    void global_path_publisher_timer_callback()
    {
        if (this->load_map_done)
        {
            RCLCPP_INFO(this->get_logger(), "publishing global path");

            global_path.header.stamp = this->get_clock()->now();
            global_path.header.frame_id = "odom";
            
            global_path_publisher_->publish(global_path);
        }
    }
    /*'''**************************************************************************************
    - FunctionName: None
    - Function    : 模板函数：将string类型变量转换为常用的数值类型（此方法具有普遍适用性）
    - Inputs      : None
    - Outputs     : None
    - Comments    : None
    **************************************************************************************'''*/
    template <class Type>
    Type stringToNum(const string &str)
    {
        std::istringstream iss(str);
        Type num;
        iss >> num;
        return num;
    }
    /*'''**************************************************************************************
    - FunctionName: None
    - Function    : 模板函数
    - Inputs      : None
    - Outputs     : None
    - Comments    : None
    **************************************************************************************'''*/
    template <class Type>
    vector<Type> string_split(const string &str, const char &pattern)
    {
        vector<Type> res;
        if (str == "")
        {
            return res;
        }
        string strs = str + pattern; // 在字符串末尾加入分割符号，方便截取最后一段。
        size_t pos = strs.find(pattern);
        while (pos != strs.npos)
        {
            string temp_string = strs.substr(0, pos);
            Type temp_number = stringToNum<Type>(temp_string);
            res.push_back(temp_number);
            strs = strs.substr(pos + 1, strs.size()); // 去掉已分割的字符串,在剩下的字符串中进行分割
            pos = strs.find(pattern);
        }
        return res;
    }
    /*'''**************************************************************************************
    - FunctionName: None
    - Function    : None
    - Inputs      : None
    - Outputs     : None
    - Comments    : None
    **************************************************************************************'''*/
    void load_map()
    {
        char *buffer;
        if ((buffer = getcwd(NULL, 0)) == NULL) {
            perror("getcwd error");
        }
        else {
            string buffer_ = buffer;
            string local_path;
            this->get_parameter<string>("global_map_name_parameter", local_path);
            global_path_path = buffer_ + local_path;
            cout << "global_path_path" << global_path_path << endl;
        }

        ifstream infile(global_path_path);
        string value;
        int i = 0;
        getline(infile, value); // 舍弃头
        while (infile.good()) {
            cout << "加载全局地图" << endl;
            getline(infile, value);
            if (value != "")
            {
                cout << "string value : " << value << endl;
                cout.precision(12);
                vector<double> temp_values = string_split<double>(value, ',');

                this_pose_stamped.header.frame_id = "odom";             
                this_pose_stamped.header.stamp = this->get_clock()->now();
                this_pose_stamped.pose.position.x = temp_values[2];
                this_pose_stamped.pose.position.y = temp_values[3];
                this_pose_stamped.pose.position.z = 0;
                this_pose_stamped.pose.orientation.x = 0;
                this_pose_stamped.pose.orientation.y = 0;
                this_pose_stamped.pose.orientation.z = 0;
                this_pose_stamped.pose.orientation.w = temp_values[7]; // 这里实际上是放的frenet坐标系的S
                
                global_path.poses.push_back(this_pose_stamped);

                i++;
                if (i == 10)
                {
                    // break;
                }
            }
        }
        load_map_done = true;
    }
};

/*'''*****************************************************************************************************
# Function Name  : 
# FileFunction   : 
# Comments       :
*****************************************************************************************************'''*/
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto n = std::make_shared<GlobalPathVisualization>();

    RCLCPP_INFO(n->get_logger(), "running");

    n->load_map();

    rclcpp::spin(n);
    rclcpp::shutdown();

    return 0;
}
