/*'''*****************************************************************************************************
# FileName    : 
# FileFunction: 订阅惯导的消息，控制rviz2里主车的可视化
# Comments    :
*****************************************************************************************************'''*/

#include <cstdio>
#include <chrono> // C++里面处理时间的包
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/time_source.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "nav_msgs/msg/odometry.hpp"
#include <tf2/convert.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using std::placeholders::_1;
using namespace std::chrono_literals;
/*'''*****************************************************************************************************
# Class Name  : 
# FileFunction: 订阅到定位信息后，
# Comments    :
*****************************************************************************************************'''*/
class HostVehicleVisualization : public rclcpp::Node
{
public:
    /*'''**************************************************************************************
    - FunctionName: None
    - Function    : None
    - Inputs      : None
    - Outputs     : None
    - Comments    : None
    **************************************************************************************'''*/
    HostVehicleVisualization() : Node("host_vehicle_visualization_joint_states")
    {
        // odom_publisher = this->create_publisher<nav_msgs::msg::Odometry>("odom", 50);

        sub = this->create_subscription<nav_msgs::msg::Odometry>("ins_d_of_vehicle_pose", 10, std::bind(&HostVehicleVisualization::imu_position_topic_callback, this, _1));

        publisher_timer_ = this->create_wall_timer(20ms, std::bind(&HostVehicleVisualization::publisher_timer_callback, this)); // 定时器， 定时调用
    }
    /*'''**************************************************************************************
    - FunctionName: None
    - Function    : None
    - Inputs      : None
    - Outputs     : None
    - Comments    : None
    **************************************************************************************'''*/
    ~HostVehicleVisualization(){}

public:

    // rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub; //subscribes position message from imu

    bool new_message = false;

    nav_msgs::msg::Odometry current_position;

    rclcpp::TimerBase::SharedPtr publisher_timer_;

    rclcpp::Time current_time;

    geometry_msgs::msg::TransformStamped odom_translation;

    std::shared_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster;

    /*'''**************************************************************************************
    - FunctionName: None
    - Function    : 订阅到新的定位消息时候的回调函数
    - Inputs      : None
    - Outputs     : None
    - Comments    : None
    **************************************************************************************'''*/
    void imu_position_topic_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        this->current_position = *msg;
        this->new_message = true;
        RCLCPP_INFO(this->get_logger(),"got imu data at: %f, %f", this->now().seconds(), (double)this->now().nanoseconds()/1000000000);
    }
    /*'''**************************************************************************************
    - FunctionName: None
    - Function    : 定时广播器回调函数
    - Inputs      : None
    - Outputs     : None
    - Comments    : None
    **************************************************************************************'''*/
    void publisher_timer_callback()
    {
        current_time = this->get_clock()->now();
        if (new_message)
        {
            // 车辆坐标系与地图坐标系之间的tf2转换关系消息
            odom_translation.header.stamp = current_time;
            odom_translation.header.frame_id = "odom";
            odom_translation.child_frame_id = "base_link";
            odom_translation.transform.rotation = current_position.pose.pose.orientation;
            odom_translation.transform.translation.x = current_position.pose.pose.position.x;
            odom_translation.transform.translation.y = current_position.pose.pose.position.y;
            odom_translation.transform.translation.z = 0; //current_position.pose.pose.position.z;

            odom_broadcaster->sendTransform(odom_translation);

        }
        else
        {
           // 车辆坐标系与地图坐标系之间的tf2转换关系消息
            odom_translation.header.stamp = current_time;
            odom_translation.header.frame_id = "odom";
            odom_translation.child_frame_id = "base_link"; 
            odom_broadcaster->sendTransform(odom_translation);
            RCLCPP_ERROR(this->get_logger(), "no imu message");
        }
    }
    /*'''**************************************************************************************
    - FunctionName: None
    - Function    : None
    - Inputs      : None
    - Outputs     : None
    - Comments    : c++11中的shared_from_this()来源于boost中的enable_shared_form_this类和shared_from_this()函数，
                    功能为返回一个当前类的std::share_ptr,
    **************************************************************************************'''*/
    void init_tf_broadcaster()
    {
        odom_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(shared_from_this());
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto n = std::make_shared<HostVehicleVisualization>();

    n->init_tf_broadcaster(); // 创建 tf 广播器，用于广播坐标变换关系

    RCLCPP_INFO(n->get_logger(), "sdf ");

    rclcpp::spin(n);

    rclcpp::shutdown();
    return 0;
}