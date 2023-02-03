#ifndef A_STAR_PLANNER_H_
#define A_STAR_PLANNER_H_

#define BOOST_BIND_NO_PLACEHOLDERS

#include <math.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <stdint.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

#include <algorithm>
// #include <backward.hpp>
#include <chrono>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <cstdio>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/QR>
#include <eigen3/unsupported/Eigen/Splines>
#include <fstream>
#include <functional>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <iomanip>
#include <iostream>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32.hpp>
#include <string>
#include <thread>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "carla_msgs/msg/carla_ego_vehicle_control.hpp"
#include "carla_msgs/msg/carla_ego_vehicle_status.hpp"
#include "carla_msgs/msg/carla_status.hpp"
#include "carla_msgs/msg/carla_vehicle_target_velocity.hpp"
#include "rclcpp/rate.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "Astar_searcher.h"

#include "Eigen/LU"
#include "math.h"
#include "std_msgs/msg/string.hpp"

// using namespace std::chrono_literals;    // 表示时间长度的命名空间
//调用嵌套空间std::chrono_literals下的函数
using std::cout;
using std::endl;
using std::vector;
// using std::placeholders::_1;

using CppAD::AD;
using Eigen::VectorXd;
using Eigen::Vector3d;

// using namespace std;
// using namespace Eigen;
// namespace backward {
// backward::SignalHandling sh;
// }

template <typename U, typename V>
double DistanceXY(const U& u, const V& v) {
    return std::hypot(u.x - v.x, u.y - v.y);
}

class AStarPlannerNode : public rclcpp::Node {
   public:
    AStarPlannerNode();
    ~AStarPlannerNode();

    // simulation param from launch file
    double _resolution, _inv_resolution, _cloud_margin;
    double _x_size, _y_size, _z_size;

    // useful global variables
    bool _has_map = false;

    int received_global_map_count = 0;

    Vector3d _start_pt;
    Vector3d _map_lower, _map_upper;
    int _max_x_id, _max_y_id, _max_z_id;

    // ros related
    // ros::Subscriber _map_sub, _pts_sub;
    // ros::Publisher _grid_path_vis_pub, _visited_nodes_vis_pub, _grid_map_vis_pub;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _map_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _pts_sub;

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr _grid_path_vis_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr _visited_nodes_vis_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _grid_map_vis_pub;

    // 定义了结构体 AstarPathFinder 变量 _astar_path_finder，该结构体存储、实现了 Astar 路径规划所需的所有信息和功能
    AstarPathFinder* _astar_path_finder = new AstarPathFinder();

    void rcvWaypointsCallback(geometry_msgs::msg::PoseStamped::SharedPtr wp);
    void rcvPointCloudCallBack(sensor_msgs::msg::PointCloud2::SharedPtr pointcloud_map);

    void visGridPath(vector<Vector3d> nodes, bool is_use_jps);
    void visVisitedNode(vector<Vector3d> nodes);
    void pathFinding(const Vector3d start_pt, const Vector3d target_pt);

    //计算两点之间的距离
    double pointDistance(const double x1, const double y1, const double x, const double y) {
        double dx = x1 - x;
        double dy = y1 - y;
        return sqrt(dx * dx + dy * dy);
    };
};
#endif /*  */
