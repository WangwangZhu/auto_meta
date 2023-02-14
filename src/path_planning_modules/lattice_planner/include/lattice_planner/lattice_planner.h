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

#include "behavior_decision_interface/msg/fsm_decision_results.hpp"

#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>

#include <uWS/uWS.h>
#include <fstream>
#include <string>
#include <algorithm>

#include <eigen3/unsupported/Eigen/Splines>

#include "optimal_trajectory_generator.h"

#include "common.h"


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
    void fsm_behavior_decision_makeing_callback(behavior_decision_interface::msg::FSMDecisionResults::SharedPtr msg);

    
public:
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_path_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ins_data_subscription_;
    rclcpp::Subscription<behavior_decision_interface::msg::FSMDecisionResults>::SharedPtr fsm_behavior_decision_makeing_subscription;
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
    geometry_msgs::msg::Point lattice_planner_path_cardesian_points;

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

    vector<vector<double>> next_wps_previous;
    vector<double> next_ss_previous;

    vector<Object_Around> sensor_fusion;

    VectorXd coeffs;
    double cte;

    int current_velocity_behavior  = 0;
    int target_lane = 0;

    int host_lane;

    double ref_x;
    double ref_y;
    double ref_yaw;

    double target_v; // km/h
    // float c_speed_, c_d_, c_d_d_, c_d_dd_, s0_;

    void GetWayPoints();
    Vec_f wx_, wy_;
    Spline2D *csp_obj_;
    void GenerateGlobalPath();
    int GetNearestReferenceIndex(const VehicleState &ego_state);    // 根据车辆的当前位置，获取与参考路劲最近点的id
    void UpdateStaticObstacle();
    double GetNearestReferenceLength(const VehicleState &ego_state);
    double GetNearestReferenceLatDist(const VehicleState &ego_state);
    bool LeftOfLine(const VehicleState &p, const geometry_msgs::msg::PoseStamped &p1, const geometry_msgs::msg::PoseStamped &p2);
    // nav_msgs::msg::Path global_plan_;
    // double end_x_, end_y_, end_s_;
    auto createQuaternionMsgFromYaw(double yaw) {
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        return tf2::toMsg(q);
    };
    std::vector<Poi_f> obstcle_list_;
    rclcpp::TimerBase::SharedPtr lattice_planner_timer;
    void LatticePlannerCallback();
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr replan_path_publisher_;
    TrajectoryData GetTrajectoryFromFrenetPath(const FrenetPath &path);
    float c_speed_ = 10.0 / 3.6;
    float c_d_ = 0;
    float c_d_d_ = 0.0;
    float c_d_dd_ = 0.0;
    float s0_ = 0.0;
    nav_msgs::msg::Path global_plan_;
    double end_x_, end_y_, end_s_;
    bool near_goal_ = false;
    bool use_reference_line_ = false;
    TrajectoryData planningPublishedTrajectoryDebug_;    //规划下发的轨迹
    TrajectoryData last_trajectory_;                     //规划下发的轨迹
    bool plannerFlag_ = false;

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

    TrajectoryData planning_published_trajectory;
    TrajectoryPoint goal_point;
    std::vector<TrajectoryPoint> trajectory_points_;

    std::string roadmap_path;
    double speed_P, speed_I, speed_D, target_speed;

    VehicleState vehicleState_;

    TrajectoryData global_reference_trajectory;

    bool first_receive_global_path = true;

    template <typename U, typename V>
    double DistanceXY(const U &u, const V &v) {
        return std::hypot(u.x - v.x, u.y - v.y);
}

};

#endif