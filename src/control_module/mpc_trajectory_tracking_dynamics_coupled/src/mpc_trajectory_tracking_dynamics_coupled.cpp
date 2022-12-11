/*'''*****************************************************************************************************
# FileName    : 
# FileFunction: 产生控制信号发送到 ROS 网络上。
# Comments    : 
*****************************************************************************************************'''*/

#include "mpc_trajectory_tracking_dynamics_coupled/helpers.h"
#include "mpc_trajectory_tracking_dynamics_coupled/mpc_controller_trajectory_tracking_dynamics_coupled.h"
#include "mpc_trajectory_tracking_dynamics_coupled/spline.h"
#include "mpc_trajectory_tracking_dynamics_coupled/coordinate_transform.h"

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mpc_trajectory_tracking_publisher");

double inline min_mpc(double a, double b) { return (a < b) ? a : b; }
double inline max_mpc(double a, double b) { return (a > b) ? a : b; }

/*'''**************************************************************************************
- FunctionName: None
- Function    : 构造函数
- Inputs      : None
- Outputs     : None
- Comments    : None
**************************************************************************************'''*/
MpcTrajectoryTrackingPublisher::MpcTrajectoryTrackingPublisher() : Node("mpc_trajectory_tracking_publisher") // 使用初始化列表来初始化字段
{
    // vehicle_control_gas_brake_steer_msg.adu_gear_req = 1;
    vehicle_control_gas_brake_steer_msg.adu_brk_stoke_req = 0;
    vehicle_control_gas_brake_steer_msg.adu_gas_stoke_req = 0;
    vehicle_control_gas_brake_steer_msg.adu_str_whl_ang_req = 0;

    vehicle_control_gear_msg.gear_request = 1;

    int qos_ = 2; // 定义消息队列大小为2，若超过2，则最先进入的消息将被舍弃，发布与订阅均有队列

    old_steer_value = 0;
    old_throttle_value = 0;

    // 定义广播器，主题的名字和转发节点里面的主题名字对应。
    mpc_control_signals_gas_brake_steer_publisher = this->create_publisher<chassis_msg::msg::ADUDriveCmd>("vehicle_control_signals_gas_brake_steer", qos_);

    mpc_control_signals_gear_publisher = this->create_publisher<chassis_msg::msg::ADUGearRequest>("vehicle_control_signals_gear_request", qos_);
    

    // mpc 求解出来的未来一段时间的路径，以及mpc使用的未来一段时间的参考轨迹点
    mpc_reference_path_publisher = this->create_publisher<visualization_msgs::msg::Marker>("mpc_reference_path", qos_);
    mpc_output_path_publisher = this->create_publisher<visualization_msgs::msg::Marker>("mpc_output_path", qos_);
    history_path_visualization_publisher = this->create_publisher<nav_msgs::msg::Path>("history_path", qos_); // move to visualization package
    
    ins_data_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>("ins_d_of_vehicle_pose", qos_, std::bind(&MpcTrajectoryTrackingPublisher::ins_data_receive_callback, this, _1));
    eps_feedback_subscription = this->create_subscription<chassis_msg::msg::WVCUHorizontalStatus>("wvcu_horizontal_status", qos_, std::bind(&MpcTrajectoryTrackingPublisher::eps_feedback_callback, this, _1));
    global_path_subscription = this->create_subscription<nav_msgs::msg::Path>("global_path", qos_, std::bind(&MpcTrajectoryTrackingPublisher::global_path_callback, this, _1));
    
    mpc_iteration_timer_ = this->create_wall_timer(10ms, std::bind(&MpcTrajectoryTrackingPublisher::mpc_tracking_iteration_callback, this));

    mpc_iteration_time_publisher = this->create_publisher<std_msgs::msg::Float32>("mpc_iteration_duration", qos_); // 用于统计MPC求解时间的广播器

    mpc_planner_frenet_path_subscription = this->create_subscription<nav_msgs::msg::Path>("highway_with_prediction_planner_path_frenet", qos_, std::bind(&MpcTrajectoryTrackingPublisher::palnner_frenet_path_receive_callback, this, _1));

    mpc_planner_cartesian_path_subscription = this->create_subscription<visualization_msgs::msg::Marker>("highway_with_prediction_planner_path_cardesian", qos_, std::bind(&MpcTrajectoryTrackingPublisher::palnner_cartesian_path_receive_callback, this, _1));

    vehicle_longitudinal_status_feedback = this->create_subscription<chassis_msg::msg::WVCULongitudinalStatus>("wvcu_longitudinal_status", qos_,std::bind(&MpcTrajectoryTrackingPublisher::vehicle_status_feedback_callback, this, _1));

    // Declare and initialize a parameter, return the effective value.
    this->declare_parameter<double>("vehicle_ref_v", target_v);
    this->declare_parameter<double>("vehicle_steering_ratio_double", steering_ratio);
    this->declare_parameter<double>("vehicle_Lf_double", kinamatic_para_Lf);
    this->declare_parameter<int>("mpc_control_horizon_length_int", mpc_control_horizon_length);
    this->declare_parameter<double>("mpc_control_step_length_double", mpc_control_step_length);
    this->declare_parameter<bool>("mpc_tracking_enable_bool", mpc_tracking_enable_bool);
    this->declare_parameter<int>("mpc_cte_weight_int", mpc_cte_weight_int);
    this->declare_parameter<int>("mpc_epsi_weight_int", epsi_weight);
    this->declare_parameter<int>("mpc_v_weight_int", v_weight);
    this->declare_parameter<int>("mpc_steer_actuator_cost_weight_int", steer_actuator_cost_weight);
    this->declare_parameter<int>("mpc_acc_actuator_cost_weight_int", acc_actuator_cost_weight);
    this->declare_parameter<int>("mpc_change_steer_cost_weight_int", change_steer_cost_weight);
    this->declare_parameter<int>("mpc_change_accel_cost_weight_int", change_accel_cost_weight);
    this->declare_parameter<double>("mpc_reference_path_length", reference_path_length);
    this->declare_parameter<double>("mpc_controller_delay_compensation", controller_delay_compensation);
    this->declare_parameter<double>("mpc_point_distance_of_reference_line_visualization", point_distance_of_reference_line_visualization);
    this->declare_parameter<int>("mpc_points_number_of_reference_line_visualization", points_number_of_reference_line_visualization);
    this->declare_parameter<int>("mpc_former_point_of_current_position", former_point_of_current_position);
    this->declare_parameter<int>("mpc_working_mode", working_mode);
    this->declare_parameter<int> ("mpc_with_planner_flag", with_planner_flag);

    // Get the value of a parameter by the given name, and return true if it was set.
    this->get_parameter<double>("vehicle_ref_v", this->target_v);
    this->get_parameter<double>("vehicle_steering_ratio_double", this->steering_ratio);
    this->get_parameter<double>("vehicle_Lf_double", this->kinamatic_para_Lf);
    this->get_parameter<int>("mpc_control_horizon_length_int", this->mpc_control_horizon_length);
    this->get_parameter<double>("mpc_control_step_length_double", this->mpc_control_step_length);
    this->get_parameter<bool>("mpc_tracking_enable_bool", this->mpc_enable_signal);
    this->get_parameter<int>("mpc_cte_weight_int", this->cte_weight);
    this->get_parameter<int>("mpc_epsi_weight_int", this->epsi_weight);
    this->get_parameter<int>("mpc_v_weight_int", this->v_weight);
    this->get_parameter<int>("mpc_steer_actuator_cost_weight_int", this->steer_actuator_cost_weight);
    this->get_parameter<int>("mpc_acc_actuator_cost_weight_int", this->acc_actuator_cost_weight);
    this->get_parameter<int>("mpc_change_steer_cost_weight_int", this->change_steer_cost_weight);
    this->get_parameter<int>("mpc_change_accel_cost_weight_int", this->change_accel_cost_weight);
    this->get_parameter<double>("mpc_reference_path_length", this->reference_path_length);
    this->get_parameter<double>("mpc_controller_delay_compensation", this->controller_delay_compensation);
    this->get_parameter<double>("mpc_point_distance_of_reference_line_visualization", this->point_distance_of_reference_line_visualization);
    this->get_parameter<int>("mpc_points_number_of_reference_line_visualization", this->points_number_of_reference_line_visualization);
    this->get_parameter<int>("mpc_former_point_of_current_position", this->former_point_of_current_position);
    this->get_parameter<int>("mpc_working_mode", this->working_mode);
    this->get_parameter<int>("mpc_with_planner_flag", this->with_planner_flag);

    RCLCPP_INFO(this->get_logger(), "target_v %f", this->target_v);
    RCLCPP_INFO(this->get_logger(), "vehicle_steering_ratio_double %f", this->steering_ratio);
    RCLCPP_INFO(this->get_logger(), "vehicle_Lf_double %f", this->kinamatic_para_Lf);
    RCLCPP_INFO(this->get_logger(), "mpc_control_horizon_length_int %d", this->mpc_control_horizon_length);
    RCLCPP_INFO(this->get_logger(), "mpc_control_step_length_double %f", this->mpc_control_step_length);
    RCLCPP_INFO(this->get_logger(), "mpc_tracking_enable_bool %d", int(this->mpc_enable_signal));
    RCLCPP_INFO(this->get_logger(), "mpc_cte_weight_int %d", this->cte_weight);
    RCLCPP_INFO(this->get_logger(), "mpc_epsi_weight_int %d", this->epsi_weight);
    RCLCPP_INFO(this->get_logger(), "mpc_v_weight_int %d", this->v_weight);
    RCLCPP_INFO(this->get_logger(), "mpc_steer_actuator_cost_weight_int %d", this->steer_actuator_cost_weight);
    RCLCPP_INFO(this->get_logger(), "mpc_acc_actuator_cost_weight_int %d", this->acc_actuator_cost_weight);
    RCLCPP_INFO(this->get_logger(), "mpc_change_steer_cost_weight_int %d", this->change_steer_cost_weight);
    RCLCPP_INFO(this->get_logger(), "mpc_change_accel_cost_weight_int %d", this->change_accel_cost_weight);
    RCLCPP_INFO(this->get_logger(), "mpc_reference_path_length %f", this->reference_path_length);
    RCLCPP_INFO(this->get_logger(), "mpc_controller_delay_compensation %f", this->controller_delay_compensation);
    RCLCPP_INFO(this->get_logger(), "mpc_point_distance_of_reference_line_visualization %f", this->point_distance_of_reference_line_visualization);
    RCLCPP_INFO(this->get_logger(), "mpc_points_number_of_reference_line_visualization %d", this->points_number_of_reference_line_visualization);
    RCLCPP_INFO(this->get_logger(), "mpc_former_point_of_current_position %d", this->former_point_of_current_position);
    RCLCPP_INFO(this->get_logger(), "mpc_working_mode %d", this->working_mode);
    RCLCPP_INFO(this->get_logger(), "mpc_with_planner_flag %d", this->with_planner_flag);

    RCLCPP_WARN(this->get_logger(), "INIT DONE~~~~~");
}

/*'''**************************************************************************************
- FunctionName: None
- Function    : None
- Inputs      : None
- Outputs     : None
- Comments    : None
**************************************************************************************'''*/
MpcTrajectoryTrackingPublisher::~MpcTrajectoryTrackingPublisher() {}
/*'''**************************************************************************************
- FunctionName: None
- Function    : None
- Inputs      : None
- Outputs     : None
- Comments    : 规划器规划结果订阅器，规划器给的结果是全局笛卡尔坐标系下的XY坐标点
**************************************************************************************'''*/    
void MpcTrajectoryTrackingPublisher::palnner_frenet_path_receive_callback(nav_msgs::msg::Path::SharedPtr msg)
{
    // RCLCPP_INFO(this->get_logger(), "receiving planner frenet_path %lu", msg->poses.size());
    
    int path_length = msg->poses.size();

    planner_path_s.clear();
    planner_path_v.clear();

    for (int i = 0; i < path_length; i++)
    {
        planner_path_s.push_back(msg->poses[i].pose.position.x);
        planner_path_v.push_back(msg->poses[i].pose.position.y);
    }

    is_planner_frenet_path_received = true;
}

/*'''**************************************************************************************
- FunctionName: None
- Function    : None
- Inputs      : None
- Outputs     : None
- Comments    : 
**************************************************************************************'''*/    
void MpcTrajectoryTrackingPublisher::vehicle_status_feedback_callback(chassis_msg::msg::WVCULongitudinalStatus::SharedPtr msg)
{
    is_vehicle_longitudinal_received = true;
    vehicle_longitudinal_feedback_msg = msg;
    // RCLCPP_INFO(this->get_logger(), "current gear %d", vehicle_longitudinal_feedback_msg->wvcu_gear_stat);
}

void MpcTrajectoryTrackingPublisher::palnner_cartesian_path_receive_callback(visualization_msgs::msg::Marker::SharedPtr msg)
{
    // RCLCPP_INFO(this->get_logger(), "receiving planner cartesian path %lu", msg->points.size());
    
    int path_length = msg->points.size();

    planner_path_x.clear();
    planner_path_y.clear();

    for (int i = 0; i < path_length; i++)
    {
        planner_path_x.push_back(msg->points[i].x);
        planner_path_y.push_back(msg->points[i].y);
    }

    is_planner_cartesian_path_received = true;
}
/*'''**************************************************************************************
- FunctionName: None
- Function    : None
- Inputs      : None
- Outputs     : None
- Comments    : None
**************************************************************************************'''*/
void MpcTrajectoryTrackingPublisher::global_path_callback(nav_msgs::msg::Path::SharedPtr msg)
{
    // RCLCPP_INFO(this->get_logger(), "receiveing global path %lu", msg->poses.size());
    int path_length = msg->poses.size();

    global_path_x.clear();
    global_path_y.clear();

    for (int i = 0; i < path_length; i++)
    {
        global_path_x.push_back(msg->poses[i].pose.position.x);
        global_path_y.push_back(msg->poses[i].pose.position.y);
    }

    is_global_path_received = true;
}
/*'''**************************************************************************************
- FunctionName: None
- Function    : None
- Inputs      : None
- Outputs     : None
- Comments    : 这个回调函数发出去的psi的值应该是弧度单位的
**************************************************************************************'''*/
void MpcTrajectoryTrackingPublisher::ins_data_receive_callback(nav_msgs::msg::Odometry::SharedPtr msg)
{
    if (is_global_path_received)
    {
        rclcpp::Time now = this->now();
        ins_data_arrive_at_mpc_through_callback = now.seconds(); //  + now.nanoseconds()/1000000000
        // RCLCPP_INFO(this->get_logger(),"got imu data at: %f", this->now().seconds()); // this->now().nanoseconds()/1000000000

        tf2::Quaternion quat_tf;
        tf2::convert(msg->pose.pose.orientation, quat_tf);
        double roll_current, pitch_current, heading_current;
        a_longitudinal = msg->pose.covariance[0];
        a_lateral = msg->pose.covariance[4];
        px = msg->pose.pose.position.x;
        py = msg->pose.pose.position.y; // 惯导的y向前
        this->v_longitudinal = msg->twist.twist.linear.x;
        v_lateral = msg->twist.twist.linear.y;
        yaw_rate = msg->twist.twist.angular.z;
        if (v_longitudinal < 0.4/3.6)
        {
            v_lateral = 0;
            yaw_rate = 0;
        }
        RCLCPP_INFO(this->get_logger(), "velocity receiveing from ins: %f", this->v_longitudinal);

        ins_frame_arrive_time = msg->header.stamp;
        ins_arrive_at_rs232_buffer = this->ins_frame_arrive_time.seconds(); //  + this->ins_frame_arrive_time.nanoseconds()/1000000000
        if (1)
        {
            tf2::Matrix3x3(quat_tf).getRPY(roll_current, pitch_current, heading_current);
            psi = heading_current;
        }
        else
        {
            roll_current = msg->pose.covariance[3];
            pitch_current = msg->pose.covariance[2];
            heading_current = msg->pose.covariance[1];

            psi = heading_current;
        }

        is_ins_data_received = true;

        // 获取车辆在 frenet 坐标系的定位信息
        vector<double> car_s_d = cartesian_to_frenet(px, py, psi, global_path_x, global_path_y);
        car_s = car_s_d[0];
        car_d = car_s_d[1];

        /* 将收到的定位信息发布出来,在rviz里显示历史轨迹 */
        history_path.header.stamp = this->get_clock()->now();
        history_path.header.frame_id = "odom";

        history_path_points.header.stamp = this->get_clock()->now();
        history_path_points.header.frame_id = "odom";
        history_path_points.pose.position.x = px;
        history_path_points.pose.position.y = py;
        history_path_points.pose.position.z = 0;
        history_path_points.pose.orientation = msg->pose.pose.orientation;
        history_path.poses.push_back(history_path_points);

        if (history_path.poses.size() > 2000)
        {
            vector<geometry_msgs::msg::PoseStamped>::iterator k = history_path.poses.begin();
            history_path.poses.erase(k);
        }
        history_path_visualization_publisher->publish(history_path);
    }
}
/*'''**************************************************************************************
- FunctionName: None
- Function    : None
- Inputs      : None
- Outputs     : None
- Comments    : None
**************************************************************************************'''*/
void MpcTrajectoryTrackingPublisher::eps_feedback_callback(chassis_msg::msg::WVCUHorizontalStatus::SharedPtr msg)
{
    delta = deg2rad(msg->wvcu_str_whl_ang_stat) / steering_ratio;
    RCLCPP_ERROR(this->get_logger(), "receiving eps angel: %f", delta);
    is_eps_received = true;
}

/*'''**************************************************************************************
- FunctionName: None
- Function    : None
- Inputs      : None
- Outputs     : None
- Comments    : None
**************************************************************************************'''*/
void MpcTrajectoryTrackingPublisher::mpc_tracking_iteration_callback()
{
    // 直接发控制信号给底盘，测试底盘是否正常
    // vehicle_control_gas_brake_steer_msg.adu_gear_req = 3;
    // vehicle_control_gas_brake_steer_msg.adu_brk_stoke_req = 40;
    vehicle_control_gas_brake_steer_msg.adu_gas_stoke_req = 0;
    // vehicle_control_gas_brake_steer_msg.adu_str_whl_ang_req = 0;
    vehicle_control_gear_msg.gear_request = 3;

    rclcpp::Time start_mpc;
    rclcpp::Time end_mpc;
    start_mpc = this->now();
    double iteration_time_length;

    if (is_vehicle_longitudinal_received){
        if (rclcpp::ok())
        // if (0) // 失能跟踪功能，测试控制信号是否起效
        {
            this->reference_path_length = max_mpc(floor(this->mpc_control_horizon_length * this->mpc_control_step_length * this->v_longitudinal) + 1, 20.0);

            // RCLCPP_INFO(this->get_logger(), "reference_path_length: %f", this->reference_path_length);

            // if (is_eps_received && is_global_path_received && is_ins_data_received && is_planner_frenet_path_received && is_planner_cartesian_path_received)
            if (is_eps_received && is_global_path_received && is_ins_data_received)
            {
                global_path_remap_x.clear();
                global_path_remap_y.clear();

                planner_path_remap_x.clear();
                planner_path_remap_y.clear();

                rclcpp::Time now = this->now();
                double ins_parse_now = now.seconds(); 
                // 定位延迟补偿发生在将全局路径转换到车辆坐标系下之前,用来补偿定位信息到达早于被使用而引起的定位误差
                if (working_mode == 1)
                {
                    ins_delay = ins_parse_now - ins_data_arrive_at_mpc_through_callback + 0.005; // + 0.05 ins_arrive_at_rs232_buffer
                }
                if (working_mode == 2)
                {
                    ins_delay = ins_parse_now - ins_arrive_at_rs232_buffer + 0.005;
                }
                /* 全局坐标系下的定位信息延时补偿 */
                psi = psi + yaw_rate * ins_delay;
                // cout << "v_longitudinal: " << v_longitudinal << ", ins_delay:" << ins_delay << ", v_lateral:" << v_lateral << ", psi:" << psi*57.296 << endl; 
                px = px + v_longitudinal * cos(psi) * ins_delay - v_lateral * sin(psi) * ins_delay;
                py = py + v_longitudinal * sin(psi) * ins_delay + v_lateral * cos(psi) * ins_delay;

                // 运动补偿后的Frenet坐标系中的车辆定位信息
                vector<double> car_s_d = cartesian_to_frenet(px, py, psi, global_path_x, global_path_y);
                car_s = car_s_d[0];
                car_d = car_s_d[1];

                VectorXd coeffs;
                double cte;
                // 使用全局路径作为跟踪控制器的参考路径(这里可以进一步压缩时间的，每次都转换整条路径时间代价太大而且没用)
                if (with_planner_flag == 0)
                {
                    for (size_t i = 0; i < global_path_x.size() - 1; i++)
                    {
                        double shift_x = global_path_x[i] - px;
                        double shift_y = global_path_y[i] - py;
                        global_path_remap_x.push_back(  shift_x * cos(psi) + shift_y * sin(psi));
                        global_path_remap_y.push_back(- shift_x * sin(psi) + shift_y * cos(psi));
                    }
                    // 从全局路径中，找到距离当前位置最近的前方的点。
                    for (size_t i = former_point_of_current_position; i < global_path_remap_x.size(); i++)
                    {
                        if (global_path_remap_x[i] > 0.0)
                        {
                            former_point_of_current_position = i;
                            break;
                        }            
                    }
                    /* Convert to Eigen::VectorXd */
                    /* 使用map函数，可以实现eigen的矩阵和C++中的数组直接转换 */
                    // 在不进行数据拷贝的情况下，使用map进行内存映射，映射前后的数据类型需要保持一致
                    reference_path_points_number = (int)reference_path_length / 0.6; // 纯跟踪，使用的全局路径点距离为0.6m
                    // cout << "**************here: " << former_point_of_current_position << " " << reference_path_points_number << endl;
                    double *ptrx = &global_path_remap_x[former_point_of_current_position];
                    Eigen::Map<VectorXd> ptsx_transform(ptrx, reference_path_points_number);
                    double *ptry = &global_path_remap_y[former_point_of_current_position];
                    Eigen::Map<VectorXd> ptsy_transform(ptry, reference_path_points_number);            
                    /* Fit coefficients of fifth order polynomial*/
                    coeffs = polyfit(ptsx_transform, ptsy_transform, 5);
                    cte = polyeval(coeffs, 0); // 在车辆坐标系下，当前时刻的位置偏差就是期望轨迹在车辆坐标系中的截距
                }
                // **************************** 使用规划器重规划路径作为跟踪控制器的参考路径 ******************************
                else
                {
                    reference_path_points_number = 0;
                    int planner_path_former_point_of_current_position = 0;
                    for (size_t i = 0; i < planner_path_x.size() - 1; i++)
                    {
                        // cout << "000000000000000planner_path_x" << planner_path_x[i] << "   " << planner_path_y[i] << endl;
                        double shift_x = planner_path_x[i] - px;
                        double shift_y = planner_path_y[i] - py;
                        planner_path_remap_x.push_back(  shift_x * cos(psi) + shift_y * sin(psi));
                        planner_path_remap_y.push_back(- shift_x * sin(psi) + shift_y * cos(psi));
                    }
                    // 从局部路径中，找到距离当前位置最近的前方的点。
                    for (size_t i = planner_path_former_point_of_current_position; i < planner_path_remap_x.size(); i++)
                    {
                        if (planner_path_remap_x[i] > 0.0)
                        {
                            planner_path_former_point_of_current_position = i;
                            break;
                        }            
                    }
                    // cout << "planner_path_former_point_of_current_position" << planner_path_former_point_of_current_position << endl;
                    /* Convert to Eigen::VectorXd */
                    /* 使用map函数，可以实现eigen的矩阵和C++中的数组直接转换 */
                    // 在不进行数据拷贝的情况下，使用map进行内存映射，映射前后的数据类型需要保持一致
                    double temp_total_distance = 0;
                    while (temp_total_distance <= this->reference_path_length)
                    {
                        temp_total_distance += distance_two_point(planner_path_x[reference_path_points_number], 
                                                                    planner_path_y[reference_path_points_number], 
                                                                    planner_path_x[reference_path_points_number+1], 
                                                                    planner_path_y[reference_path_points_number+1]);
                        reference_path_points_number ++;
                    }
                    rclcpp::Time here1 = this->now();
                    double *ptrx = &planner_path_remap_x[planner_path_former_point_of_current_position];
                    Eigen::Map<Eigen::VectorXd> ptsx_transform(ptrx, reference_path_points_number);
                    double *ptry = &planner_path_remap_y[planner_path_former_point_of_current_position];
                    Eigen::Map<Eigen::VectorXd> ptsy_transform(ptry, reference_path_points_number);            
                    /* Fit coefficients of fifth order polynomial*/
                    coeffs = polyfit(ptsx_transform, ptsy_transform, 5);
                    cte    = polyeval(coeffs, 0); // 在车辆坐标系下，当前时刻的位置偏差就是期望轨迹在车辆坐标系中的截距
                    
                    // 速度                
                    Eigen::VectorXd _planner_path_s(planner_path_s.size());
                    Eigen::VectorXd _planner_path_v(planner_path_v.size());
                    for (uint i = 0; i < planner_path_s.size(); i++)
                    {
                        _planner_path_s[i] = planner_path_s[i];
                        _planner_path_v[i] = planner_path_v[i];
                    }
                    for (uint i = 0; i < _planner_path_s.size(); i++)
                    {
                        // cout << std::setprecision(6) << _planner_path_s[i] << "   " << _planner_path_v[i] << endl;;
                    }
                    // cout << endl;
                    auto coeffs_s_v = polyfit(_planner_path_s, _planner_path_v, 5);
                    target_v = polyeval(coeffs_s_v, car_s);
                    // cout << car_s << "*****************************  " << target_v << endl;
                    
                    // 确定转向阻尼参数
                    cal_heading_ratio.clear();
                    cal_heading_ratio_diff.clear();
                    for (uint j = 1; j < ptsx_transform.size(); j++)
                    {
                        cal_heading_ratio.push_back(atan2(ptsy_transform[j] - ptsy_transform[j-1], ptsx_transform[j] - ptsx_transform[j-1]));
                    }
                    for (uint k = 1; k < cal_heading_ratio.size()-1; k++)
                    {
                        cal_heading_ratio_diff.push_back(fabs(cal_heading_ratio[k+1] - cal_heading_ratio[k-1])); 
                    }
                    std::sort(cal_heading_ratio_diff.begin(), cal_heading_ratio_diff.end());

                    steering_damp = 1 - fabs(30 * cal_heading_ratio_diff.back()) + target_v * 0.02;

                    steering_damp = min_mpc(1, steering_damp);
                    steering_damp = max_mpc(0.6, steering_damp);
                    steering_damp = 1;

                    // cout << " *****************************  " << steering_damp << "*****************************  "  << endl;

                }

                if (cte > max_cte)
                {
                    max_cte = cte;
                }
                if (cte < min_cte)
                {
                    min_cte = cte;
                }
                // std::cout << "ins_delay: " << ins_delay << "cte: " << cte << " , max cte: " << max_cte << " , min cte: " << min_cte << std::endl;

                /* Before reference system change:
                doubel epsi = psi - atan(coeffs[1] + 2 * px * coeffs[2] + 3 * coeffs[3* * pow(px,2) where px =0
                在车辆坐标系下，px为车辆当前时刻的纵向向坐标，经过坐标变换后，值为0，而且在新的坐标系下，psi也为0, 当前状态点的航向偏差为实际航向减去期望航向，期望航向为期望轨迹的导数在该点的反正切值*/
                double epsi = -atan(coeffs[1]);

                /*Latency for predicting time at actuation*/
                double dt = controller_delay_compensation;
                // double dt = 0;

                double Lf = kinamatic_para_Lf;

                /* Predict future state (take latency into account) x, y and psi are all zero in the new reference system */
                double pred_px = 0.0 + v_longitudinal * dt; // psi is 0, cos(0) = 1, can leave out
                double pred_py = 0.0 + v_lateral * 0;      // sin(0) = 0
                double pred_psi = 0.0 - v_longitudinal * delta / Lf * 0;
                double pred_v_longitudinal = v_longitudinal + a_longitudinal * 0;
                double pred_v_lateral = v_lateral + 0.0 * 0;
                double pred_omega = yaw_rate;
                double pred_cte = cte + v_longitudinal * tan(epsi) * 0;
                double pred_epsi = epsi - v_longitudinal * delta / Lf * 0;
                // cout << pred_v_longitudinal <<" *********** v_longitudinal" << target_v <<"target_v" << endl;
                /* Feed in the predicted state values.  这里传入的是车辆坐标系下的控制器时延模型*/
                Eigen::VectorXd state(8);
                state << pred_px, pred_py, pred_psi, pred_v_longitudinal, pred_v_lateral, pred_omega, pred_cte, pred_epsi;
                if (target_v <= 0.02) // TODO：超过这个速度极限，MPC就不工作了，直接停车
                {
                    target_v = 0.02;
                }
                // cout << "here" << endl;
                auto vars = mpc.Solve(state,
                                    coeffs,
                                    target_v, // m/s
                                    cte_weight,
                                    epsi_weight,
                                    v_weight,
                                    steer_actuator_cost_weight,
                                    acc_actuator_cost_weight,
                                    change_steer_cost_weight,
                                    change_accel_cost_weight,
                                    mpc_control_horizon_length,
                                    mpc_control_step_length,
                                    kinamatic_para_Lf,
                                    a_lateral,
                                    old_steer_value,
                                    old_throttle_value,
                                    steering_ratio);
                old_steer_value = vars[0];
                old_throttle_value = vars[1];
                // cout << "mpc solve done ~~~~" << endl;
                /* ----------------------------------------------------------------------------- 横向控制信号 ----------------------------------------------------------------------------- */
                if (vars[0] < 0)
                {
                    steer_value = steering_damp * rad2deg((vars[0] / 1)); 
                }
                else
                {
                    steer_value = steering_damp * rad2deg((vars[0] / 1)); 
                }
                vehicle_control_gas_brake_steer_msg.adu_str_whl_ang_req = -steer_value;

                /* ----------------------------------------------------------------------------- 纵向控制信号 ----------------------------------------------------------------------------- */
                // determining if brake or throttle
                throttle_value = vars[1];
                if (throttle_value <= -0.02) // 刹车
                {
                    vehicle_control_gas_brake_steer_msg.adu_gas_stoke_req = 0;
                    vehicle_control_gas_brake_steer_msg.adu_brk_stoke_req = max_mpc(0, min_mpc(100, -9.03 * throttle_value - 9.548));
                }
                else if (throttle_value > -0.02 && throttle_value <=0 ) // 滑行
                {
                    vehicle_control_gas_brake_steer_msg.adu_gas_stoke_req = 0;
                    vehicle_control_gas_brake_steer_msg.adu_brk_stoke_req = 0;
                }
                else if (throttle_value > 0) // 加油
                {
                    vehicle_control_gas_brake_steer_msg.adu_gas_stoke_req = max_mpc(0, min_mpc(100, -0.00208291341652652 * v_longitudinal * v_longitudinal - 0.17134578234501768 * v_longitudinal * throttle_value + 1.035345901656135 * throttle_value * throttle_value + 0.7720091370971741 * v_longitudinal + 26.614571054897834 * throttle_value - 1.2802321021287273));
                    vehicle_control_gas_brake_steer_msg.adu_brk_stoke_req = 0;                    
                }
                
                // if (target_v <= 0.02) // TODO：超过这个速度极限，MPC就不工作了，直接停车
                // {
                //     vehicle_control_gas_brake_steer_msg.adu_gas_stoke_req = 0;
                //     vehicle_control_gas_brake_steer_msg.adu_brk_stoke_req = 20;
                // }
                cout << "Steer:" << std::setprecision(4) << vehicle_control_gas_brake_steer_msg.adu_str_whl_ang_req << " Throttle:" << throttle_value << " Accelerate:" << vehicle_control_gas_brake_steer_msg.adu_gas_stoke_req << " Brake:" << vehicle_control_gas_brake_steer_msg.adu_brk_stoke_req << endl;

                /* 这里的路径和全局路径可视化是重合的，消息发出来，但是不一定要用的 */
                /* Display the waypoints reference line. */
                vector<double> next_x_vals;
                vector<double> next_y_vals;

                next_x_vals.clear();
                next_y_vals.clear();

                /* 可视化里面,这里覆盖了从yaml配置文件传过来的可视化长度 */
                double poly_inc = point_distance_of_reference_line_visualization; // step on x
                size_t num_points = points_number_of_reference_line_visualization; /* how many point "int the future" to be plotted. */

                num_points = reference_path_points_number;
                poly_inc = 0.6;
                
                for (size_t i = 0; i < num_points; i++)
                {
                    double future_x;
                    if (with_planner_flag == 0)
                    {
                        future_x = poly_inc * i;
                    }
                    else
                    {
                        future_x = planner_path_remap_x[i];
                    }
                    double future_y = polyeval(coeffs, future_x);
                    next_x_vals.push_back(future_x);
                    next_y_vals.push_back(future_y);
                }
                reference_path_id ++ ;
                if (reference_path_id > 10000)
                {
                    reference_path_id = 100;
                }
                reference_path.id = reference_path_id;
                reference_path.header.frame_id = "base_link";
                reference_path.header.stamp = this->get_clock()->now();
                reference_path.type = visualization_msgs::msg::Marker::LINE_STRIP;
                reference_path.action = visualization_msgs::msg::Marker::ADD;
                reference_path.lifetime = rclcpp::Duration(20ms); // 如果不想可视化所有的，就设置为 1ms，想可视化所有的就设置为 0ns，可视化一定时间长度的可以自定义时间
                reference_path.scale.x = 0.04;
                reference_path.scale.y = 0.04;
                reference_path.scale.z = 0.04;
                reference_path.color.g = 1.0;
                reference_path.color.a = 1.0;

                reference_path.points.clear();
                geometry_msgs::msg::Point p;
                // 可视化拟合后反求的结果
                for (uint i = 0; i < next_x_vals.size(); i++)
                {
                    if (i % 2 == 0)
                    {
                        p.x = next_x_vals[i];
                    }
                    else
                    {
                        p.y = next_y_vals[i];
                        reference_path.points.push_back(p);
                    }
                }

                // mpc_output_path;
                mpc_output_path.id = reference_path_id;
                mpc_output_path.header.frame_id = "base_link";
                mpc_output_path.header.stamp = this->get_clock()->now();
                mpc_output_path.type = visualization_msgs::msg::Marker::LINE_STRIP;
                mpc_output_path.action = visualization_msgs::msg::Marker::ADD;
                mpc_output_path.lifetime = rclcpp::Duration(20ms);
                mpc_output_path.scale.x = 0.04;
                mpc_output_path.scale.y = 0.04;
                mpc_output_path.scale.z = 0.04;
                mpc_output_path.color.r = 1.0;
                mpc_output_path.color.a = 1.0;

                mpc_output_path.points.clear();

                geometry_msgs::msg::Point pp;
                // 可视化原始点
                for (uint i = 2; i < vars.size(); i++)
                {
                    if (i % 2 == 0)
                    {
                        pp.x = vars[i];
                    }
                    else
                    {
                        pp.y = vars[i];
                        mpc_output_path.points.push_back(pp);
                    }
                }
            }
        }
        
        mpc_reference_path_publisher->publish(reference_path);
        mpc_output_path_publisher->publish(mpc_output_path);
        if (vehicle_longitudinal_feedback_msg->wvcu_gear_stat == vehicle_control_gear_msg.gear_request){
            mpc_control_signals_gas_brake_steer_publisher->publish(vehicle_control_gas_brake_steer_msg);
            // RCLCPP_INFO(this->get_logger(), "RUNNING");
        }
        else{
            mpc_control_signals_gear_publisher->publish(vehicle_control_gear_msg);
            // RCLCPP_INFO(this->get_logger(), "Applying Gear DD!!!!!!!!!!!!!!!!!!!");

        }
        // mpc_speed_control_publisher->publish(speed_control_msg);
        // mpc_shift_control_publisher->publish(shift_control_msg);
        // mpc_parking_control_publisher->publish(parking_control_msg);
        // mpc_brake_pressure_contrl_publisher->publish(brake_pressure_control_msg);
        end_mpc = this->now();
        iteration_time_length = (end_mpc - start_mpc).nanoseconds();
        mpc_iteration_duration_msg.data = iteration_time_length / 1000000;
        mpc_iteration_time_publisher->publish(mpc_iteration_duration_msg);
        RCLCPP_INFO(this->get_logger(), "mpc iteration time: %f ms", iteration_time_length / 1000000);
    }
}
/*'''**************************************************************************************
- FunctionName: None
- Function    : None
- Inputs      : None
- Outputs     : None
- Comments    : None
**************************************************************************************'''*/
int main(int argc, char **argv)
{
    RCLCPP_INFO(LOGGER, "Initialize node");

    rclcpp::init(argc, argv);

    auto n = std::make_shared<MpcTrajectoryTrackingPublisher>(); 

    rclcpp::spin(n); // Create a default single-threaded executor and spin the specified node.

    rclcpp::shutdown();
    return 0;
}