/*'''*****************************************************************************************************
# FileName    : 
# FileFunction: 产生控制信号发送到 ROS 网络上。
# Comments    : 
*****************************************************************************************************'''*/

#include "lqr_pid_trajectory_tracking/spline.h"
#include "lqr_pid_trajectory_tracking/helpers.h"
#include "lqr_pid_trajectory_tracking/coordinate_transform.h"
#include "lqr_pid_trajectory_tracking/lqr_pid_trajectory_tracking_controller.h"
#include "lqr_pid_trajectory_tracking/lqr_pid_trajectory_tracking.h"

static const rclcpp::Logger LOGGER = rclcpp::get_logger("lqr_pid_trajectory_tracking");

double inline min_calculation(double a, double b) { return (a < b) ? a : b; }
double inline max_calculation(double a, double b) { return (a > b) ? a : b; }

/*'''**************************************************************************************
- FunctionName: None
- Function    : 构造函数
- Inputs      : None
- Outputs     : None
- Comments    : None
**************************************************************************************'''*/
LQRPIDTrajectoryTracking::LQRPIDTrajectoryTracking() : Node("lqr_pid_trajectory_tracking") // 使用初始化列表来初始化字段
{
    // Declare and initialize a parameter, return the effective value.
    this->declare_parameter<double>("vehicle_ref_v", target_v);
    this->declare_parameter<double>("vehicle_steering_ratio_double", steering_ratio);
    this->declare_parameter<double>("vehicle_Lf_double", kinamatic_para_Lf);
    this->declare_parameter<int>("lqr_pid_control_horizon_length_int", lqr_pid_control_horizon_length);
    this->declare_parameter<double>("lqr_pid_control_step_length_double", lqr_pid_control_step_length);
    this->declare_parameter<bool>("lqr_pid_tracking_enable_bool", lqr_pid_tracking_enable_bool);
    this->declare_parameter<int>("lqr_pid_cte_weight_int", lqr_pid_cte_weight_int);
    this->declare_parameter<int>("lqr_pid_epsi_weight_int", epsi_weight);
    this->declare_parameter<int>("lqr_pid_v_weight_int", v_weight);
    this->declare_parameter<int>("lqr_pid_steer_actuator_cost_weight_int", steer_actuator_cost_weight);
    this->declare_parameter<int>("lqr_pid_acc_actuator_cost_weight_int", acc_actuator_cost_weight);
    this->declare_parameter<int>("lqr_pid_change_steer_cost_weight_int", change_steer_cost_weight);
    this->declare_parameter<int>("lqr_pid_change_accel_cost_weight_int", change_accel_cost_weight);
    this->declare_parameter<double>("lqr_pid_reference_path_length", reference_path_length);
    this->declare_parameter<double>("lqr_pid_controller_delay_compensation", controller_delay_compensation);
    this->declare_parameter<int>("lqr_pid_former_point_of_current_position", former_point_of_current_position);
    this->declare_parameter<int>("lqr_pid_working_mode", working_mode);
    this->declare_parameter<int> ("lqr_pid_with_planner_flag", with_planner_flag);

    this->declare_parameter<double>("goal_tolerance", goalTolerance_);     //读取目标速度
    this->declare_parameter<double>("speed_P", speed_P);                   //读取PID参数
    this->declare_parameter<double>("speed_I", speed_I);
    this->declare_parameter<double>("speed_D", speed_D);

    // Get the value of a parameter by the given name, and return true if it was set.
    this->get_parameter<double>("vehicle_ref_v", this->target_v);
    this->get_parameter<double>("vehicle_steering_ratio_double", this->steering_ratio);
    this->get_parameter<double>("vehicle_Lf_double", this->kinamatic_para_Lf);
    this->get_parameter<int>("lqr_pid_control_horizon_length_int", this->lqr_pid_control_horizon_length);
    this->get_parameter<double>("lqr_pid_control_step_length_double", this->lqr_pid_control_step_length);
    this->get_parameter<bool>("lqr_pid_tracking_enable_bool", this->lqr_pid_enable_signal);
    this->get_parameter<int>("lqr_pid_cte_weight_int", this->cte_weight);
    this->get_parameter<int>("lqr_pid_epsi_weight_int", this->epsi_weight);
    this->get_parameter<int>("lqr_pid_v_weight_int", this->v_weight);
    this->get_parameter<int>("lqr_pid_steer_actuator_cost_weight_int", this->steer_actuator_cost_weight);
    this->get_parameter<int>("lqr_pid_acc_actuator_cost_weight_int", this->acc_actuator_cost_weight);
    this->get_parameter<int>("lqr_pid_change_steer_cost_weight_int", this->change_steer_cost_weight);
    this->get_parameter<int>("lqr_pid_change_accel_cost_weight_int", this->change_accel_cost_weight);
    this->get_parameter<double>("lqr_pid_reference_path_length", this->reference_path_length);
    this->get_parameter<double>("lqr_pid_controller_delay_compensation", this->controller_delay_compensation);
    this->get_parameter<int>("lqr_pid_former_point_of_current_position", this->former_point_of_current_position);
    this->get_parameter<int>("lqr_pid_working_mode", this->working_mode);
    this->get_parameter<int>("lqr_pid_with_planner_flag", this->with_planner_flag);

    this->get_parameter<double>("goal_tolerance", goalTolerance_);     //读取目标速度
    this->get_parameter<double>("speed_P", speed_P);                   //读取PID参数
    this->get_parameter<double>("speed_I", speed_I);
    this->get_parameter<double>("speed_D", speed_D);

    // vehicle_control_gas_brake_steer_msg.adu_gear_req = 1;
    vehicle_control_gas_brake_steer_msg.adu_brk_stoke_req = 0;
    vehicle_control_gas_brake_steer_msg.adu_gas_stoke_req = 0;
    vehicle_control_gas_brake_steer_msg.adu_str_whl_ang_req = 0;
    vehicle_control_gear_msg.gear_request = 1;

    lqr_pid_control_signals_gas_brake_steer_publisher = this->create_publisher<chassis_msg::msg::ADUDriveCmd>("vehicle_control_signals_gas_brake_steer", qos_);
    lqr_pid_control_signals_gear_publisher = this->create_publisher<chassis_msg::msg::ADUGearRequest>("vehicle_control_signals_gear_request", qos_);
    carla_vehicle_control_publisher = this->create_publisher<carla_msgs::msg::CarlaEgoVehicleControl>("/carla/ego_vehicle/vehicle_control_cmd", 10);
    carla_control_cmd.header.stamp = this->now();
    carla_control_cmd.gear = 1;
    carla_control_cmd.manual_gear_shift = false;
    carla_control_cmd.reverse = false;
    carla_control_cmd.hand_brake = false;
    
    lqr_pid_reference_path_publisher = this->create_publisher<visualization_msgs::msg::Marker>("controller_reference_path", qos_);
    lqr_pid_output_path_publisher = this->create_publisher<visualization_msgs::msg::Marker>("controller_output_path", qos_);
    lqr_pid_iteration_time_publisher = this->create_publisher<std_msgs::msg::Float32>("controller_iteration_duration", qos_); // 用于统计lqr_pid求解时间的广播器
    vehicle_control_target_velocity_publisher = this->create_publisher<carla_msgs::msg::CarlaVehicleTargetVelocity>("/carla/ego_vehicle/target_velocity", 10);

    // lqr_pid 求解所需要的车辆信息
    ins_data_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>("ins_d_of_vehicle_pose", qos_, std::bind(&LQRPIDTrajectoryTracking::ins_data_receive_callback, this, _1));
    // eps_feedback_subscription = this->create_subscription<chassis_msg::msg::WVCUHorizontalStatus>("wvcu_horizontal_status", qos_, std::bind(&LQRPIDTrajectoryTracking::eps_feedback_callback, this, _1));
    global_path_subscription = this->create_subscription<nav_msgs::msg::Path>("global_path", qos_, std::bind(&LQRPIDTrajectoryTracking::global_path_callback, this, _1));
    lqr_pid_planner_frenet_path_subscription = this->create_subscription<nav_msgs::msg::Path>("lattice_planner_path_frenet", qos_, std::bind(&LQRPIDTrajectoryTracking::palnner_frenet_path_receive_callback, this, _1));
    lqr_pid_planner_cartesian_path_subscription = this->create_subscription<visualization_msgs::msg::Marker>("lattice_planner_path_cardesian", qos_, std::bind(&LQRPIDTrajectoryTracking::palnner_cartesian_path_receive_callback, this, _1));
    vehicle_longitudinal_status_feedback_subscription = this->create_subscription<chassis_msg::msg::WVCULongitudinalStatus>("wvcu_longitudinal_status", qos_,std::bind(&LQRPIDTrajectoryTracking::vehicle_status_feedback_callback, this, _1));
    carla_localization_data_subscription = this->create_subscription<nav_msgs::msg::Odometry>("/carla/ego_vehicle/odometry", 10, std::bind(&LQRPIDTrajectoryTracking::carla_odom_callback, this, _1));
    carla_lacalization_data_imu_subscription = this->create_subscription<sensor_msgs::msg::Imu>("/carla/ego_vehicle/imu", 10, std::bind(&LQRPIDTrajectoryTracking::carla_imu_callback, this, _1));

    vehicle_control_target_velocity.header.stamp = this->now();
    vehicle_control_target_velocity.velocity = 0.0;
    // carla_status_subscription = this->create_subscription<carla_msgs::msg::CarlaEgoVehicleStatus>("/carla/ego_vehicle/vehicle_status", 10, std::bind(&LQRPIDTrajectoryTracking::carla_vehicle_status_callback, this, _1));

    // 定频调用求解器，时间必须大于lqr_pid单次求解耗时
    lqr_pid_iteration_timer_ = this->create_wall_timer(10ms, std::bind(&LQRPIDTrajectoryTracking::lqr_pid_tracking_iteration_callback, this));

    pid_controller_longitudinal = std::make_unique<zww::control::PIDController>(speed_P, speed_I, speed_D);

    lqr_controller_lateral = std::make_unique<zww::control::LqrController>();
    lqr_controller_lateral->LoadControlConf();
    lqr_controller_lateral->Init();

    RCLCPP_INFO(this->get_logger(), "target_v %f", this->target_v);
    RCLCPP_INFO(this->get_logger(), "vehicle_steering_ratio_double %f", this->steering_ratio);
    RCLCPP_INFO(this->get_logger(), "vehicle_Lf_double %f", this->kinamatic_para_Lf);
    RCLCPP_INFO(this->get_logger(), "lqr_pid_control_horizon_length_int %d", this->lqr_pid_control_horizon_length);
    RCLCPP_INFO(this->get_logger(), "lqr_pid_control_step_length_double %f", this->lqr_pid_control_step_length);
    RCLCPP_INFO(this->get_logger(), "lqr_pid_tracking_enable_bool %d", int(this->lqr_pid_enable_signal));
    RCLCPP_INFO(this->get_logger(), "lqr_pid_cte_weight_int %d", this->cte_weight);
    RCLCPP_INFO(this->get_logger(), "lqr_pid_epsi_weight_int %d", this->epsi_weight);
    RCLCPP_INFO(this->get_logger(), "lqr_pid_v_weight_int %d", this->v_weight);
    RCLCPP_INFO(this->get_logger(), "lqr_pid_steer_actuator_cost_weight_int %d", this->steer_actuator_cost_weight);
    RCLCPP_INFO(this->get_logger(), "lqr_pid_acc_actuator_cost_weight_int %d", this->acc_actuator_cost_weight);
    RCLCPP_INFO(this->get_logger(), "lqr_pid_change_steer_cost_weight_int %d", this->change_steer_cost_weight);
    RCLCPP_INFO(this->get_logger(), "lqr_pid_change_accel_cost_weight_int %d", this->change_accel_cost_weight);
    RCLCPP_INFO(this->get_logger(), "lqr_pid_reference_path_length %f", this->reference_path_length);
    RCLCPP_INFO(this->get_logger(), "lqr_pid_controller_delay_compensation %f", this->controller_delay_compensation);
    RCLCPP_INFO(this->get_logger(), "lqr_pid_former_point_of_current_position %d", this->former_point_of_current_position);
    RCLCPP_INFO(this->get_logger(), "lqr_pid_working_mode %d", this->working_mode);
    RCLCPP_INFO(this->get_logger(), "lqr_pid_with_planner_flag %d", this->with_planner_flag);

    RCLCPP_WARN(this->get_logger(), "INIT DONE~~~~~");
}

/*'''**************************************************************************************
- FunctionName: None
- Function    : None
- Inputs      : None
- Outputs     : None
- Comments    : None
**************************************************************************************'''*/
LQRPIDTrajectoryTracking::~LQRPIDTrajectoryTracking() {}
/*'''**************************************************************************************
- FunctionName: None
- Function    : None
- Inputs      : None
- Outputs     : None
- Comments    : 规划器规划结果订阅器，规划器给的结果是全局笛卡尔坐标系下的XY坐标点
**************************************************************************************'''*/    
void LQRPIDTrajectoryTracking::palnner_frenet_path_receive_callback(nav_msgs::msg::Path::SharedPtr msg){
    // RCLCPP_INFO(this->get_logger(), "receiving planner frenet_path %lu", msg->poses.size());
    int path_length = msg->poses.size();
    planner_path_s.clear();
    planner_path_v.clear();
    for (int i = 0; i < path_length; i++){
        planner_path_s.push_back(msg->poses[i].pose.position.x);
        planner_path_v.push_back(msg->poses[i].pose.position.y);
        cout << "planner_path_vplanner_path_vplanner_path_v::: " << msg->poses[i].pose.position.y << endl;
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
void LQRPIDTrajectoryTracking::vehicle_status_feedback_callback(chassis_msg::msg::WVCULongitudinalStatus::SharedPtr msg){
    is_vehicle_longitudinal_received = true;
    vehicle_longitudinal_feedback_msg = msg;
    // RCLCPP_INFO(this->get_logger(), "current gear %d", vehicle_longitudinal_feedback_msg->wvcu_gear_stat);
}

void LQRPIDTrajectoryTracking::palnner_cartesian_path_receive_callback(visualization_msgs::msg::Marker::SharedPtr msg){
    // RCLCPP_INFO(this->get_logger(), "receiving planner cartesian path %lu", msg->points.size());
    int path_length = msg->points.size();
    planner_path_x.clear();
    planner_path_y.clear();
    xy_points.clear();
    for (int i = 0; i < path_length; i++){
        planner_path_x.push_back(msg->points[i].x);
        planner_path_y.push_back(msg->points[i].y);
        xy_points.push_back(std::make_pair(msg->points[i].x, msg->points[i].y));
    }
    is_planner_cartesian_path_received = true;

    // v_points.clear();
    
    from_planner_reference_trajectory.trajectory_points.clear();

    // for (int i = 0; i < path_length; i++){
        // global_path_x.push_back(msg->poses[i].pose.position.x);
        // global_path_y.push_back(msg->poses[i].pose.position.y);

        // v_points.push_back(target_v);

    // }

    // Construct the reference_line path profile
    std::vector<double> headings;
    std::vector<double> accumulated_s;
    std::vector<double> kappas;
    std::vector<double> dkappas;
    std::unique_ptr<zww::control::ReferenceLine> reference_line = std::make_unique<zww::control::ReferenceLine>(xy_points);
    reference_line->ComputePathProfile(&headings, &accumulated_s, &kappas, &dkappas);

    // for (size_t i = 0; i < headings.size(); i++) {
    //     std::cout << "pt " << i << " heading: " << headings[i] << " acc_s: " << accumulated_s[i] << " kappa: " << kappas[i] << " dkappas: " << dkappas[i] << std::endl;
    // }

    // size_t _count_points = headings.size();
    // size_t _stop_begin_point = ceil(_count_points * 0.85);
    // size_t _stop_point = ceil(_count_points * 0.95);
    // std::cout << "slow down points:" << _stop_begin_point << "  " << _stop_point << std::endl;

    int _index_before_stop = 0;
    for (size_t i = 0; i < headings.size(); i++) {
        TrajectoryPoint trajectory_pt;
        trajectory_pt.x = xy_points[i].first;
        trajectory_pt.y = xy_points[i].second;
        // trajectory_pt.v = v_points[i];
        trajectory_pt.a = 0.0;
        trajectory_pt.heading = headings[i];
        trajectory_pt.kappa = kappas[i];

        from_planner_reference_trajectory.trajectory_points.push_back(trajectory_pt);
    }
}
/*'''**************************************************************************************
- FunctionName: None
- Function    : None
- Inputs      : None
- Outputs     : None
- Comments    : None
**************************************************************************************'''*/
void LQRPIDTrajectoryTracking::global_path_callback(nav_msgs::msg::Path::SharedPtr msg){
    // RCLCPP_INFO(this->get_logger(), "receiveing global path %lu", msg->poses.size());
    int path_length = msg->poses.size();
    global_path_x.clear();
    global_path_y.clear();
    v_points.clear();
    xy_points.clear();
    global_reference_trajectory.trajectory_points.clear();
    for (int i = 0; i < path_length; i++){
        global_path_x.push_back(msg->poses[i].pose.position.x);
        global_path_y.push_back(msg->poses[i].pose.position.y);

        v_points.push_back(target_v);
        xy_points.push_back(std::make_pair(msg->poses[i].pose.position.x, msg->poses[i].pose.position.y));

    }
    is_global_path_received = true;

    // Construct the reference_line path profile
    std::vector<double> headings;
    std::vector<double> accumulated_s;
    std::vector<double> kappas;
    std::vector<double> dkappas;
    std::unique_ptr<zww::control::ReferenceLine> reference_line = std::make_unique<zww::control::ReferenceLine>(xy_points);
    reference_line->ComputePathProfile(&headings, &accumulated_s, &kappas, &dkappas);

    // for (size_t i = 0; i < headings.size(); i++) {
    //     std::cout << "pt " << i << " heading: " << headings[i] << " acc_s: " << accumulated_s[i] << " kappa: " << kappas[i] << " dkappas: " << dkappas[i] << std::endl;
    // }

    size_t _count_points = headings.size();
    size_t _stop_begin_point = ceil(_count_points * 0.85);
    size_t _stop_point = ceil(_count_points * 0.95);
    std::cout << "slow down points:" << _stop_begin_point << "  " << _stop_point << std::endl;

    int _index_before_stop = 0;
    for (size_t i = 0; i < headings.size(); i++) {
        TrajectoryPoint trajectory_pt;
        trajectory_pt.x = xy_points[i].first;
        trajectory_pt.y = xy_points[i].second;
        if (i < _stop_begin_point) {
            trajectory_pt.v = v_points[i];
            _index_before_stop++;
        } else {
            if (trajectory_pt.v > 1.0) {
                trajectory_pt.v = v_points[_index_before_stop] * ((double)i / ((double)_stop_begin_point - (double)_stop_point) - (double)_stop_point / ((double)_stop_begin_point - (double)_stop_point));
            } else {
                trajectory_pt.v = 0;
            }
        }
        trajectory_pt.a = 0.0;
        trajectory_pt.heading = headings[i];
        trajectory_pt.kappa = kappas[i];

        global_reference_trajectory.trajectory_points.push_back(trajectory_pt);
    }

    goal_point = global_reference_trajectory.trajectory_points.back();

    trajectory_points_ = global_reference_trajectory.trajectory_points;

}

/*'''**************************************************************************************
- FunctionName: None
- Function    : None
- Inputs      : None
- Outputs     : None
- Comments    : 这个回调函数发出去的psi的值应该是弧度单位的
**************************************************************************************'''*/
void LQRPIDTrajectoryTracking::ins_data_receive_callback(nav_msgs::msg::Odometry::SharedPtr msg){
    if (is_global_path_received && working_mode == 2){

        ins_frame_arrive_time = msg->header.stamp;
        ins_arrive_at_rs232_buffer = this->ins_frame_arrive_time.seconds(); 

        rclcpp::Time now = this->now();
        ins_data_arrive_at_lqr_pid_through_callback = now.seconds(); //  + now.nanoseconds()/1000000000

        // RCLCPP_INFO(this->get_logger(),"got imu data at: %f", this->now().seconds()); // this->now().nanoseconds()/1000000000

        vehicleState_.ax = msg->pose.covariance[0];
        vehicleState_.ay = msg->pose.covariance[4];
        vehicleState_.x = msg->pose.pose.position.x;
        vehicleState_.y = msg->pose.pose.position.y;
        vehicleState_.vx = msg->twist.twist.linear.x;
        vehicleState_.vy = msg->twist.twist.linear.y;
        vehicleState_.velocity = std::sqrt(vehicleState_.vx * vehicleState_.vx + vehicleState_.vy * vehicleState_.vy + vehicleState_.vz * vehicleState_.vz);    // 本车速度
        vehicleState_.angular_velocity = msg->twist.twist.angular.z;
        // if (vehicleState_.vx < 0.4/3.6){
        //     vehicleState_.vy = 0;
        //     vehicleState_.angular_velocity = 0;
        // }

        // RCLCPP_INFO(this->get_logger(), "velocity receiveing from ins: %f", this->v_longitudinal);

        tf2::Quaternion quat_tf;
        tf2::convert(msg->pose.pose.orientation, quat_tf);
        double roll_current, pitch_current, heading_current;
        tf2::Matrix3x3(quat_tf).getRPY(roll_current, pitch_current, heading_current);
        vehicleState_.yaw = heading_current;
        vehicleState_.heading = vehicleState_.yaw;

        is_ins_data_received = true;

        // 获取车辆在 frenet 坐标系的定位信息
        vector<double> car_s_d = cartesian_to_frenet(vehicleState_.x, vehicleState_.y, vehicleState_.yaw, global_path_x, global_path_y);
        car_s = car_s_d[0];
        car_d = car_s_d[1];
    }
}

/*'''**************************************************************************************
- FunctionName: None
- Function    : None
- Inputs      : None
- Outputs     : None
- Comments    : the x direction of msg is longitudinal
**************************************************************************************'''*/
void LQRPIDTrajectoryTracking::carla_odom_callback(nav_msgs::msg::Odometry::SharedPtr msg){       
    if (is_global_path_received && working_mode == 1){        
        is_ins_data_received = true;
        is_vehicle_longitudinal_received = true;
        rclcpp::Time now = this->now();
        ins_data_arrive_at_lqr_pid_through_callback = now.seconds(); //  + now.nanoseconds()/1000000000
        RCLCPP_INFO(LOGGER, "Got ODOM data!!!");
        // 将orientation(四元数)转换为欧拉角(roll, pitch, yaw) 
        tf2::Quaternion quat_tf;
        tf2::convert(msg->pose.pose.orientation, quat_tf);
        tf2::Matrix3x3(quat_tf).getRPY(vehicleState_.roll, vehicleState_.pitch, vehicleState_.yaw);

        if (firstRecord_) {
            vehicleState_.start_point_x = msg->pose.pose.position.x;
            vehicleState_.start_point_y = msg->pose.pose.position.y;
            firstRecord_ = false;
        }
        vehicleState_.x = msg->pose.pose.position.x;
        vehicleState_.y = msg->pose.pose.position.y;
        vehicleState_.vx = msg->twist.twist.linear.x;
        vehicleState_.vy = msg->twist.twist.linear.y;
        vehicleState_.vz = msg->twist.twist.linear.z;
        vehicleState_.velocity = std::sqrt(vehicleState_.vx * vehicleState_.vx + vehicleState_.vy * vehicleState_.vy + vehicleState_.vz * vehicleState_.vz);    // 本车速度
        vehicleState_.heading = vehicleState_.yaw;

        px = msg->pose.pose.position.x;
        py = msg->pose.pose.position.y;
        psi = vehicleState_.yaw;
        v_longitudinal = msg->twist.twist.linear.x;
        v_lateral = msg->twist.twist.linear.y;
        }
}

/*'''**************************************************************************************
- FunctionName: None
- Function    : None
- Inputs      : None
- Outputs     : None
- Comments    : None
**************************************************************************************'''*/
void LQRPIDTrajectoryTracking::carla_imu_callback(sensor_msgs::msg::Imu::SharedPtr msg){
    if (is_global_path_received && working_mode == 1){
        RCLCPP_INFO(LOGGER, "Got IMU data!!!");
        vehicleState_.angular_velocity = msg->angular_velocity.z;                                                                                                // 平面角速度(绕z轴转动的角速度)
        vehicleState_.acceleration = sqrt(msg->linear_acceleration.x * msg->linear_acceleration.x + msg->linear_acceleration.y * msg->linear_acceleration.y);    // 加速度

        a_longitudinal = msg->linear_acceleration.x;
        a_lateral = msg->linear_acceleration.y;
        yaw_rate = msg->angular_velocity.z;
    }
}

// /*'''**************************************************************************************
// - FunctionName: None
// - Function    : None
// - Inputs      : None
// - Outputs     : None
// - Comments    : None
// **************************************************************************************'''*/
// void LQRPIDTrajectoryTracking::eps_feedback_callback(chassis_msg::msg::WVCUHorizontalStatus::SharedPtr msg){
//     delta = deg2rad(msg->wvcu_str_whl_ang_stat) / steering_ratio;
//     // RCLCPP_INFO(this->get_logger(), "receiving eps angel: %f", delta);
//     is_eps_received = true;
// }

// /*'''**************************************************************************************
// - FunctionName: None
// - Function    : None
// - Inputs      : None
// - Outputs     : None
// - Comments    : 为了在rqt里面，一个plot里面查看目标速度和实际速度
// **************************************************************************************'''*/
// void LQRPIDTrajectoryTracking::carla_vehicle_status_callback(carla_msgs::msg::CarlaEgoVehicleStatus::SharedPtr msg){
//     vehicle_control_target_velocity.header.stamp = msg->header.stamp;
//     delta = deg2rad(msg->control.steer * 30 );    // [-1, 1] from carla 30这里当做前轮最大转角
//     is_eps_received = true;
// }

double LQRPIDTrajectoryTracking::PointDistanceSquare(const TrajectoryPoint& point, const double x, const double y)
/*'''**************************************************************************************
- FunctionName: None
- Function    : 两点之间的距离
- Inputs      : None
- Outputs     : None
- Comments    : None
**************************************************************************************'''*/
{
    double dx = point.x - x;
    double dy = point.y - y;
    return dx * dx + dy * dy;
}

TrajectoryPoint LQRPIDTrajectoryTracking::QueryNearestPointByPosition(const double x, const double y)
/*'''**************************************************************************************
- FunctionName: None
- Function    : None
- Inputs      : None
- Outputs     : None
- Comments    : None
**************************************************************************************'''*/
{
    double d_min = this->PointDistanceSquare(trajectory_points_.front(), x, y);
    size_t index_min = 0;

    for (size_t i = 1; i < trajectory_points_.size(); ++i) {
        double d_temp = this->PointDistanceSquare(trajectory_points_[i], x, y);
        if (d_temp < d_min) {
            d_min = d_temp;
            index_min = i;
        }
    }
    return trajectory_points_[index_min];
}

int LQRPIDTrajectoryTracking::QueryNearestPointByPosition(const double x, const double y, std::vector<TrajectoryPoint> current_trajectory )
/*'''**************************************************************************************
- FunctionName: None
- Function    : None
- Inputs      : None
- Outputs     : None
- Comments    : None
**************************************************************************************'''*/
{
    double d_min = this->PointDistanceSquare(current_trajectory.front(), x, y);
    int index_min = 0;

    for (int i = 1; i < current_trajectory.size(); ++i) {
        double d_temp = this->PointDistanceSquare(current_trajectory[i], x, y);
        if (d_temp < d_min) {
            d_min = d_temp;
            index_min = i;
        }
    }
    return index_min;
}

/*'''**************************************************************************************
- FunctionName: None
- Function    : None
- Inputs      : None
- Outputs     : None
- Comments    : None
**************************************************************************************'''*/
void LQRPIDTrajectoryTracking::lqr_pid_tracking_iteration_callback(){
    // 直接发控制信号给底盘，测试底盘是否正常
    // vehicle_control_gas_brake_steer_msg.adu_gear_req = 3;
    // vehicle_control_gas_brake_steer_msg.adu_brk_stoke_req = 0;
    // vehicle_control_gas_brake_steer_msg.adu_gas_stoke_req = 40;
    // vehicle_control_gas_brake_steer_msg.adu_str_whl_ang_req = 100;
    vehicle_control_gear_msg.gear_request = 3;
    carla_control_cmd.steer = 0.0;
    carla_control_cmd.throttle = 0.2;
    // carla_control_cmd.brake = 0;
    // carla_control_cmd.gear = 1;

    rclcpp::Time start_lqr_pid;
    rclcpp::Time end_lqr_pid;
    start_lqr_pid = this->now();
    double iteration_time_length;

    ControlCmd cmd;

    double v_err;

    if (is_vehicle_longitudinal_received){
        if (rclcpp::ok())
        // if (0) // 失能跟踪功能，测试控制信号是否起效
        {
            // this->reference_path_length = max_calculation(floor(this->lqr_pid_control_horizon_length * this->lqr_pid_control_step_length * this->v_longitudinal) + 1, 10.0);
            // this->reference_path_length = 30;
            // RCLCPP_INFO(this->get_logger(), "reference_path_length: %f", this->reference_path_length);

            // if (is_eps_received && is_global_path_received && is_ins_data_received && is_planner_frenet_path_received && is_planner_cartesian_path_received){
            if (is_global_path_received && is_ins_data_received){
                global_path_remap_x.clear();
                global_path_remap_y.clear();

                planner_path_remap_x.clear();
                planner_path_remap_y.clear();

                rclcpp::Time now = this->now();
                double ins_parse_now = now.seconds(); 
                // 定位延迟补偿发生在将全局路径转换到车辆坐标系下之前,用来补偿定位信息到达早于被使用而引起的定位误差
                if (working_mode == 1){
                    ins_delay = ins_parse_now - ins_data_arrive_at_lqr_pid_through_callback + 0.005;
                }
                else if (working_mode == 2){
                    ins_delay = ins_parse_now - ins_arrive_at_rs232_buffer + 0.005;
                }

                /* 全局坐标系下的定位信息延时补偿 */
                vehicleState_.heading = vehicleState_.heading + vehicleState_.angular_velocity * ins_delay;
                vehicleState_.x = vehicleState_.x + vehicleState_.vx * cos(vehicleState_.heading) * ins_delay - vehicleState_.vy * sin(vehicleState_.heading) * ins_delay;
                vehicleState_.y = vehicleState_.y + vehicleState_.vx * sin(vehicleState_.heading) * ins_delay + vehicleState_.vy * cos(vehicleState_.heading) * ins_delay;

                // 运动补偿后的Frenet坐标系中的车辆定位信息
                vector<double> car_s_d = cartesian_to_frenet(vehicleState_.x, vehicleState_.y, vehicleState_.heading, global_path_x, global_path_y);
                car_s = car_s_d[0];
                car_d = car_s_d[1];

                // LQR ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                TrajectoryPoint target_point_;
                target_point_ = this->QueryNearestPointByPosition(vehicleState_.x, vehicleState_.y);
                // double v_err = target_point_.v - vehicleState_.velocity;           // 速度误差
                double yaw_err = vehicleState_.heading - target_point_.heading;    // 横摆角误差
                

                // -------------------- 使用全局路径作为跟踪控制器的参考路径(这里可以进一步压缩时间的，每次都转换整条路径时间代价太大而且没用) --------------------
                if (with_planner_flag == 0){
                    this->local_reference_trajectory = this->global_reference_trajectory;
                    v_err = target_point_.v - vehicleState_.velocity;           // 速度误差
                }
                // -------------------- 使用规划器重规划路径作为跟踪控制器的参考路径 --------------------
                else{
                    this->local_reference_trajectory = this->from_planner_reference_trajectory;
                    int current_index = this->QueryNearestPointByPosition(vehicleState_.x, vehicleState_.y, this->local_reference_trajectory.trajectory_points);
                    // reference_path_points_number = 0;
                    // int planner_path_former_point_of_current_position = 0;
                    // for (size_t i = 0; i < planner_path_x.size() - 1; i++){
                    //     double shift_x = planner_path_x[i] - px;
                    //     double shift_y = planner_path_y[i] - py;
                    //     planner_path_remap_x.push_back(  shift_x * cos(psi) + shift_y * sin(psi));
                    //     planner_path_remap_y.push_back(- shift_x * sin(psi) + shift_y * cos(psi));
                    // }
                    // // 从局部路径中，找到距离当前位置最近的前方的点。
                    // for (size_t i = planner_path_former_point_of_current_position; i < planner_path_remap_x.size(); i++){
                    //     if (planner_path_remap_x[i] > 0.0){
                    //         planner_path_former_point_of_current_position = i;
                    //         break;
                    //     }            
                    // }
                    // double temp_total_distance = 0;
                    // while (temp_total_distance <= this->reference_path_length){
                    //     temp_total_distance += distance_two_point(planner_path_x[reference_path_points_number], 
                    //                                                 planner_path_y[reference_path_points_number], 
                    //                                                 planner_path_x[reference_path_points_number+1], 
                    //                                                 planner_path_y[reference_path_points_number+1]);
                    //     reference_path_points_number ++;
                    // }
                    // cout << "reference_path_points_number: " << reference_path_points_number << endl;
                    // rclcpp::Time here1 = this->now();
                    // double *ptrx = &planner_path_remap_x[planner_path_former_point_of_current_position];
                    // Eigen::Map<Eigen::VectorXd> ptsx_transform(ptrx, reference_path_points_number);
                    // double *ptry = &planner_path_remap_y[planner_path_former_point_of_current_position];
                    // Eigen::Map<Eigen::VectorXd> ptsy_transform(ptry, reference_path_points_number);            
                    // coeffs = polyfit(ptsx_transform, ptsy_transform, 5);
                    // cte    = polyeval(coeffs, 0); // 在车辆坐标系下，当前时刻的位置偏差就是期望轨迹在车辆坐标系中的截距
                    
                    // 速度                
                    Eigen::VectorXd _planner_path_s(planner_path_s.size());
                    Eigen::VectorXd _planner_path_v(planner_path_v.size());
                    for (uint i = 0; i < planner_path_s.size(); i++){
                        _planner_path_s[i] = planner_path_s[i];
                        _planner_path_v[i] = planner_path_v[i];
                        cout << "_planner_path_s::::::::::" << _planner_path_s[i] << "  " << _planner_path_v[i] << endl;
                    }
                    auto coeffs_s_v = polyfit(_planner_path_s, _planner_path_v, 5);
                    target_v = polyeval(coeffs_s_v, car_s);
                    v_err = 12 - vehicleState_.velocity;
                    cout << "target_VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV: " << target_v << ",  current car_s" << car_s << endl;
                    // target_v = 4;
                }

                // 小于容忍距离，车辆速度设置为0
                if (pointDistance(goal_point, vehicleState_.x, vehicleState_.y) < goalTolerance_) {
                    targetSpeed_ = 0;
                    isReachGoal_ = true;
                }
                if (!isReachGoal_) {
                    lqr_controller_lateral->ComputeControlCommand(this->vehicleState_, this->local_reference_trajectory, cmd);
                }
                double acceleration_cmd = pid_controller_longitudinal->Control(v_err, 0.01);
                

                carla_control_cmd.header.stamp = this->now();
                if (acceleration_cmd >= 1.0) {
                    acceleration_cmd = 1.0;
                }
                if (acceleration_cmd <= -1) {
                    acceleration_cmd = -1.0;
                }
                if (acceleration_cmd <= 0) {
                    carla_control_cmd.brake = -acceleration_cmd;
                    // carla_control_cmd.brake = 0;
                    carla_control_cmd.throttle = 0;
                } else {
                    carla_control_cmd.throttle = acceleration_cmd;
                    carla_control_cmd.brake = 0;
                }
                if (isnan(cmd.steer_target)) {
                    carla_control_cmd.steer = 0;
                } else {
                    carla_control_cmd.steer = cmd.steer_target;
                }
                // carla_control_cmd.steer = 0;
                carla_control_cmd.gear = 1;
                carla_control_cmd.reverse = false;
                carla_control_cmd.hand_brake = false;
                carla_control_cmd.manual_gear_shift = false;
                
                cout << "v_err: " << v_err << ", acceleration_cmd: " << acceleration_cmd << endl;
                cout << "steer control cmd: " << cmd.steer_target << endl;
                cout << "carla_control_cmd.steer: " << carla_control_cmd.steer << endl;
                cout << "~~ vehicleState_.v: " << vehicleState_.velocity << ", target_point_.v: " << target_point_.v << ", v_err: " << v_err << endl;
                cout << "yaw_err: " << yaw_err << endl;
                

                // if (cte > max_cte){
                //     max_cte = cte;
                // }
                // if (cte < min_cte){
                //     min_cte = cte;
                // }
                // // std::cout << "ins_delay: " << ins_delay << "cte: " << cte << " , max cte: " << max_cte << " , min cte: " << min_cte << std::endl;

                // /* Before reference system change:
                // doubel epsi = psi - atan(coeffs[1] + 2 * px * coeffs[2] + 3 * coeffs[3* * pow(px,2) where px =0
                // 在车辆坐标系下，px为车辆当前时刻的纵向向坐标，经过坐标变换后，值为0，而且在新的坐标系下，psi也为0, 当前状态点的航向偏差为实际航向减去期望航向，期望航向为期望轨迹的导数在该点的反正切值*/
                // double epsi = -atan(coeffs[1]);

                // /*Latency for predicting time at actuation*/
                // double dt = controller_delay_compensation;
                // double Lf = kinamatic_para_Lf;

                // /* Predict future state (take latency into account) x, y and psi are all zero in the new reference system */
                // double pred_px = 0.0 + v_longitudinal * dt; // psi is 0, cos(0) = 1, can leave out
                // double pred_py = 0.0 + v_lateral * 0;      // sin(0) = 0
                // double pred_psi = 0.0 - v_longitudinal * delta / Lf * 0;
                // double pred_v_longitudinal = v_longitudinal + a_longitudinal * 0;
                // double pred_v_lateral = v_lateral + 0.0 * 0;
                // double pred_omega = yaw_rate;
                // double pred_cte = cte + v_longitudinal * tan(epsi) * 0;
                // double pred_epsi = epsi - v_longitudinal * delta / Lf * 0;
                // /* Feed in the predicted state values.  这里传入的是车辆坐标系下的控制器时延模型*/
                // Eigen::VectorXd state(8);
                // state << pred_px, pred_py, pred_psi, pred_v_longitudinal, pred_v_lateral, pred_omega, pred_cte, pred_epsi;
                // if (target_v <= 0.1) // TODO：超过这个速度极限，lqr_pid就不工作了，直接停车
                // {
                //     target_v = 0;
                // }
                // auto vars = lqr_pid.Solve(state,
                //                     coeffs,
                //                     target_v,
                //                     cte_weight,
                //                     epsi_weight,
                //                     v_weight,
                //                     steer_actuator_cost_weight,
                //                     acc_actuator_cost_weight,
                //                     change_steer_cost_weight,
                //                     change_accel_cost_weight,
                //                     lqr_pid_control_horizon_length,
                //                     lqr_pid_control_step_length,
                //                     kinamatic_para_Lf,
                //                     a_lateral,
                //                     steering_ratio);
                // old_steer_value = vars[0]; 
                // old_throttle_value = vars[1];

                /* ------------------------------------------------------------------------- Carla 控制接口 ------------------------------------------------------------------------- */
                // carla_control_cmd.header.stamp = this->now();

                // if (vars[0] >= 1.0) {
                //     vars[0] = 0.0;
                // }
                // if (vars[0] <= -1) {
                //     vars[0] = -0.0;
                // }
                // if (isnan(old_steer_value)) {
                //     carla_control_cmd.steer = 0;
                // } else {
                //     carla_control_cmd.steer = rad2deg(vars[0]) / 30;
                // }

                // if (vars[1] >= 1.0) {
                //     vars[1] = 1.0;
                // }
                // if (vars[1] <= -1) {
                //     vars[1] = -1.0;
                // }
                // if (vars[1] <= 0) {
                //     carla_control_cmd.brake = -vars[1];
                //     carla_control_cmd.throttle = 0;
                // } else {
                //     carla_control_cmd.throttle = vars[1];
                //     carla_control_cmd.brake = 0;
                // }
            
                // carla_control_cmd.reverse = false;
                // carla_control_cmd.hand_brake = false;
                // carla_control_cmd.manual_gear_shift = false;

                // /* ----------------------------------------------------------------------------- 横向控制信号 ----------------------------------------------------------------------------- */
                // steer_value = 1 * rad2deg((vars[0] / 1)); 
                // vehicle_control_gas_brake_steer_msg.adu_str_whl_ang_req = -steer_value;
                // if (target_v <= 1) // TODO：超过这个速度极限，lqr_pid就不工作了，直接停车
                // {
                //    vehicle_control_gas_brake_steer_msg.adu_str_whl_ang_req = 0;
                // } 

                // /* ----------------------------------------------------------------------------- 纵向控制信号 ----------------------------------------------------------------------------- */
                // // determining if brake or throttle
                // throttle_value = vars[1];
                // if (throttle_value <= -0.01){
                //     vehicle_control_gas_brake_steer_msg.adu_gas_stoke_req = 0;
                //     vehicle_control_gas_brake_steer_msg.adu_brk_stoke_req = 3.8 * max_calculation(0, min_calculation(100, 0.3892 * throttle_value  * throttle_value - throttle_value *  5.072 - 0.7559));
                // }
                // else if (throttle_value > -0.02 && throttle_value <=0 ){
                //     vehicle_control_gas_brake_steer_msg.adu_gas_stoke_req = 0;
                //     vehicle_control_gas_brake_steer_msg.adu_brk_stoke_req = 0;
                // }
                // else if (throttle_value > 0){
                //     vehicle_control_gas_brake_steer_msg.adu_gas_stoke_req = 1.4 * max_calculation(0, min_calculation(100, -0.00208291341652652 * v_longitudinal * v_longitudinal - 0.17134578234501768 * v_longitudinal * throttle_value + 1.035345901656135 * throttle_value * throttle_value + 0.7720091370971741 * v_longitudinal + 26.614571054897834 * throttle_value - 1.2802321021287273));
                //     vehicle_control_gas_brake_steer_msg.adu_brk_stoke_req = 0;                    
                // }
                
                // cout << "Steer:" << std::setprecision(4) << vehicle_control_gas_brake_steer_msg.adu_str_whl_ang_req << " Throttle:" << throttle_value << " Accelerate:" << vehicle_control_gas_brake_steer_msg.adu_gas_stoke_req << " Brake:" << vehicle_control_gas_brake_steer_msg.adu_brk_stoke_req << endl;

                /* 这里的路径和全局路径可视化是重合的，消息发出来，但是不一定要用的 */
            //     next_x_vals.clear();
            //     next_y_vals.clear();
            //     for (int i = 0; i < reference_path_points_number; i++){
            //         double future_x;
            //         if (with_planner_flag == 0){
            //             future_x = 0.6 * i;
            //         }
            //         else{
            //             future_x = planner_path_remap_x[i];
            //         }
            //         double future_y = polyeval(coeffs, future_x);
            //         next_x_vals.push_back(future_x);
            //         next_y_vals.push_back(future_y);
            //     }

            //     // reference_path_id ++ ;
            //     // if (reference_path_id > 10000){
            //     //     reference_path_id = 100;
            //     // }
            //     reference_path.id = reference_path_id;
            //     reference_path.header.frame_id = "base_link";
            //     reference_path.header.stamp = this->get_clock()->now();
            //     reference_path.type = visualization_msgs::msg::Marker::LINE_STRIP;
            //     reference_path.action = visualization_msgs::msg::Marker::ADD;
            //     reference_path.lifetime = rclcpp::Duration(20ms); 
            //     reference_path.scale.x = 0.04;
            //     reference_path.scale.y = 0.04;
            //     reference_path.scale.z = 0.04;
            //     reference_path.color.g = 1.0;
            //     reference_path.color.a = 1.0;

            //     reference_path.points.clear();
            //     geometry_msgs::msg::Point p;
            //     // 可视化拟合后反求的结果
            //     for (uint i = 0; i < next_x_vals.size(); i++){
            //         if (i % 2 == 0){
            //             p.x = next_x_vals[i];
            //         }
            //         else{
            //             p.y = next_y_vals[i];
            //             reference_path.points.push_back(p);
            //         }
            //     }
            //     // lqr_pid_output_path;
            //     lqr_pid_output_path.id = reference_path_id + 1;
            //     lqr_pid_output_path.header.frame_id = "base_link";
            //     lqr_pid_output_path.header.stamp = this->get_clock()->now();
            //     lqr_pid_output_path.type = visualization_msgs::msg::Marker::LINE_STRIP;
            //     lqr_pid_output_path.action = visualization_msgs::msg::Marker::ADD;
            //     lqr_pid_output_path.lifetime = rclcpp::Duration(20ms);
            //     // lqr_pid_output_path.lifetime = rclcpp::Duration(200000ms);
            //     lqr_pid_output_path.scale.x = 0.04;
            //     lqr_pid_output_path.scale.y = 0.04;
            //     lqr_pid_output_path.scale.z = 0.04;
            //     lqr_pid_output_path.color.r = 1.0;
            //     lqr_pid_output_path.color.a = 1.0;

            //     lqr_pid_output_path.points.clear();
            //     geometry_msgs::msg::Point pp;
            //     // 可视化原始点
            //     for (uint i = 2; i < vars.size(); i++){
            //         if (i % 2 == 0){
            //             pp.x = vars[i];
            //         }
            //         else{
            //             pp.y = vars[i];
            //             lqr_pid_output_path.points.push_back(pp);
            //         }
            //     }
            }
            // lqr_pid_reference_path_publisher->publish(reference_path);
            // lqr_pid_output_path_publisher->publish(lqr_pid_output_path);
        }
        
        if(working_mode == 2){
            if (vehicle_longitudinal_feedback_msg->wvcu_gear_stat == vehicle_control_gear_msg.gear_request){
                lqr_pid_control_signals_gas_brake_steer_publisher->publish(vehicle_control_gas_brake_steer_msg);
            }
            else{
                lqr_pid_control_signals_gear_publisher->publish(vehicle_control_gear_msg);
            }
        }
        if(working_mode == 1){
            carla_vehicle_control_publisher->publish(carla_control_cmd);
            vehicle_control_target_velocity.velocity = target_v;
            vehicle_control_target_velocity_publisher->publish(vehicle_control_target_velocity);
        }   
        end_lqr_pid = this->now();
        iteration_time_length = (end_lqr_pid - start_lqr_pid).nanoseconds();
        lqr_pid_iteration_duration_msg.data = iteration_time_length / 1000000;
        lqr_pid_iteration_time_publisher->publish(lqr_pid_iteration_duration_msg);
        cout << "~~~ lqr_pid iteration time: " <<  iteration_time_length / 1000000 << "ms ~~~" << endl;
        
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
    rclcpp::init(argc, argv);
    auto n = std::make_shared<LQRPIDTrajectoryTracking>(); 
    rclcpp::spin(n);
    rclcpp::shutdown();
    return 0;
}