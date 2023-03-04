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
    this->declare_parameter<int>("lqr_pid_working_mode", working_mode);
    this->declare_parameter<int> ("lqr_pid_with_planner_flag", with_planner_flag);
    this->declare_parameter<double>("goal_tolerance", goalTolerance_);     //读取目标速度
    this->declare_parameter<double>("speed_P", speed_P);                   //读取PID参数
    this->declare_parameter<double>("speed_I", speed_I);
    this->declare_parameter<double>("speed_D", speed_D);
    this->declare_parameter<double>("lqr_controller_u", lqr_controller_u);
    this->declare_parameter<double>("lqr_controller_cost_q_1", lqr_controller_cost_q_1);
    this->declare_parameter<double>("lqr_controller_cost_q_2", lqr_controller_cost_q_2);
    this->declare_parameter<double>("lqr_controller_cost_q_3", lqr_controller_cost_q_3);
    this->declare_parameter<double>("lqr_controller_cost_q_4", lqr_controller_cost_q_4);

    // Get the value of a parameter by the given name, and return true if it was set.
    this->get_parameter<double>("vehicle_ref_v", this->target_v);
    this->get_parameter<double>("vehicle_steering_ratio_double", this->steering_ratio);
    this->get_parameter<int>("lqr_pid_working_mode", this->working_mode);
    this->get_parameter<int>("lqr_pid_with_planner_flag", this->with_planner_flag);
    this->get_parameter<double>("goal_tolerance", goalTolerance_);     //读取目标速度
    this->get_parameter<double>("speed_P", speed_P);                   //读取PID参数
    this->get_parameter<double>("speed_I", speed_I);
    this->get_parameter<double>("speed_D", speed_D);
    this->get_parameter<double>("lqr_controller_u", lqr_controller_u);
    this->get_parameter<double>("lqr_controller_cost_q_1", lqr_controller_cost_q_1);
    this->get_parameter<double>("lqr_controller_cost_q_2", lqr_controller_cost_q_2);
    this->get_parameter<double>("lqr_controller_cost_q_3", lqr_controller_cost_q_3);
    this->get_parameter<double>("lqr_controller_cost_q_4", lqr_controller_cost_q_4);

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
    global_path_subscription = this->create_subscription<nav_msgs::msg::Path>("global_path", qos_, std::bind(&LQRPIDTrajectoryTracking::global_path_callback, this, _1));
    lqr_pid_planner_frenet_path_subscription = this->create_subscription<nav_msgs::msg::Path>("lattice_planner_path_frenet", qos_, std::bind(&LQRPIDTrajectoryTracking::palnner_frenet_path_receive_callback, this, _1));
    lqr_pid_planner_cartesian_path_subscription = this->create_subscription<visualization_msgs::msg::Marker>("lattice_planner_path_cardesian", qos_, std::bind(&LQRPIDTrajectoryTracking::palnner_cartesian_path_receive_callback, this, _1));
    vehicle_longitudinal_status_feedback_subscription = this->create_subscription<chassis_msg::msg::WVCULongitudinalStatus>("wvcu_longitudinal_status", qos_,std::bind(&LQRPIDTrajectoryTracking::vehicle_status_feedback_callback, this, _1));

    // localization_data_subscription = this->create_subscription<nav_msgs::msg::Odometry>("/carla/ego_vehicle/odometry", 10, std::bind(&LQRPIDTrajectoryTracking::localization_data_callback, this, _1));
    localization_data_subscription = this->create_subscription<nav_msgs::msg::Odometry>("ins_d_of_vehicle_pose", 10, std::bind(&LQRPIDTrajectoryTracking::localization_data_callback, this, _1));

    lacalization_data_imu_subscription = this->create_subscription<sensor_msgs::msg::Imu>("/carla/ego_vehicle/imu", 10, std::bind(&LQRPIDTrajectoryTracking::localization_data_imu_callback, this, _1));

    vehicle_control_target_velocity.header.stamp = this->now();
    vehicle_control_target_velocity.velocity = 0.0;

    // 定频调用求解器，时间必须大于lqr_pid单次求解耗时
    lqr_pid_iteration_timer_ = this->create_wall_timer(20ms, std::bind(&LQRPIDTrajectoryTracking::lqr_pid_tracking_iteration_callback, this));

    pid_controller_longitudinal = std::make_unique<zww::control::PIDController>(speed_P, speed_I, speed_D);

    lqr_controller_lateral = std::make_unique<zww::control::LqrController>();
    lqr_controller_lateral->LoadControlConf();
    lqr_controller_lateral->Init(lqr_controller_u, lqr_controller_cost_q_1, lqr_controller_cost_q_2, lqr_controller_cost_q_3, lqr_controller_cost_q_4);

    matrix_ad_control_publisher_ = this->create_publisher<matrix_interfaces::msg::MsgToCan>("matrix_vehicle_control_cmd", 10);

    RCLCPP_INFO(this->get_logger(), "target_v %f", this->target_v);
    RCLCPP_INFO(this->get_logger(), "vehicle_steering_ratio_double %f", this->steering_ratio);
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
        // cout << "planner_path_vplanner_path_vplanner_path_v::: " << msg->poses[i].pose.position.y << endl;
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

    from_planner_reference_trajectory.trajectory_points.clear();

    // Construct the reference_line path profile
    std::vector<double> headings;
    std::vector<double> accumulated_s;
    std::vector<double> kappas;
    std::vector<double> dkappas;
    std::unique_ptr<zww::control::ReferenceLine> reference_line = std::make_unique<zww::control::ReferenceLine>(xy_points);
    reference_line->ComputePathProfile(&headings, &accumulated_s, &kappas, &dkappas);

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
- Comments    : the x direction of msg is longitudinal
**************************************************************************************'''*/
void LQRPIDTrajectoryTracking::localization_data_callback(nav_msgs::msg::Odometry::SharedPtr msg){       
    if (is_global_path_received){        
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
    }
}

/*'''**************************************************************************************
- FunctionName: None
- Function    : None
- Inputs      : None
- Outputs     : None
- Comments    : None
**************************************************************************************'''*/
void LQRPIDTrajectoryTracking::localization_data_imu_callback(sensor_msgs::msg::Imu::SharedPtr msg){
    if (is_global_path_received && working_mode == 1){
        RCLCPP_INFO(LOGGER, "Got IMU data!!!");
        vehicleState_.angular_velocity = msg->angular_velocity.z;                                                                                                // 平面角速度(绕z轴转动的角速度)
        vehicleState_.acceleration = sqrt(msg->linear_acceleration.x * msg->linear_acceleration.x + msg->linear_acceleration.y * msg->linear_acceleration.y);    // 加速度
    }
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

    matrix_ad_control_message.connect = 1;
    matrix_ad_control_message.forward = 1;
    matrix_ad_control_message.back = 0;
    matrix_ad_control_message.buzzer = 0;
    matrix_ad_control_message.clock = 0;
    matrix_ad_control_message.speed = 1; // m/s
    matrix_ad_control_message.acc = 16; // 在2.5s内到达指定转速
    matrix_ad_control_message.dec = 1.5; // 没有速度请求的时候在1.5S内减速到0
    // matrix_ad_control_message.angle = 0;

    // cout << "testing" << endl;

    rclcpp::Time start_lqr_pid;
    rclcpp::Time end_lqr_pid;
    start_lqr_pid = this->now();
    double iteration_time_length;

    ControlCmd cmd;

    double v_err;

    if (is_vehicle_longitudinal_received){
        if (rclcpp::ok()){
        // if (0){ // 失能跟踪功能，测试控制信号是否起效
            if (is_global_path_received && is_ins_data_received && is_planner_frenet_path_received && is_planner_cartesian_path_received){
            // if (is_global_path_received && is_ins_data_received){
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
                else{
                    ins_delay = 0;
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
                double v_err = target_point_.v - vehicleState_.velocity;           // 速度误差
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
                    
                    // 速度                
                    Eigen::VectorXd _planner_path_s(planner_path_s.size());
                    Eigen::VectorXd _planner_path_v(planner_path_v.size());
                    for (uint i = 0; i < planner_path_s.size(); i++){
                        _planner_path_s[i] = planner_path_s[i];
                        _planner_path_v[i] = planner_path_v[i];
                        // cout << "_planner_path_s::::::::::" << _planner_path_s[i] << "  " << _planner_path_v[i] << endl;
                    }
                    // auto coeffs_s_v = polyfit(_planner_path_s, _planner_path_v, 5);
                    // target_v = polyeval(coeffs_s_v, car_s);
                    target_v = _planner_path_v.size()/2;
                    v_err = _planner_path_v[int(_planner_path_v.size()/2)] - vehicleState_.velocity; // 如果直接用当前位置的目标速度，那车无法产生足够的加速度请求
                }

                // 小于容忍距离，车辆速度设置为0
                if (pointDistance(goal_point, vehicleState_.x, vehicleState_.y) < goalTolerance_) {
                    // targetSpeed_ = 0;
                    isReachGoal_ = true;
                }
                if (!isReachGoal_) {
                    lqr_controller_lateral->ComputeControlCommand(this->vehicleState_, this->local_reference_trajectory, cmd);
                }
                double acceleration_cmd = pid_controller_longitudinal->Control(v_err, 0.01);
                // double acceleration_cmd = 0.0;

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
                    carla_control_cmd.steer = cmd.steer_target;  // Carla Interface 
                    matrix_ad_control_message.angle = cmd.steer_target * 55; // Matrix Interface
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
            }
        }
        
        if(working_mode == 1){
            carla_vehicle_control_publisher->publish(carla_control_cmd);
            vehicle_control_target_velocity.velocity = target_v;
            vehicle_control_target_velocity_publisher->publish(vehicle_control_target_velocity);
        } 
        else if(working_mode == 2){
            if (vehicle_longitudinal_feedback_msg->wvcu_gear_stat == vehicle_control_gear_msg.gear_request){
                lqr_pid_control_signals_gas_brake_steer_publisher->publish(vehicle_control_gas_brake_steer_msg);
            }
            else{
                lqr_pid_control_signals_gear_publisher->publish(vehicle_control_gear_msg);
            }
        }
        else if (working_mode == 3){
            matrix_ad_control_publisher_->publish(matrix_ad_control_message);
            cout << "matrix vehicle running~~~" << endl;
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