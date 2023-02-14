/*'''*****************************************************************************************************
# FileName    : 
# FileFunction: 
# Comments    : 
*****************************************************************************************************'''*/
#include "lattice_planner/json.hpp"
#include "lattice_planner/spline.h"
#include "lattice_planner/lattice_planner.h"

/* For converting back and forth between radians and degrees. */

static const rclcpp::Logger LOGGER = rclcpp::get_logger("lattice_planner");

double inline min_planner(double a, double b) { return (a < b) ? a : b; }
double inline max_planner(double a, double b) { return (a > b) ? a : b; }

/*'''**************************************************************************************
- FunctionName: None
- Function    : 构造函数
- Inputs      : None
- Outputs     : None
- Comments    : None
**************************************************************************************'''*/
LatticePlanner::LatticePlanner() : Node("lattice_planner") {
    // ins_data_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>("/carla/ego_vehicle/odometry", 10, std::bind(&LatticePlanner::ins_data_receive_callback, this, _1)); // carla
    ins_data_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>("ins_d_of_vehicle_pose", qos_, std::bind(&LatticePlanner::ins_data_receive_callback, this, _1));

    global_path_subscription_ = this->create_subscription<nav_msgs::msg::Path>("global_path", qos_, std::bind(&LatticePlanner::global_path_callback, this, _1));

    planner_iteration_timer_ = this->create_wall_timer(100ms, std::bind(&LatticePlanner::planner_tracking_iteration_callback, this));

    planner_iteration_time_publisher = this->create_publisher<std_msgs::msg::Float32>("planner_iteration_duration", qos_); // 用于统计planner求解时间的广播器

    lattice_planner_path_cartesian_publisher = this->create_publisher<visualization_msgs::msg::Marker>("lattice_planner_path_cardesian", qos_);
    lattice_planner_path_frenet_publisher = this->create_publisher<nav_msgs::msg::Path>("lattice_planner_path_frenet", qos_);

    sensor_fusion_results_bounding_box_subscription_ = this->create_subscription<visualization_msgs::msg::MarkerArray>("sensor_fusion_results_bounding_box", qos_, std::bind(&LatticePlanner::sensor_fusion_results_bounding_box_callback, this, _1));

    sensor_fusion_results_label_subscription_ = this->create_subscription<visualization_msgs::msg::MarkerArray>("sensor_fusion_results_label", qos_, std::bind(&LatticePlanner::sensor_fusion_results_label_callback, this, _1));

    fsm_behavior_decision_makeing_subscription = this->create_subscription<custom_interfaces::msg::FSMDecisionResults>("fsm_behavior_decision", qos_, std::bind(&LatticePlanner::fsm_behavior_decision_makeing_callback, this, _1));

    // Declare and initialize a parameter, return the effective value.
    this->declare_parameter<int>("planner_former_point_of_current_position", former_point_of_current_position);
    this->declare_parameter<int>("planner_working_mode", working_mode);

    // Get the value of a parameter by the given name, and return true if it was set.
    this->get_parameter<int>("planner_former_point_of_current_position", this->former_point_of_current_position);
    this->get_parameter<int>("planner_working_mode", this->working_mode);

    RCLCPP_INFO(this->get_logger(), "planner_former_point_of_current_position %d", this->former_point_of_current_position);
    RCLCPP_INFO(this->get_logger(), "planner_working_mode %d", this->working_mode);

    this->declare_parameter<float>("c_speed", c_speed_);                        //读取Frenet规划器，初始目标速度
    this->declare_parameter<float>("c_d", c_d_);                                //读取Frenet规划器，初始横向偏差
    this->declare_parameter<float>("c_d_d", c_d_d_);                            //读取Frenet规划器，初始横向速度偏差
    this->declare_parameter<float>("c_d_dd", c_d_dd_);                          //读取Frenet规划器，初始横向加速度偏差
    this->declare_parameter<float>("s0", s0_);                                  //读取Frenet规划器，初始纵向距离

    this->get_parameter<float>("c_speed", c_speed_);                        //读取Frenet规划器，初始目标速度
    this->get_parameter<float>("c_d", c_d_);                                //读取Frenet规划器，初始横向偏差
    this->get_parameter<float>("c_d_d", c_d_d_);                            //读取Frenet规划器，初始横向速度偏差
    this->get_parameter<float>("c_d_dd", c_d_dd_);                          //读取Frenet规划器，初始横向加速度偏差
    this->get_parameter<float>("s0", s0_);
}

/*'''**************************************************************************************
- FunctionName: None
- Function    : None
- Inputs      : None
- Outputs     : None
- Comments    : None
**************************************************************************************'''*/
LatticePlanner::~LatticePlanner() {}
/*'''**************************************************************************************
- FunctionName: None
- Function    : None
- Inputs      : None
- Outputs     : None
- Comments    : 这个回调函数发出去的psi的值应该是弧度单位的
**************************************************************************************'''*/
void LatticePlanner::ins_data_receive_callback(nav_msgs::msg::Odometry::SharedPtr msg) {
    // if (is_global_path_received)
    // {
    //     rclcpp::Time now = this->now();
    //     ins_data_arrive_at_planner_through_callback = now.seconds();

    //     // RCLCPP_INFO(this->get_logger(),"got imu data at: %f", this->now().seconds());

    //     tf2::Quaternion quat_tf;
    //     tf2::convert(msg->pose.pose.orientation, quat_tf);
    //     double roll_current, pitch_current, heading_current;
    //     a_longitudinal = msg->pose.covariance[0];
    //     a_lateral = msg->pose.covariance[4];
    //     px = msg->pose.pose.position.x;
    //     py = msg->pose.pose.position.y; // 惯导的y向前
    //     v_longitudinal = msg->twist.twist.linear.x;
    //     v_lateral = msg->twist.twist.linear.y;
    //     yaw_rate = msg->twist.twist.angular.z;

    //     if (v_longitudinal < 0.4 / 3.6)
    //     {
    //         v_lateral = 0;
    //         yaw_rate = 0;
    //     }

    //     ins_frame_arrive_time = msg->header.stamp;
    //     ins_arrive_at_rs232_buffer = this->ins_frame_arrive_time.seconds();

    //     if (1){
    //         tf2::Matrix3x3(quat_tf).getRPY(roll_current, pitch_current, heading_current);
    //         psi = heading_current;
    //     }
    //     else{
    //         roll_current = msg->pose.covariance[3];
    //         pitch_current = msg->pose.covariance[2];
    //         heading_current = msg->pose.covariance[1];

    //         psi = heading_current;
    //     }
        // is_ins_data_received = true;

    //     vector<double> car_s_d = cartesian_to_frenet(px, py, psi, global_path_x, global_path_y);
    //     car_s = car_s_d[0];
    //     car_d = car_s_d[1];

    //     RCLCPP_INFO(this->get_logger(), "got imu data: car_s %f, car_d: %f", car_s, car_d);
    // }
    if (is_global_path_received){

        // ins_frame_arrive_time = msg->header.stamp;
        // ins_arrive_at_rs232_buffer = this->ins_frame_arrive_time.seconds(); 

        // rclcpp::Time now = this->now();
        // ins_data_arrive_at_lqr_pid_through_callback = now.seconds(); //  + now.nanoseconds()/1000000000

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
- Comments    : None
**************************************************************************************'''*/
void LatticePlanner::global_path_callback(nav_msgs::msg::Path::SharedPtr msg) {
    // RCLCPP_INFO(this->get_logger(), "receiveing global path %lu", msg->poses.size());
    // int path_length = msg->poses.size();

    // global_path_x.clear();
    // global_path_y.clear();
    // global_path_s.clear();

    // for (int i = 0; i < path_length; i++)
    // {
    //     global_path_x.push_back(msg->poses[i].pose.position.x);
    //     global_path_y.push_back(msg->poses[i].pose.position.y);
    //     global_path_s.push_back(msg->poses[i].pose.orientation.w);
    // }

    // is_global_path_received = true;

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

    if (first_receive_global_path){
        first_receive_global_path = false;

        // Construct the reference_line path profile
        std::vector<double> headings;
        std::vector<double> accumulated_s;
        std::vector<double> kappas;
        std::vector<double> dkappas;
        std::unique_ptr<zww::control::ReferenceLine> reference_line = std::make_unique<zww::control::ReferenceLine>(xy_points);
        reference_line->ComputePathProfile(&headings, &accumulated_s, &kappas, &dkappas);

        for (size_t i = 0; i < headings.size(); i++) {
            std::cout << "pt " << i << " heading: " << headings[i] << " acc_s: " << accumulated_s[i] << " kappa: " << kappas[i] << " dkappas: " << dkappas[i] << std::endl;
        }

        size_t _count_points = headings.size();
        size_t _stop_begin_point = ceil(_count_points * 0.85);
        size_t _stop_point = ceil(_count_points * 0.95);
        std::cout << "slow down points:" << _stop_begin_point << "  " << _stop_point << std::endl;

        int _index_before_stop = 0;
        for (size_t i = 0; i < headings.size(); i++) {
            cout << i << endl;
            TrajectoryPoint trajectory_pt;
            trajectory_pt.x = xy_points[i].first;
            trajectory_pt.y = xy_points[i].second;

            wx_.push_back(xy_points[i].first);
            wy_.push_back(xy_points[i].second);



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
        cout << "0000000000000" << endl;
        // 构建相对平滑的Frenet曲线坐标系，一个中间暂时方案
        csp_obj_ = new Spline2D(wx_, wy_);

        cout << "11111111111111111" << endl;


        // 构造全局路径变量
        GenerateGlobalPath();

        cout << "222222222222222222222" << endl;



        //  Update Obstacle 添加虚拟障碍物
        UpdateStaticObstacle();

        cout << "33333333333333333333" << endl;

    }
}


/*'''**************************************************************************************
- FunctionName: None
- Function    : None
- Inputs      : None
- Outputs     : None
- Comments    : None
**************************************************************************************'''*/
void LatticePlanner::sensor_fusion_results_bounding_box_callback(visualization_msgs::msg::MarkerArray::SharedPtr msg) {
    sensor_fusion_results_bounding_box = *msg;
    is_sensor_fusion_results_bounding_box_reveived = true;
    RCLCPP_INFO(this->get_logger(), "receiveing sensor fusion bounding box %u", msg->markers[0].header.stamp.nanosec);
}

/*'''**************************************************************************************
- FunctionName: None
- Function    : None
- Inputs      : None
- Outputs     : None
- Comments    : None
**************************************************************************************'''*/
void LatticePlanner::sensor_fusion_results_label_callback(visualization_msgs::msg::MarkerArray::SharedPtr msg) {
    sensor_fusion_results_label = *msg;
    is_sensor_fusion_results_label_received = true;
    RCLCPP_INFO(this->get_logger(), "receiveing sensor fusion label %u", msg->markers[0].header.stamp.nanosec);
}

/*'''**************************************************************************************
- FunctionName: None
- Function    : None
- Inputs      : None
- Outputs     : None
- Comments    : None
**************************************************************************************'''*/
void LatticePlanner::fsm_behavior_decision_makeing_callback(custom_interfaces::msg::FSMDecisionResults::SharedPtr msg){
    RCLCPP_INFO(this->get_logger(), "receiveing behavior decision results: %d", msg->target_behavior);
    this->current_velocity_behavior = msg->target_behavior;
    this->target_lane = msg->target_lane;
}

/*'''**************************************************************************************
- FunctionName: None
- Function    : None
- Inputs      : None
- Outputs     : None
- Comments    : None
**************************************************************************************'''*/
void LatticePlanner::planner_tracking_iteration_callback() {
    rclcpp::Time start_planner;
    rclcpp::Time end_planner;
    start_planner = this->now();
    double iteration_time_length;
    if (rclcpp::ok()) {
        if (is_global_path_received && is_ins_data_received && is_sensor_fusion_results_bounding_box_reveived && is_sensor_fusion_results_label_received) {
            // rclcpp::Time now = this->now();
            // double ins_parse_now = now.seconds();
            double _max_safe_speed = max_safe_speed;
            int current_index_in_last_iteration_path = 0;
            int ref_index_in_front_of_host_vehicle;

            double host_car_s = 0;
            double resuse_length = 0;
            
            // 定位延迟补偿发生在将全局路径转换到车辆坐标系下之前,用来补偿定位信息到达早于被使用而引起的定位误差
            // if (working_mode == 1) {
                // ins_delay = ins_parse_now - ins_data_arrive_at_planner_through_callback + 0.005;
            // }
            // else if (working_mode == 2) {
                // ins_delay = ins_parse_now - ins_arrive_at_rs232_buffer + 0.005;
            // }
            /* 全局坐标系下的定位信息延时补偿 */
            // psi = psi + yaw_rate * ins_delay;
            // px = px + v_longitudinal * cos(psi) * ins_delay - v_lateral * sin(psi) * ins_delay;
            // py = py + v_longitudinal * sin(psi) * ins_delay + v_lateral * cos(psi) * ins_delay;

            // vector<double> car_s_d = cartesian_to_frenet(px, py, psi, global_path_x, global_path_y);
            // car_s = car_s_d[0];
            // car_d = car_s_d[1];
            // host_car_s = car_s_d[0];

            // TODO:这里之后可以再被优化，采用更好的Frenet坐标系取点方式。
            const double ego_s = GetNearestReferenceLength(vehicleState_);
            const double ego_l = GetNearestReferenceLatDist(vehicleState_);
            const double ego_speed = vehicleState_.velocity;

            // cout << "Current Position in Frenet Coordinate, s: " << car_s << ", d: " << car_d << endl;

            // double _safety_margin = safety_margin + _max_safe_speed * 2.0;

            // next_x_vals.clear();
            // next_y_vals.clear();
            // next_s_vals.clear();
            // next_v_vals.clear();
            // next_x_vals_previous_remap.clear();
            // next_y_vals_previous_remap.clear();

            // double velocity_of_front_vehicle_in_same_lane = 0;

            // host_lane = which_lane(car_d);

            // if (fabs((double)sensor_fusion_results_bounding_box.markers[0].header.stamp.nanosec - (double)sensor_fusion_results_label.markers[0].header.stamp.nanosec) < 1000000) {
            //     sensor_fusion.clear();
            //     bounding_box_label_same_frame_check_flag = true;
            //     for (uint i = 0; i < sensor_fusion_results_bounding_box.markers.size(); i++) {
            //         Object_Around sensor_fusion_single = Object_Around(sensor_fusion_results_bounding_box.markers[i],
            //                                                            sensor_fusion_results_label.markers[i],
            //                                                            global_path_s,
            //                                                            global_path_x,
            //                                                            global_path_y);
            //         sensor_fusion.push_back(sensor_fusion_single);
            //         // cout << "d:" << sensor_fusion_single.d << ",s:" << sensor_fusion_single.s << ",px:" << sensor_fusion_single.px << ",py:" << sensor_fusion_single.py << ",v_X:" << sensor_fusion_single.v_X << ",v_Y:" << sensor_fusion_single.v_Y << ",v_lon:" << sensor_fusion_single.v_longitudinal << ",v_lat:" << sensor_fusion_single.v_lateral << ",yaw:" << sensor_fusion_single.yaw << ",yaw_rate:" << sensor_fusion_single.yaw_rate << ",L:" << sensor_fusion_single.length << ",W:" << sensor_fusion_single.width << ",H:" << sensor_fusion_single.height << ",Label:" << sensor_fusion_single.label << "." << endl;
            //     }
            // }
            // if (bounding_box_label_same_frame_check_flag) {
            //     Object_Around sensor_fusion_single = sensor_fusion[0];
            //     for (uint i = 0; i < sensor_fusion.size(); i++) {
            //         sensor_fusion_single = sensor_fusion[i];
            //         // TODO:Collision Check 
            //         if (is_same_lane(sensor_fusion_single.d, car_d)){
            //             velocity_of_front_vehicle_in_same_lane = sensor_fusion_single.v_longitudinal;
            //         }
            //     }
            // }

            
            // // 向左变道
            // if (current_velocity_behavior == 1) {
            //     // lane = host_lane - 1;
            //     lane = target_lane;
            // }
            
            // // 向右变道
            // if (current_velocity_behavior == 2) {
            //     // lane = host_lane + 1;
            //     lane = target_lane;
            // }
            // // 返回目标车道
            // if (current_velocity_behavior == 6) {
            //     if( ((which_lane(car_d) == -1) || (which_lane(car_d) == 1)) && (fabs(psi - ref_yaw) < 10/57.6)){
            //         lane = 0;
            //     }
            //     if ((which_lane(car_d) == -2)  && (fabs(psi - ref_yaw) < 10.0/57.6)){
            //         lane = -1;
            //     }
            //     if ((which_lane(car_d) == 2)  && (fabs(psi - ref_yaw) < 10.0/57.6)){
            //         lane = 1;
            //     }

            // }
            // // 本车道内跟车行驶，参考速度为前车速度
            // if (current_velocity_behavior == 4) {
            //     if(ref_vel >= velocity_of_front_vehicle_in_same_lane){
            //         ref_vel -= 0.3; // 减速策略，匹配该函数的调用频率，可以算得到减速度大小,这个值要和期望速度成倍数，否则停不下来的
            //     }
            // }
            
            // // 起步
            // if (current_velocity_behavior == 7) {
            //     ref_vel += 0.1; // TODO:加速策略，起步策略公用，这里可以更加复杂
            // }

            // if (current_velocity_behavior == 9) {
            //     if (ref_vel > _max_safe_speed){
            //         ref_vel -= 0.2; // TODO:减速策略，起步策略公用，这里可以更加复杂
            //     }
            //     else{
            //         ref_vel += 0.1;
            //     }
            // }

            // // 停车
            // if (current_velocity_behavior == 8) {
            //     ref_vel = 0.00;
            // }

            // if (ref_vel < 0) {
            //     ref_vel = 0;
            // }
            // if (ref_vel > _max_safe_speed) {
            //     ref_vel = _max_safe_speed;
            // }

            // cout << "target_lane results::::::::::::: " << lane << endl;
            
            // //  * TODO: define a path made up of (x,y) points that the car will visit sequentially
            // vector<double> ptsx;
            // vector<double> ptsy;

            // ref_x = px;
            // ref_y = py;
            // ref_yaw = psi;

            // int prev_size = next_s_vals_previous.size();
            

            // // if previous size is almost empty, use the car as starting reference
            // // use two points that make the path tangent to the car
            // if (prev_size < 2) {
            //     double prev_car_x = px - cos(psi);
            //     double prev_car_y = py - sin(psi);

            //     ptsx.push_back(prev_car_x);
            //     ptsx.push_back(px);

            //     ptsy.push_back(prev_car_y);
            //     ptsy.push_back(py);
            // }
            // 新生成路径的时候，复用上一次迭代生成的路径，复用的点是上次生成的路径位于主车前方的两个点
            // else {
                
            //     for (int j = 1; j < next_x_vals_previous.size(); j++){
            //         resuse_length += distance_two_point(next_x_vals_previous[j], next_y_vals_previous[j], next_x_vals_previous[j - 1], next_y_vals_previous[j - 1]);
            //     }

            //     // redefine reference state
                
            //     for (size_t i = 0; i < next_x_vals_previous.size(); i++) {
            //         double shift_x = next_x_vals_previous[i] - px;
            //         double shift_y = next_y_vals_previous[i] - py;
            //         next_x_vals_previous_remap.push_back(shift_x * cos(psi) + shift_y * sin(psi));
            //         next_y_vals_previous_remap.push_back(-shift_x * sin(psi) + shift_y * cos(psi));
            //     }
            //     // 从局部路径中，找到距离当前位置最近的前方的点。
            //     for (size_t i = 0; i < next_x_vals_previous_remap.size(); i++) {
            //         if (next_x_vals_previous_remap[i] >= 0.0){
            //             current_index_in_last_iteration_path = i;
            //             break;
            //         }
            //     }

            //     ref_index_in_front_of_host_vehicle = min_planner(prev_size, current_index_in_last_iteration_path + 36);
            //     if (prev_size > 0){
            //         car_s = next_s_vals_previous[ref_index_in_front_of_host_vehicle]; // car s represent the end point in the last path planning module iteration // car_s represent the future of our host car
            //     }

            //     ref_x = next_x_vals_previous[ref_index_in_front_of_host_vehicle - 1];
            //     ref_y = next_y_vals_previous[ref_index_in_front_of_host_vehicle - 1];
            //     double ref_x_prev = next_x_vals_previous[ref_index_in_front_of_host_vehicle - 2];
            //     double ref_y_prev = next_y_vals_previous[ref_index_in_front_of_host_vehicle - 2];

            //     // ref_yaw = atan2(ref_y_future - ref_y, ref_x_future - ref_x);
            //     ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

            //     ptsx.push_back(ref_x_prev);
            //     ptsx.push_back(ref_x);

            //     ptsy.push_back(ref_y_prev);
            //     ptsy.push_back(ref_y);

            // }
            // vector<double> next_wp0;
            // vector<double> next_wp1;
            // vector<double> next_wp2;
            // vector<double> next_wp3;
            // vector<double> next_wp4;
            // vector<double> next_wp5;
            // // 修改这里可以改变变道的激进程度
            // next_wp1 = frenet_to_cartesian(car_s + max_planner(5, (1 + v_longitudinal * 2.4)), (lane * lane_width), global_path_s, global_path_x, global_path_y); 
            // next_wp3 = frenet_to_cartesian(car_s + max_planner(6, (2 + v_longitudinal * 2.4)), (lane * lane_width), global_path_s, global_path_x, global_path_y);
            // next_wp5 = frenet_to_cartesian(car_s + max_planner(7, (3 + v_longitudinal * 2.4)), (lane * lane_width), global_path_s, global_path_x, global_path_y);

            // ptsx.push_back(next_wp1[0]);
            // ptsx.push_back(next_wp3[0]);
            // ptsx.push_back(next_wp5[0]);

            // ptsy.push_back(next_wp1[1]);
            // ptsy.push_back(next_wp3[1]);
            // ptsy.push_back(next_wp5[1]);

            // 从世界坐标系变换到车辆坐标系(不严格是车辆坐标系，坐标原点修正到了历史轨迹同轴线上)
            // for (uint i = 0; i < ptsx.size(); i++)
            // {
            //     // shift car reference angle to 0 degrees
            //     double shift_x = ptsx[i] - ref_x;
            //     double shift_y = ptsy[i] - ref_y;

            //     ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
            //     ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
            // }

            // // create a spline
            // tk::spline s;
            // // set (x,y) points to the spline
            // s.set_points(ptsx, ptsy);

            // for (int i = 0; i < ptsx.size(); i++){
            //     cout << "ptsx:::" << ptsx[i] << endl;
            //     cout << "ptsy:::" << ptsy[i] << endl;
            // }

            // // start with all of the previous path points from last time
            // if (prev_size > 2){
            //     for (int i = current_index_in_last_iteration_path; i < (ref_index_in_front_of_host_vehicle); i++){
            //         next_x_vals.push_back(next_x_vals_previous[i]);
            //         next_y_vals.push_back(next_y_vals_previous[i]);
            //         next_s_vals.push_back(next_s_vals_previous[i]);
            //         next_v_vals.push_back(ref_vel);
            //     }
            // }
            
            // 每次生成的路径的长度，这个target_dist的值一定不能小于MPC里面的reference_path_length的长度，否则跟踪控制器会震荡
            // 如果要增加重复使用的历史轨迹长度，这里需要把20加大，假如复用40个点，那第一次的规划结果里应该有不少于40个点，这样才行，考虑将复用的路径长度做成和速度成比例的。TODO:
            // double target_x = max_planner(20, v_longitudinal * 3 + 10); // 相当于每次规划都是只规划到前方20m的位置 // TODO:这个值跟着速度变
            // double target_y = s(target_x);
            // double target_dist = sqrt(pow(target_x, 2) + pow(target_y, 2));

            // double x_add_on = 0;
            // // file up the rest of our path planner after filling it with previous points, here we will always output 50 points
            // double x_point = 0;
            // double x_point_last;
            // double y_point_last;
            // double point_distance_x = 0.5;
            // double N = target_dist / point_distance_x;
            // for (int i = 0; i < N; i++) {
            //     x_point = x_add_on + point_distance_x;
            //     double y_point = s(x_point);
            //     x_add_on = x_point;

            //     double x_ref = x_point;
            //     double y_ref = y_point;

            //     // 前面的计算是在车辆坐标系下的（或者是在上一次生成路径点最后的两个点形成的坐标系下），这里转回到世界坐标系下
            //     x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
            //     y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

            //     x_point += ref_x;
            //     y_point += ref_y;

            //     next_x_vals.push_back(x_point);
            //     next_y_vals.push_back(y_point);

            //     if (i == 0){
            //         vector<double> s_d = cartesian_to_frenet(x_point, y_point, ref_yaw, global_path_x, global_path_y);
            //         double _s = s_d[0];
            //         // double d = s_d[1];
            //         next_s_vals.push_back(_s);
            //         next_v_vals.push_back(ref_vel);
            //     }
            //     else{
            //         double future_heading = atan2(y_point - y_point_last, x_point - x_point_last);
            //         vector<double> s_d = cartesian_to_frenet(x_point, y_point, future_heading, global_path_x, global_path_y);
            //         double _s = s_d[0];
            //         next_s_vals.push_back(_s);
            //         next_v_vals.push_back(ref_vel);
            //     }
            //     x_point_last = x_point;
            //     y_point_last = y_point;
                
                
            // }

            s0_ = ego_s;
            if (std::abs(ego_speed) > 1e-3) {
                c_speed_ = ego_speed;
            }
            c_d_ = ego_l;
            std::cout << "vehicleState_.x: " << vehicleState_.x << ", vehicleState_.y: " << vehicleState_.y << ", c_d_: " << c_d_ << ", ego_s: " << ego_s << std::endl;
            // Idea:
            // 判断是否是终点,这里之后需要优化一下，加一个精准停车功能，然后缩小误差范围，发送Stop命令
            if (std::abs(s0_ - end_s_) < 2.0) {
                // break;
                isReachGoal_ = true;
                RCLCPP_ERROR(LOGGER, "Goal Reached!");
            }

            FrenetOptimalTrajectory frenet_optimal_trajectory;
            // to-do step 1 finish frenet_optimal_planning
            FrenetPath final_path = frenet_optimal_trajectory.frenet_optimal_planning(*csp_obj_, s0_, c_speed_, c_d_, c_d_d_, c_d_dd_, obstcle_list_);
            // std::cout << "final_path.s.size(): " << final_path.s.size() << std::endl;
            // std::cout << "final_path.s.empty(): " << final_path.s.empty() << std::endl;
            // std::cout << "near_goal_: " << near_goal_ << std::endl;
            if (!final_path.s.empty() && !near_goal_) {
                s0_ = final_path.s[1];
                c_d_ = final_path.d[1];
                c_d_d_ = final_path.d_d[1];
                c_d_dd_ = final_path.d_dd[1];
                c_speed_ = final_path.s_d[1];

                // // 可视化重规划轨迹
                // // visualization_->publishLocalPlan(final_path);
                // nav_msgs::msg::Path local_path;
                // local_path.header.frame_id = "map";
                // local_path.header.stamp = this->get_clock()->now();
                // const int size = final_path.t.size();
                // for (int i = 0; i < size; i++) {
                //     geometry_msgs::msg::PoseStamped pose;
                //     pose.header.frame_id = "map";
                //     pose.header.stamp = rclcpp::Time();
                //     pose.pose.position.x = final_path.x[i];
                //     pose.pose.position.y = final_path.y[i];
                //     pose.pose.position.z = 0.0;
                //     pose.pose.orientation.x = 0.0;
                //     pose.pose.orientation.y = 0.0;
                //     pose.pose.orientation.z = 0.0;
                //     pose.pose.orientation.w = 0.0;
                //     local_path.poses.push_back(pose);
                // }
                // replan_path_publisher_->publish(local_path);

                const auto trajectory = GetTrajectoryFromFrenetPath(final_path);
                planningPublishedTrajectoryDebug_ = trajectory;
                last_trajectory_ = trajectory;

                if (std::abs(final_path.s.back() - end_s_) < 2.0) {
                    RCLCPP_INFO(LOGGER, "Near Goal");
                    near_goal_ = true;
                }
                // addLocalTrajectoryMarker(
                //     planningPublishedTrajectoryDebug_.trajectory_points, frame_id_);

                plannerFlag_ = true;



                // TODO:打包消息发出去, 规划器发出的结果是在全局笛卡尔坐标系中的
                lattice_planner_path_id ++;
                lattice_planner_path_cardesian.id = lattice_planner_path_id;
                lattice_planner_path_cardesian.header.stamp = this->get_clock()->now();
                lattice_planner_path_cardesian.header.frame_id = "odom";
                lattice_planner_path_cardesian.type = visualization_msgs::msg::Marker::LINE_STRIP;
                lattice_planner_path_cardesian.action = visualization_msgs::msg::Marker::ADD;
                // lattice_planner_path_cardesian.lifetime = rclcpp::Duration(0s); // 显示所有的
                lattice_planner_path_cardesian.lifetime = rclcpp::Duration(0.1s);
                lattice_planner_path_cardesian.scale.x = 1.6;
                lattice_planner_path_cardesian.scale.y = 1.6;
                lattice_planner_path_cardesian.scale.z = 1.6;
                float rrrr =  60.0 / 255.0;
                float gggg = 179.0 / 255.0;
                float bbbb = 113.0 / 255.0;
                lattice_planner_path_cardesian.color.r = rrrr / 2;
                lattice_planner_path_cardesian.color.g = gggg / 2;
                lattice_planner_path_cardesian.color.b = bbbb / 2;
                lattice_planner_path_cardesian.color.a = 1;

                lattice_planner_path_cardesian.points.clear();

                for (size_t i = 0; i < final_path.t.size(); i++) {
                    lattice_planner_path_cardesian_points.x = final_path.x[i];
                    lattice_planner_path_cardesian_points.y = final_path.y[i];
                    lattice_planner_path_cardesian_points.z = 0;

                    lattice_planner_path_cardesian.points.push_back(lattice_planner_path_cardesian_points);
                }
                
                // TODO:打包消息发出去, 规划器发出的结果是在全局Frenet坐标系中的
                lattice_planner_path_frenet.poses.clear();

                lattice_planner_path_frenet.header.stamp = this->get_clock()->now();
                lattice_planner_path_frenet.header.frame_id = "odom";
                for (size_t i = 0; i < final_path.t.size(); i++) {
                    highway_with_prediction_planner_point_frenet.header.frame_id = "odom";
                    highway_with_prediction_planner_point_frenet.header.stamp = this->get_clock()->now();

                    highway_with_prediction_planner_point_frenet.pose.position.x = final_path.s[i];
                    // highway_with_prediction_planner_point_frenet.pose.position.y = 15;
                    // highway_with_prediction_planner_point_frenet.pose.position.y = trajectory.trajectory_points[i].v;
                    highway_with_prediction_planner_point_frenet.pose.position.y = final_path.s_d[i];

                    cout << "final_path.s_d[i]:::@@@@@@@@@@@@@@@  " << final_path.s[i] << ", " << final_path.s_d[i] << endl;

                    highway_with_prediction_planner_point_frenet.pose.position.z = 0;

                    lattice_planner_path_frenet.poses.push_back(highway_with_prediction_planner_point_frenet);
                }
            } 
            else {
                // Backup
                planningPublishedTrajectoryDebug_ = last_trajectory_;
            }
            
            lattice_planner_path_cartesian_publisher->publish(lattice_planner_path_cardesian);
            lattice_planner_path_frenet_publisher->publish(lattice_planner_path_frenet);

            // next_x_vals_previous = next_x_vals;
            // next_y_vals_previous = next_y_vals;
            // next_s_vals_previous = next_s_vals;
            // next_v_vals_previous = next_v_vals;
            // cout << "next_s_vals_previous   next_s_vals_previous:::::::" << next_s_vals_previous.size() << endl;
        }
    }
    end_planner = this->now();
    iteration_time_length = (end_planner - start_planner).nanoseconds();
    planner_iteration_duration_msg.data = iteration_time_length / 1000000;
    planner_iteration_time_publisher->publish(planner_iteration_duration_msg);
    RCLCPP_INFO(this->get_logger(), "planner iteration time: %f ms", iteration_time_length / 1000000);
}


int LatticePlanner::GetNearestReferenceIndex(const VehicleState &ego_state) 
/*'''**************************************************************************************
- FunctionName: None
- Function    : None
- Inputs      : None
- Outputs     : None
- Comments    : None
**************************************************************************************'''*/
{
    double min_dist = std::numeric_limits<double>::max();
    size_t min_index = 0;

    for (size_t i = 0; i < global_plan_.poses.size(); ++i) {
        const double distance = DistanceXY(ego_state, global_plan_.poses[i].pose.position);
        if (distance < min_dist) {
            min_dist = distance;
            min_index = i;
        }
    }
    return min_index;
}

double LatticePlanner::GetNearestReferenceLength(const VehicleState &ego_state) 
/*'''**************************************************************************************
- FunctionName: None
- Function    : None
- Inputs      : None
- Outputs     : None
- Comments    : None
**************************************************************************************'''*/
{
    return global_plan_.poses[GetNearestReferenceIndex(ego_state)].pose.position.z;    // s存在position.z中
}

double LatticePlanner::GetNearestReferenceLatDist(const VehicleState &ego_state) 
/*'''**************************************************************************************
- FunctionName: None
- Function    : None
- Inputs      : None
- Outputs     : None
- Comments    : None
**************************************************************************************'''*/
{
    double min_dist = std::numeric_limits<double>::max();
    size_t min_index = 0;

    for (size_t i = 0; i < global_plan_.poses.size() - 1; ++i) {
        const double distance = DistanceXY(ego_state, global_plan_.poses[i].pose.position);
        if (distance < min_dist) {
            min_dist = distance;
            min_index = i;
        }
    }
    const int sign = LeftOfLine(ego_state, global_plan_.poses[min_index], global_plan_.poses[min_index + 1]) ? 1 : -1;
    return sign * min_dist;
}
/*'''**************************************************************************************
- FunctionName: None
- Function    : None
- Inputs      : None
- Outputs     : None
- Comments    : None
**************************************************************************************'''*/
bool LatticePlanner::LeftOfLine(const VehicleState &p, const geometry_msgs::msg::PoseStamped &p1, const geometry_msgs::msg::PoseStamped &p2) {
    // const double tmpx = (p1.pose.position.x - p2.pose.position.x) / (p1.pose.position.y - p2.pose.position.y) * (p.y - p2.pose.position.y) + p2.pose.position.x;
    // std::cout << "p1.pose.position.x: " << p1.pose.position.x << ", p2.pose.position.x: " << p2.pose.position.x << ", p1.pose.position.y: " << p1.pose.position.y << ", p2.pose.position.y: " << p2.pose.position.y << ", p.y: " << p.y << ", p2.pose.position.y" << p2.pose.position.y << ", p2.pose.position.x" << p2.pose.position.x << std::endl;
    // std::cout << "tmpx: " << tmpx << "p.x: " << p.x << std::endl;
    
    // if (tmpx > p.x)    //当tmpx>p.x的时候，说明点在线的左边，小于在右边，等于则在线上。
    //     return true;
    // return false;

    const double tmpx = (p1.pose.position.x - p.x) * (p2.pose.position.y-p.y) - (p1.pose.position.y - p.y) * (p2.pose.position.x - p.x);
    if (tmpx > 0.0)
    {
        return true;
    }
    else{
        return false;
    }
}

TrajectoryData LatticePlanner::GetTrajectoryFromFrenetPath(const FrenetPath &path) 
/*'''**************************************************************************************
- FunctionName: None
- Function    : None
- Inputs      : None
- Outputs     : None
- Comments    : None
**************************************************************************************'''*/
{
    TrajectoryData trajectory;
    const int traj_size = path.t.size();
    trajectory.trajectory_points.reserve(traj_size);

    for (int i = 0; i < traj_size; i++) {
        TrajectoryPoint trajectory_pt;
        trajectory_pt.x = path.x[i];
        trajectory_pt.y = path.y[i];
        trajectory_pt.v = path.ds[i]; // wrong 这个实际上是两个点之间的距离列表
        trajectory_pt.a = 0.0;
        trajectory_pt.heading = path.yaw[i];
        trajectory_pt.kappa = path.c[i];
        trajectory.trajectory_points.push_back(trajectory_pt);
        cout << "!!!!!!!!!!!!!!!!@@@@@@@@@@@@@@@@@  " << trajectory_pt.v << endl;
    }
    return trajectory;
}

void LatticePlanner::UpdateStaticObstacle() 
/*'''**************************************************************************************
- FunctionName: None
- Function    : None
- Inputs      : None
- Outputs     : None
- Comments    : None
**************************************************************************************'''*/
{
    std::vector<Poi_f> obstcles{{144.414, 250.23}, {40.1, 212.7}}; //TODO 
    // std::vector<Poi_f> obstcles{};
    // std::vector<Poi_f> obstcles{};
    obstcle_list_ = obstcles;
}


void LatticePlanner::GenerateGlobalPath() 
/*'''**************************************************************************************
- FunctionName: None
- Function    : None
- Inputs      : None
- Outputs     : None
- Comments    : None
**************************************************************************************'''*/
{

    Vec_f r_x;
    Vec_f r_y;
    Vec_f ryaw;
    Vec_f rcurvature;
    Vec_f rs;

    global_plan_.poses.clear();
    global_plan_.header.frame_id = "map";
    global_plan_.header.stamp = this->get_clock()->now();
    // 0.1米的间隔进行踩点
    for (float i = 0; i < csp_obj_->s.back(); i += 0.1) {
        std::array<float, 2> point_ = csp_obj_->calc_postion(i);
        r_x.push_back(point_[0]);
        r_y.push_back(point_[1]);
        ryaw.push_back(csp_obj_->calc_yaw(i));
        rcurvature.push_back(csp_obj_->calc_curvature(i));
        rs.push_back(i);

        geometry_msgs::msg::PoseStamped pt;
        pt.header.stamp = this->get_clock()->now();
        pt.header.frame_id = "map";
        pt.pose.position.x = point_[0];
        pt.pose.position.y = point_[1];
        pt.pose.position.z = i;    //使用position.z存储路径的s
        pt.pose.orientation = this->createQuaternionMsgFromYaw(csp_obj_->calc_yaw(i));
        global_plan_.poses.push_back(pt);
    }

    end_x_ = r_x.back();
    end_y_ = r_y.back();
    end_s_ = rs.back();
    RCLCPP_INFO_STREAM(LOGGER, "s_end= " << end_s_);

    // for (float i = 0; i < csp_obj_->s.back(); i += 0.1) {
    //     std::array<float, 2> point_ = csp_obj_->calc_postion(i);
    //     // r_x.push_back(point_[0]);
    //     // r_y.push_back(point_[1]);
    //     // ryaw.push_back(csp_obj_->calc_yaw(i));
    //     // rcurvature.push_back(csp_obj_->calc_curvature(i));
    //     // rs.push_back(i);
    //     global_path.header.frame_id = "gps";
    //     geometry_msgs::msg::PoseStamped pt;
    //     pt.header.stamp = this->get_clock()->now();
    //     pt.header.frame_id = "gps";
    //     pt.pose.position.x = point_[0];
    //     pt.pose.position.y = point_[1];
    //     pt.pose.position.z = 0;    //使用position.z存储路径的s
    //     pt.pose.orientation = this->createQuaternionMsgFromYaw(csp_obj_->calc_yaw(i));
    //     global_path.poses.push_back(pt);
    // }
}
void LatticePlanner::GetWayPoints() {
    const int refline_size = planning_published_trajectory.trajectory_points.size();

    const auto &trajectory_pt = planning_published_trajectory.trajectory_points;

    double sum_s = 0;
    wx_.push_back(trajectory_pt[0].x);
    wy_.push_back(trajectory_pt[0].y);
    for (int i = 1; i < refline_size; i++) {
        const double dx = trajectory_pt[i].x - trajectory_pt[i - 1].x;
        const double dy = trajectory_pt[i].y - trajectory_pt[i - 1].y;
        const double s = std::sqrt(dx * dx + dy * dy);
        sum_s += s;
        // 每隔10米的距离进行采点
        if (sum_s > 2.0) {
            wx_.push_back(trajectory_pt[i].x);
            wy_.push_back(trajectory_pt[i].y);
            RCLCPP_INFO_STREAM(LOGGER, "waypt, x= " << wx_.back() << ", y= " << wy_.back());
            sum_s = 0;
        }
    }

    RCLCPP_INFO_STREAM(LOGGER, "refline_size= " << refline_size << ", waypoint size= " << wx_.size());
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
    // RCLCPP_INFO(LOGGER, "Initialize node");
    rclcpp::init(argc, argv);
    auto n = std::make_shared<LatticePlanner>(); // n指向一个值初始化的 MpcTrajectoryTrackingPublisher
    rclcpp::spin(n); // Create a default single-threaded executor and spin the specified node.
    rclcpp::shutdown();
    return 0;
}