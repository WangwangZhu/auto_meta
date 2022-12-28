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
    ins_data_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>("ins_d_of_vehicle_pose", qos_, std::bind(&LatticePlanner::ins_data_receive_callback, this, _1));

    global_path_subscription_ = this->create_subscription<nav_msgs::msg::Path>("global_path", qos_, std::bind(&LatticePlanner::global_path_callback, this, _1));

    planner_iteration_timer_ = this->create_wall_timer(100ms, std::bind(&LatticePlanner::planner_tracking_iteration_callback, this));

    planner_iteration_time_publisher = this->create_publisher<std_msgs::msg::Float32>("planner_iteration_duration", qos_); // 用于统计planner求解时间的广播器

    lattice_planner_path_cartesian_publisher = this->create_publisher<visualization_msgs::msg::Marker>("lattice_planner_path_cardesian", qos_);
    lattice_planner_path_frenet_publisher = this->create_publisher<nav_msgs::msg::Path>("lattice_planner_path_frenet", qos_);

    sensor_fusion_results_bounding_box_subscription_ = this->create_subscription<visualization_msgs::msg::MarkerArray>("sensor_fusion_results_bounding_box", qos_, std::bind(&LatticePlanner::sensor_fusion_results_bounding_box_callback, this, _1));

    sensor_fusion_results_label_subscription_ = this->create_subscription<visualization_msgs::msg::MarkerArray>("sensor_fusion_results_label", qos_, std::bind(&LatticePlanner::sensor_fusion_results_label_callback, this, _1));

    fsm_behavior_decision_makeing_subscription = this->create_subscription<std_msgs::msg::Int16>("fsm_behavior_decision", qos_, std::bind(&LatticePlanner::fsm_behavior_decision_makeing_callback, this, _1));

    // Declare and initialize a parameter, return the effective value.
    this->declare_parameter<int>("planner_former_point_of_current_position", former_point_of_current_position);
    this->declare_parameter<int>("planner_working_mode", working_mode);

    // Get the value of a parameter by the given name, and return true if it was set.
    this->get_parameter<int>("planner_former_point_of_current_position", this->former_point_of_current_position);
    this->get_parameter<int>("planner_working_mode", this->working_mode);

    RCLCPP_INFO(this->get_logger(), "planner_former_point_of_current_position %d", this->former_point_of_current_position);
    RCLCPP_INFO(this->get_logger(), "planner_working_mode %d", this->working_mode);
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
    if (is_global_path_received)
    {
        rclcpp::Time now = this->now();
        ins_data_arrive_at_planner_through_callback = now.seconds();

        // RCLCPP_INFO(this->get_logger(),"got imu data at: %f", this->now().seconds());

        tf2::Quaternion quat_tf;
        tf2::convert(msg->pose.pose.orientation, quat_tf);
        double roll_current, pitch_current, heading_current;
        a_longitudinal = msg->pose.covariance[0];
        a_lateral = msg->pose.covariance[4];
        px = msg->pose.pose.position.x;
        py = msg->pose.pose.position.y; // 惯导的y向前
        v_longitudinal = msg->twist.twist.linear.x;
        v_lateral = msg->twist.twist.linear.y;
        yaw_rate = msg->twist.twist.angular.z;

        if (v_longitudinal < 0.4 / 3.6)
        {
            v_lateral = 0;
            yaw_rate = 0;
        }

        ins_frame_arrive_time = msg->header.stamp;
        ins_arrive_at_rs232_buffer = this->ins_frame_arrive_time.seconds();

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

        vector<double> car_s_d = cartesian_to_frenet(px, py, psi, global_path_x, global_path_y);
        car_s = car_s_d[0];
        car_d = car_s_d[1];

        RCLCPP_INFO(this->get_logger(), "got imu data: car_s %f, car_d: %f", car_s, car_d);
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
    RCLCPP_INFO(this->get_logger(), "receiveing global path %lu", msg->poses.size());
    int path_length = msg->poses.size();

    global_path_x.clear();
    global_path_y.clear();
    global_path_s.clear();

    for (int i = 0; i < path_length; i++)
    {
        global_path_x.push_back(msg->poses[i].pose.position.x);
        global_path_y.push_back(msg->poses[i].pose.position.y);
        global_path_s.push_back(msg->poses[i].pose.orientation.w);
    }

    is_global_path_received = true;
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
void LatticePlanner::fsm_behavior_decision_makeing_callback(std_msgs::msg::Int16::SharedPtr msg){
    RCLCPP_INFO(this->get_logger(), "~~~~~~~~~~~~~~~~~~~receiveing behavior decision results: %d", msg->data);
    this->current_velocity_behavior = msg->data;
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
            rclcpp::Time now = this->now();
            double ins_parse_now = now.seconds();
            double _max_safe_speed = max_safe_speed;
            
            // 定位延迟补偿发生在将全局路径转换到车辆坐标系下之前,用来补偿定位信息到达早于被使用而引起的定位误差
            if (working_mode == 1) {
                ins_delay = ins_parse_now - ins_data_arrive_at_planner_through_callback + 0.005;
            }
            else if (working_mode == 2) {
                ins_delay = ins_parse_now - ins_arrive_at_rs232_buffer + 0.005;
            }
            /* 全局坐标系下的定位信息延时补偿 */
            psi = psi + yaw_rate * ins_delay;
            px = px + v_longitudinal * cos(psi) * ins_delay - v_lateral * sin(psi) * ins_delay;
            py = py + v_longitudinal * sin(psi) * ins_delay + v_lateral * cos(psi) * ins_delay;

            vector<double> car_s_d = cartesian_to_frenet(px, py, psi, global_path_x, global_path_y);
            car_s = car_s_d[0];
            car_d = car_s_d[1];

            cout << "Current Position in Frenet Coordinate, s: " << car_s << ", d: " << car_d << endl;

            // 找到合适的未来一段峰值速度
            // 从全局路径中，找到距离当前位置最近的前方的点。
            size_t former_point_of_current_position = 0;
            double distance_car_s = v_longitudinal * 6 + 16; // TODO 可以让车提前感知到前方有弯，提前减速
            for (size_t i = 0; i < global_path_s.size(); i++) {
                if (global_path_s[i] - car_s >= -10) // 除了前方的点，车辆后面10m的点也计入统计曲率的范围
                {
                    former_point_of_current_position = i;
                    break;
                }            
            }
            vector<double> cal_curvature_x;
            vector<double> cal_curvature_y;
            vector<double> cal_heading_ratio;
            vector<double> cal_heading_ratio_diff;

            for (size_t i = former_point_of_current_position; global_path_s[i] <= car_s + distance_car_s; i++) {
                cal_curvature_x.push_back(global_path_x[i]);
                cal_curvature_y.push_back(global_path_y[i]);
            }
            for (size_t j = 1; j < cal_curvature_x.size(); j++) {
                cal_heading_ratio.push_back(atan2(cal_curvature_y[j] - cal_curvature_y[j-1], cal_curvature_x[j] - cal_curvature_x[j-1]));
            }
            for (size_t k = 1; k < cal_heading_ratio.size()-1; k++) {
                cal_heading_ratio_diff.push_back(fabs(cal_heading_ratio[k+1] - cal_heading_ratio[k-1])); 
            }
            std::sort(cal_heading_ratio_diff.begin(), cal_heading_ratio_diff.end());

            double max_heading_ratio_diff = cal_heading_ratio_diff.back();
            // cout << "vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv  " << _max_safe_speed << "  " << max_heading_ratio_diff << endl;
            // TODO: 道路曲率影响速度的最小阈值
            if (max_heading_ratio_diff < 0.04)  {
                max_heading_ratio_diff = 0;
            }
            // TODO 控制减速幅值
            _max_safe_speed = _max_safe_speed - max_heading_ratio_diff * 60; 
            if (max_safe_speed > 1.5) {
                _max_safe_speed = max_planner(1.5, _max_safe_speed);
            }
            double _safety_margin = safety_margin + _max_safe_speed * 2.0;

            next_x_vals.clear();
            next_y_vals.clear();
            next_s_vals.clear();
            next_v_vals.clear();
            next_x_vals_previous_remap.clear();
            next_y_vals_previous_remap.clear();

            int object_lane = 0;

            double velocity_of_front_vehicle_in_same_lane = 0;

            int host_lane = which_lane(car_d);

            if (fabs((double)sensor_fusion_results_bounding_box.markers[0].header.stamp.nanosec - (double)sensor_fusion_results_label.markers[0].header.stamp.nanosec) < 1000000) {
                sensor_fusion.clear();
                
                bounding_box_label_same_frame_check_flag = true;
                for (uint i = 0; i < sensor_fusion_results_bounding_box.markers.size(); i++) {
                    Object_Around sensor_fusion_single = Object_Around(sensor_fusion_results_bounding_box.markers[i],
                                                                       sensor_fusion_results_label.markers[i],
                                                                       global_path_s,
                                                                       global_path_x,
                                                                       global_path_y);
                    sensor_fusion.push_back(sensor_fusion_single);
                    // cout << "d:" << sensor_fusion_single.d << ",s:" << sensor_fusion_single.s << ",px:" << sensor_fusion_single.px << ",py:" << sensor_fusion_single.py << ",v_X:" << sensor_fusion_single.v_X << ",v_Y:" << sensor_fusion_single.v_Y << ",v_lon:" << sensor_fusion_single.v_longitudinal << ",v_lat:" << sensor_fusion_single.v_lateral << ",yaw:" << sensor_fusion_single.yaw << ",yaw_rate:" << sensor_fusion_single.yaw_rate << ",L:" << sensor_fusion_single.length << ",W:" << sensor_fusion_single.width << ",H:" << sensor_fusion_single.height << ",Label:" << sensor_fusion_single.label << "." << endl;
                }
            }
            if (bounding_box_label_same_frame_check_flag) {
                Object_Around sensor_fusion_single = sensor_fusion[0];
                for (uint i = 0; i < sensor_fusion.size(); i++) {
                    sensor_fusion_single = sensor_fusion[i];
                    // TODO:Collision Check 
                    if (is_same_lane(sensor_fusion_single.d, car_d)){
                        velocity_of_front_vehicle_in_same_lane = sensor_fusion_single.v_longitudinal;
                    }
                }
            }

            
            // 向左变道
            if (current_velocity_behavior == 1) {
                lane = host_lane - 1;
            }
            // 向右变道
            if (current_velocity_behavior == 2) {
                lane = host_lane + 1;
            }
            // 返回目标车道
            if (current_velocity_behavior == 6) {
                lane = 0;
            }
            // 本车道内跟车行驶，参考速度为前车速度
            if (current_velocity_behavior == 4) {
                if(ref_vel >= velocity_of_front_vehicle_in_same_lane){
                    ref_vel -= 0.3; // 减速策略，匹配该函数的调用频率，可以算得到减速度大小,这个值要和期望速度成倍数，否则停不下来的
                }
            }
            
            // 起步
            if (current_velocity_behavior == 7) {
                ref_vel += 0.1; // TODO:加速策略，起步策略公用，这里可以更加复杂
            }

            if (current_velocity_behavior == 9) {
                if (ref_vel > _max_safe_speed){
                    ref_vel -= 0.2; // TODO:加速策略，起步策略公用，这里可以更加复杂
                }
                else{
                    ref_vel += 0.1;
                }
            }

            // 停车
            if (current_velocity_behavior == 8) {
                ref_vel = 0.00;
            }

            if (ref_vel < 0) {
                ref_vel = 0;
            }
            if (ref_vel > _max_safe_speed) {
                ref_vel = _max_safe_speed;
            }
            
            //  * TODO: define a path made up of (x,y) points that the car will visit sequentially
            vector<double> ptsx;
            vector<double> ptsy;

            double ref_x = px;
            double ref_y = py;
            double ref_yaw = psi;

            // if previous size is almost empty, use the car as starting reference
            // use two points that make the path tangent to the car
            if (next_s_vals_previous.size() < 2) {
                double prev_car_x = px - cos(psi);
                double prev_car_y = py - sin(psi);

                ptsx.push_back(prev_car_x);
                ptsx.push_back(px);

                ptsy.push_back(prev_car_y);
                ptsy.push_back(py);
            }
            else {
                // redefine reference state
                int current_index_in_last_iteration_path = 0;
                for (size_t i = 0; i < next_x_vals_previous.size(); i++) {
                    double shift_x = next_x_vals_previous[i] - px;
                    double shift_y = next_y_vals_previous[i] - py;
                    next_x_vals_previous_remap.push_back(shift_x * cos(psi) + shift_y * sin(psi));
                    next_y_vals_previous_remap.push_back(-shift_x * sin(psi) + shift_y * cos(psi));
                }
                // 从局部路径中，找到距离当前位置最近的前方的点。
                for (size_t i = 0; i < next_x_vals_previous_remap.size(); i++) {
                    if (next_x_vals_previous_remap[i] >= 0.0)
                    {
                        current_index_in_last_iteration_path = i;
                        break;
                    }
                }
                
                ref_x = next_x_vals_previous[current_index_in_last_iteration_path];
                ref_y = next_y_vals_previous[current_index_in_last_iteration_path];
                // cout << "%%%%%%%%%%%%%%%%%%%%%%%%% : " <<  current_index_in_last_iteration_path << "   " << ref_x << "  " << ref_y << "  " << next_x_vals_previous[0]<< endl;
                double ref_x_future = next_x_vals_previous[current_index_in_last_iteration_path + 1];
                double ref_y_future = next_y_vals_previous[current_index_in_last_iteration_path + 1];

                ref_yaw = atan2(ref_y_future - ref_y, ref_x_future - ref_x);

                ptsx.push_back(ref_x);
                ptsx.push_back(ref_x_future);

                ptsy.push_back(ref_y);
                ptsy.push_back(ref_y_future);

                int resuse_number_of_previous_last = min_planner(60, 5 + 60 / v_longitudinal); // TODO:这个值跟着速度变 12
                // int resuse_number_of_previous_last = 20; // TODO:这个值跟着速度变 12
                for (int i = 2; i < resuse_number_of_previous_last; i++) // 因为之前里面已经有两个点了
                {
                    ptsx.push_back(next_x_vals_previous[current_index_in_last_iteration_path + i]);
                    ptsy.push_back(next_y_vals_previous[current_index_in_last_iteration_path + i]);
                }
                double resuse_length = 0;
                for (uint j = 1; j < ptsx.size(); j++)
                {
                    resuse_length += distance_two_point(ptsx[j], ptsy[j], ptsx[j - 1], ptsy[j - 1]);
                }
                cout << "*************** resuse length " << resuse_length << "***************" << endl;
            }
            vector<double> next_wp0;
            vector<double> next_wp1;
            vector<double> next_wp2;
            vector<double> next_wp3;
            vector<double> next_wp4;
            vector<double> next_wp5;
            // cout << "*************** " << fabs(car_d - lane * lane_width / 2)   << "  " <<   fabs(lane * lane_width / 2) << endl;

            // next_wp0 = frenet_to_cartesian(car_s + max_planner(5.5, (v_longitudinal * 3.5 + 6.5)) * 0.5, (lane * lane_width * 0.5), global_path_s, global_path_x, global_path_y); // TODO:这个值跟着速度变
            next_wp1 = frenet_to_cartesian(car_s + max_planner(5.5, (v_longitudinal * 3.4 + 6.5 - max_heading_ratio_diff * 40)), (lane * lane_width), global_path_s, global_path_x, global_path_y); 
            next_wp2 = frenet_to_cartesian(car_s + max_planner(5.7, (v_longitudinal * 3.4 + 6.7 - max_heading_ratio_diff * 40)), (lane * lane_width), global_path_s, global_path_x, global_path_y);
            next_wp3 = frenet_to_cartesian(car_s + max_planner(5.9, (v_longitudinal * 3.4 + 6.9 - max_heading_ratio_diff * 40)), (lane * lane_width), global_path_s, global_path_x, global_path_y);
            next_wp4 = frenet_to_cartesian(car_s + max_planner(6.1, (v_longitudinal * 3.4 + 7.1 - max_heading_ratio_diff * 40)), (lane * lane_width), global_path_s, global_path_x, global_path_y);
            next_wp5 = frenet_to_cartesian(car_s + max_planner(6.3, (v_longitudinal * 3.4 + 7.3 - max_heading_ratio_diff * 40)), (lane * lane_width), global_path_s, global_path_x, global_path_y);

            // ptsx.push_back(next_wp0[0]);
            ptsx.push_back(next_wp1[0]);
            ptsx.push_back(next_wp2[0]);
            ptsx.push_back(next_wp3[0]);
            ptsx.push_back(next_wp4[0]);
            ptsx.push_back(next_wp5[0]);

            // ptsy.push_back(next_wp0[1]);
            ptsy.push_back(next_wp1[1]);
            ptsy.push_back(next_wp2[1]);
            ptsy.push_back(next_wp3[1]);
            ptsy.push_back(next_wp4[1]);
            ptsy.push_back(next_wp5[1]);

            // 从世界坐标系变换到车辆坐标系(不严格是车辆坐标系，坐标原点修正到了历史轨迹同轴线上)
            for (uint i = 0; i < ptsx.size(); i++)
            {
                // shift car reference angle to 0 degrees
                double shift_x = ptsx[i] - ref_x;
                double shift_y = ptsy[i] - ref_y;

                ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
                ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
            }

            // create a spline
            tk::spline s;
            // set (x,y) points to the spline
            s.set_points(ptsx, ptsy);

            // Calculate how to break up spline points so that we travel at our desired reference velocity
            double target_x = max_planner(5.4, v_longitudinal * 3.4 + 6 - max_heading_ratio_diff * 40); // 相当于每次规划都是只规划到前方20m的位置 // TODO:这个值跟着速度变
            double target_y = s(target_x);
            double target_dist = sqrt(pow(target_x, 2) + pow(target_y, 2));

            // 根据当前车辆的期望速度，可以算得到 N 为在目标距离内，生成的点的路径点的数量
            double N = floor(min_planner((target_dist / (0.04 * (ref_vel + 0.01))), 300.0)); // 期望速度单位 m/s // 刚开始的时候速度很低，会导致点特点多

            double x_add_on = 0;
            // file up the rest of our path planner after filling it with previous points, here we will always output 50 points
            double x_point = 0;
            double x_point_last;
            double y_point_last;
            double point_distance_x = target_x / N;
            for (int i = 0; i < N; i++) {
                double y_point = s(x_point);
                x_add_on = x_point;

                double x_ref = x_point;
                double y_ref = y_point;

                // 前面的计算是在车辆坐标系下的（或者是在上一次生成路径点最后的两个点形成的坐标系下），这里转回到世界坐标系下
                x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
                y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

                x_point += ref_x;
                y_point += ref_y;

                next_x_vals.push_back(x_point);
                next_y_vals.push_back(y_point);

                if (i == 0){
                    vector<double> s_d = cartesian_to_frenet(x_point, y_point, ref_yaw, global_path_x, global_path_y);
                    double _s = s_d[0];
                    // double d = s_d[1];
                    next_s_vals.push_back(_s);
                    next_v_vals.push_back(ref_vel);
                }
                else{
                    double future_heading = atan2(y_point - y_point_last, x_point - x_point_last);
                    vector<double> s_d = cartesian_to_frenet(x_point, y_point, future_heading, global_path_x, global_path_y);
                    double _s = s_d[0];
                    // double d = s_d[1];
                    next_s_vals.push_back(_s);
                    next_v_vals.push_back(ref_vel);
                }
                x_point_last = x_point;
                y_point_last = y_point;
                
                x_point = x_add_on + point_distance_x;
            }
            // cout << *next_x_vals.begin() << " &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&" << endl;


            // TODO:打包消息发出去, 规划器发出的结果是在全局笛卡尔坐标系中的
            
            lattice_planner_path_id ++;

            lattice_planner_path_cardesian.id = lattice_planner_path_id;
            lattice_planner_path_cardesian.header.stamp = this->get_clock()->now();
            lattice_planner_path_cardesian.header.frame_id = "odom";
            lattice_planner_path_cardesian.type = visualization_msgs::msg::Marker::LINE_STRIP;
            lattice_planner_path_cardesian.action = visualization_msgs::msg::Marker::ADD;
            lattice_planner_path_cardesian.lifetime = rclcpp::Duration(0s);
            lattice_planner_path_cardesian.scale.x = 0.02;
            lattice_planner_path_cardesian.scale.y = 0.02;
            lattice_planner_path_cardesian.scale.z = 0.02;
            lattice_planner_path_cardesian.color.r = 1.0;
            lattice_planner_path_cardesian.color.g = 1.0;
            lattice_planner_path_cardesian.color.b = 1.0;
            lattice_planner_path_cardesian.color.a = 1.0;

            lattice_planner_path_cardesian.points.clear();

            for (size_t i = 0; i < next_x_vals.size(); i++) {
                highway_with_prediction_planner_point_cartesian.x = next_x_vals[i];
                highway_with_prediction_planner_point_cartesian.y = next_y_vals[i];
                highway_with_prediction_planner_point_cartesian.z = 0;

                lattice_planner_path_cardesian.points.push_back(highway_with_prediction_planner_point_cartesian);
            }
            
            // TODO:打包消息发出去, 规划器发出的结果是在全局Frenet坐标系中的
            lattice_planner_path_frenet.poses.clear();

            lattice_planner_path_frenet.header.stamp = this->get_clock()->now();
            lattice_planner_path_frenet.header.frame_id = "odom";
            for (size_t i = 0; i < next_x_vals.size(); i++) {
                highway_with_prediction_planner_point_frenet.header.frame_id = "odom";
                highway_with_prediction_planner_point_frenet.header.stamp = this->get_clock()->now();

                highway_with_prediction_planner_point_frenet.pose.position.x = next_s_vals[i];
                highway_with_prediction_planner_point_frenet.pose.position.y = next_v_vals[i];
                highway_with_prediction_planner_point_frenet.pose.position.z = 0;

                lattice_planner_path_frenet.poses.push_back(highway_with_prediction_planner_point_frenet);
            }
            
            lattice_planner_path_cartesian_publisher->publish(lattice_planner_path_cardesian);
            lattice_planner_path_frenet_publisher->publish(lattice_planner_path_frenet);

            next_x_vals_previous = next_x_vals;
            next_y_vals_previous = next_y_vals;
            next_s_vals_previous = next_s_vals;
            next_v_vals_previous = next_v_vals;
        }
    }
    end_planner = this->now();
    iteration_time_length = (end_planner - start_planner).nanoseconds();
    planner_iteration_duration_msg.data = iteration_time_length / 1000000;
    planner_iteration_time_publisher->publish(planner_iteration_duration_msg);
    RCLCPP_INFO(this->get_logger(), "planner iteration time: %f ms", iteration_time_length / 1000000);
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