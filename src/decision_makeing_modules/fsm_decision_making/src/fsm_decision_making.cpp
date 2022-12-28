/*'''*****************************************************************************************************
# FileName    : 
# FileFunction: 
# Comments    : 
*****************************************************************************************************'''*/
#include "fsm_decision_making/json.hpp"
#include "fsm_decision_making/spline.h"
#include "fsm_decision_making/fsm_decision_making.h"

/* For converting back and forth between radians and degrees. */

static const rclcpp::Logger LOGGER = rclcpp::get_logger("fsm_decision_making");

double inline min_planner(double a, double b) { return (a < b) ? a : b; }
double inline max_planner(double a, double b) { return (a > b) ? a : b; }

/*'''**************************************************************************************
- FunctionName: None
- Function    : 构造函数
- Inputs      : None
- Outputs     : None
- Comments    : None
**************************************************************************************'''*/
FSMDecisionMaking::FSMDecisionMaking() : Node("fsm_decision_making") // 使用初始化列表来初始化字段
{
    ins_data_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>("ins_d_of_vehicle_pose", qos_, std::bind(&FSMDecisionMaking::ins_data_receive_callback, this, _1));
    global_path_subscription_ = this->create_subscription<nav_msgs::msg::Path>("global_path", qos_, std::bind(&FSMDecisionMaking::global_path_callback, this, _1));
    
    sensor_fusion_results_bounding_box_subscription_ = this->create_subscription<visualization_msgs::msg::MarkerArray>("sensor_fusion_results_bounding_box", qos_, std::bind(&FSMDecisionMaking::sensor_fusion_results_bounding_box_callback, this, _1));
    sensor_fusion_results_label_subscription_ = this->create_subscription<visualization_msgs::msg::MarkerArray>("sensor_fusion_results_label", qos_, std::bind(&FSMDecisionMaking::sensor_fusion_results_label_callback, this, _1));
    
    // fsm_decision_making_path_cartesian_publisher = this->create_publisher<visualization_msgs::msg::Marker>("fsm_decision_making_path_cardesian", qos_);
    // fsm_decision_making_path_frenet_publisher = this->create_publisher<nav_msgs::msg::Path>("fsm_decision_making_path_frenet", qos_);

    fsm_behavior_decision_makeing_publisher = this->create_publisher<std_msgs::msg::Int16>("fsm_behavior_decision", qos_);


    planner_iteration_timer_ = this->create_wall_timer(100ms, std::bind(&FSMDecisionMaking::decision_iteration_callback, this));
    planner_iteration_time_publisher = this->create_publisher<std_msgs::msg::Float32>("planner_iteration_duration", qos_); // 用于统计planner求解时间的广播器

}

/*'''**************************************************************************************
- FunctionName: None
- Function    : None
- Inputs      : None
- Outputs     : None
- Comments    : None
**************************************************************************************'''*/
FSMDecisionMaking::~FSMDecisionMaking() {}
/*'''**************************************************************************************
- FunctionName: None
- Function    : None
- Inputs      : None
- Outputs     : None
- Comments    : 这个回调函数发出去的psi的值应该是弧度单位的
**************************************************************************************'''*/
void FSMDecisionMaking::ins_data_receive_callback(nav_msgs::msg::Odometry::SharedPtr msg)
{
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
        // cout << "***ins_ data::: " << "a_lateral: " << a_lateral << ", v_lateral: " << v_lateral << ", yaw_rate: " << yaw_rate << ", psi: " << psi << ", px:" << px << ", py" << py << endl;
        is_ins_data_received = true;

        vector<double> car_s_d = cartesian_to_frenet(px, py, psi, global_path_x, global_path_y);
        car_s = car_s_d[0];
        car_d = car_s_d[1];

        // RCLCPP_INFO(this->get_logger(), "got imu data: car_s %f, car_d: %f", car_s, car_d);
    }
}

/*'''**************************************************************************************
- FunctionName: None
- Function    : None
- Inputs      : None
- Outputs     : None
- Comments    : None
**************************************************************************************'''*/
void FSMDecisionMaking::global_path_callback(nav_msgs::msg::Path::SharedPtr msg)
{
    // RCLCPP_INFO(this->get_logger(), "receiveing global path %lu", msg->poses.size());
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
void FSMDecisionMaking::sensor_fusion_results_bounding_box_callback(visualization_msgs::msg::MarkerArray::SharedPtr msg)
{
    sensor_fusion_results_bounding_box = *msg;
    is_sensor_fusion_results_bounding_box_reveived = true;
    // RCLCPP_INFO(this->get_logger(), "receiveing sensor fusion bounding box %u", msg->markers[0].header.stamp.nanosec);
}

/*'''**************************************************************************************
- FunctionName: None
- Function    : None
- Inputs      : None
- Outputs     : None
- Comments    : None
**************************************************************************************'''*/
void FSMDecisionMaking::sensor_fusion_results_label_callback(visualization_msgs::msg::MarkerArray::SharedPtr msg)
{
    sensor_fusion_results_label = *msg;
    is_sensor_fusion_results_label_received = true;
    // RCLCPP_INFO(this->get_logger(), "receiveing sensor fusion label %u", msg->markers[0].header.stamp.nanosec);
}

/*'''**************************************************************************************
- FunctionName: None
- Function    : None
- Inputs      : None
- Outputs     : None
- Comments    : None
**************************************************************************************'''*/
void FSMDecisionMaking::decision_iteration_callback(){
    rclcpp::Time start_planner;
    rclcpp::Time end_planner;
    start_planner = this->now();
    double iteration_time_length;
    if (rclcpp::ok()){
        if (is_global_path_received && is_ins_data_received && is_sensor_fusion_results_bounding_box_reveived && is_sensor_fusion_results_label_received){
            current_velocity_behavior = 9; 
            rclcpp::Time now = this->now();
            double ins_parse_now = now.seconds();
            double _max_safe_speed = max_safe_speed;
            
            // 定位延迟补偿发生在将全局路径转换到车辆坐标系下之前,用来补偿定位信息到达早于被使用而引起的定位误差
            if (working_mode == 1){
                ins_delay = ins_parse_now - ins_data_arrive_at_planner_through_callback + 0.005;
            }
            else if (working_mode == 2){
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

            //TODO: 采取规避动作的极限距离体现了不同激进程度的驾驶风格
            double _safety_margin = safety_margin + v_longitudinal * 2.0;

            next_x_vals.clear();
            next_y_vals.clear();
            next_s_vals.clear();
            next_v_vals.clear();
            next_x_vals_previous_remap.clear();
            next_y_vals_previous_remap.clear();

            bool is_too_close = false;
            bool prepare_for_lane_change = false;
            bool ready_for_lane_change = false;
            bool is_left_lane_free = true;
            bool is_right_lane_free = true;
            bool time_to_back_reference_lane = false;
            bool is_road_free = false;
            bool front_ready_back_to_ref = true;
            bool rear_ready_back_to_ref = true;
            bool need_to_slow_down = false;
            bool current_lane_free = false;

            

            if (fabs((double)sensor_fusion_results_bounding_box.markers[0].header.stamp.nanosec - (double)sensor_fusion_results_label.markers[0].header.stamp.nanosec) < 1000000){
                sensor_fusion.clear();
                bounding_box_label_same_frame_check_flag = true;
                for (uint i = 0; i < sensor_fusion_results_bounding_box.markers.size(); i++){
                    Object_Around sensor_fusion_single = Object_Around(sensor_fusion_results_bounding_box.markers[i],
                                                                       sensor_fusion_results_label.markers[i],
                                                                       global_path_s,
                                                                       global_path_x,
                                                                       global_path_y);
                    sensor_fusion.push_back(sensor_fusion_single);
                    // cout << "d:" << sensor_fusion_single.d << ",s:" << sensor_fusion_single.s << ",px:" << sensor_fusion_single.px << ",py:" << sensor_fusion_single.py << ",v_X:" << sensor_fusion_single.v_X << ",v_Y:" << sensor_fusion_single.v_Y << ",v_lon:" << sensor_fusion_single.v_longitudinal << ",v_lat:" << sensor_fusion_single.v_lateral << ",yaw:" << sensor_fusion_single.yaw << ",yaw_rate:" << sensor_fusion_single.yaw_rate << ",L:" << sensor_fusion_single.length << ",W:" << sensor_fusion_single.width << ",H:" << sensor_fusion_single.height << ",Label:" << sensor_fusion_single.label << "." << endl;
                }
            }
            if (bounding_box_label_same_frame_check_flag){
                Object_Around sensor_fusion_single = sensor_fusion[0];
                for (uint i = 0; i < sensor_fusion.size(); i++){
                    sensor_fusion_single = sensor_fusion[i];

                    if (is_same_lane(sensor_fusion_single.d, car_d)){
                        bool is_in_front_of_us = sensor_fusion_single.s > car_s;
                        
                        bool is_close_than_safety_margin = (sensor_fusion_single.s - car_s) < _safety_margin;

                        if (is_in_front_of_us && is_close_than_safety_margin){
                            is_too_close = true;
                            prepare_for_lane_change = true;
                            current_lane_free = false;
                        }
                        else{
                            current_lane_free = true;
                            current_velocity_behavior = 9; 
                        }
                    }
                }
                
                if (which_lane(car_d) != preference_lane_id){
                    time_to_back_reference_lane = true;
                }
                if (prepare_for_lane_change){
                    int num_vehicles_left = 0;
                    int num_vehicles_right = 0;
                    // check if left and right lanes are free
                    for (uint i = 0; i < sensor_fusion.size(); i++){
                        sensor_fusion_single = sensor_fusion[i];
                        object_lane = which_lane(sensor_fusion_single.d);
                        host_lane = which_lane(car_d);
                        if ((object_lane - host_lane) == -1){
                            num_vehicles_left++;
                            // 旁边车道有车，要求旁边车道的车至少比本车道内的障碍物距离主车距离远10m，才可能允许变道
                            bool too_close_to_change = (sensor_fusion_single.s > (car_s - _safety_margin - sensor_fusion_single.length)) && (sensor_fusion_single.s < (car_s + _safety_margin + 10));
                            if (too_close_to_change){
                                is_left_lane_free = false;
                            }
                        } 
                        // check right lane , 当希望变道的时候，优先向左变道，左边不行的时候，再往右变                    
                        else if ((object_lane - host_lane) == 1){
                            num_vehicles_right++;
                            bool too_close_to_change = (sensor_fusion_single.s > (car_s - _safety_margin - sensor_fusion_single.length)) && (sensor_fusion_single.s < (car_s + _safety_margin + 10));
                            if (too_close_to_change){
                                is_right_lane_free = false;
                            }
                        }
                        // 先确定能不能变道，再说往哪边变
                        if (is_left_lane_free || is_right_lane_free) {
                            ready_for_lane_change = true;
                            prepare_for_lane_change = false;
                        }
                    }
                }
                if (is_too_close) {
                    need_to_slow_down = true; 
                    current_velocity_behavior = 4; 
                }
                if (time_to_back_reference_lane) {
                    int num_vehicles_left = 0;
                    int num_vehicles_right = 0;
                    // check if left and right lanes are free
                    for (uint i = 0; i < sensor_fusion.size(); i++) {
                        sensor_fusion_single = sensor_fusion[i];
                        // 前方的车要足够远，后方的车也要足够远
                        if (sensor_fusion_single.s > car_s)  {
                            front_ready_back_to_ref = (sensor_fusion_single.s > (car_s + _safety_margin)) && front_ready_back_to_ref;
                        }
                        else {
                            rear_ready_back_to_ref = (sensor_fusion_single.s < (car_s - _safety_margin * 0.3)) && rear_ready_back_to_ref;
                        }
                    }
                    if (front_ready_back_to_ref && rear_ready_back_to_ref) {
                        is_road_free = true;
                        current_velocity_behavior = 6;
                        lane = preference_lane_id;
                    }

                    
                }
            }
            // cout << "****************************************************************" << endl;
            std::cout << "  ~~~~~~~~~~~~~ Host Lane: " << which_lane(car_d) << std::endl;
            // std::cout << "LEFT: " << num_vehicles_left << "  RIGHT: " << num_vehicles_right << std::endl;
            // cout << "****************************************************************" << endl;
            // acutally perform lane change, 
            // 修改 host lane 的取值可以限制主车变道的可用车道，用于调整是2车道道路还是3车道道路还是4、5车道道路
            // 先捕获到前车，才会触发变道
            if (ready_for_lane_change && is_left_lane_free && which_lane(car_d) >= 0) {
                lane = which_lane(car_d) - 1;
                current_velocity_behavior = 1;
                cout << "!!!!!!!!!!!" << endl;
            }
            else if (ready_for_lane_change && is_right_lane_free && which_lane(car_d) <= 0) {
                lane = which_lane(car_d) + 1;
                current_velocity_behavior = 2;
            }

            // 停车，参考路径前面一定的距离
            if (car_s >= (global_path_s.back() - 50.00)) {
                current_velocity_behavior = 8;
            }

            if (car_s > 0 && fabs(car_d) < 2.0 && (v_longitudinal == 0)){
                current_velocity_behavior = 7;
            }

            behavior_decision_result_msg.data = current_velocity_behavior;
            RCLCPP_INFO(this->get_logger(), "Current Behavior Decision Results:~~~~ %d", behavior_decision_result_msg.data);
            fsm_behavior_decision_makeing_publisher->publish(behavior_decision_result_msg);
        }
    }
    end_planner = this->now();
    iteration_time_length = (end_planner - start_planner).nanoseconds();
    planner_iteration_duration_msg.data = iteration_time_length / 1000000;
    planner_iteration_time_publisher->publish(planner_iteration_duration_msg);
    // RCLCPP_INFO(this->get_logger(), "Planner Iteration Time: %f ms", iteration_time_length / 1000000);
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

    auto n = std::make_shared<FSMDecisionMaking>(); // n指向一个值初始化的 MpcTrajectoryTrackingPublisher

    rclcpp::spin(n); // Create a default single-threaded executor and spin the specified node.

    rclcpp::shutdown();
    return 0;
}