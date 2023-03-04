/*'''*****************************************************************************************************
# FileName    : 
# FileFunction: 
# Comments    : 
*****************************************************************************************************'''*/
#include "lattice_planner/json.hpp"
#include "lattice_planner/spline.h"
#include "lattice_planner/lattice_planner.h"
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
    ins_data_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>("/carla/ego_vehicle/odometry", 10, std::bind(&LatticePlanner::ins_data_receive_callback, this, _1)); // carla
    // ins_data_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>("ins_d_of_vehicle_pose", qos_, std::bind(&LatticePlanner::ins_data_receive_callback, this, _1));

    global_path_subscription_ = this->create_subscription<nav_msgs::msg::Path>("global_path", qos_, std::bind(&LatticePlanner::global_path_callback, this, _1));
    sensor_fusion_results_bounding_box_subscription_ = this->create_subscription<visualization_msgs::msg::MarkerArray>("sensor_fusion_results_bounding_box", qos_, std::bind(&LatticePlanner::sensor_fusion_results_bounding_box_callback, this, _1));
    sensor_fusion_results_label_subscription_ = this->create_subscription<visualization_msgs::msg::MarkerArray>("sensor_fusion_results_label", qos_, std::bind(&LatticePlanner::sensor_fusion_results_label_callback, this, _1));
    fsm_behavior_decision_makeing_subscription = this->create_subscription<behavior_decision_interface::msg::FSMDecisionResults>("fsm_behavior_decision", qos_, std::bind(&LatticePlanner::fsm_behavior_decision_makeing_callback, this, _1));

    planner_iteration_timer_ = this->create_wall_timer(100ms, std::bind(&LatticePlanner::planner_tracking_iteration_callback, this));

    planner_iteration_time_publisher = this->create_publisher<std_msgs::msg::Float32>("planner_iteration_duration", qos_); // 用于统计planner求解时间的广播器
    lattice_planner_path_cartesian_publisher = this->create_publisher<visualization_msgs::msg::Marker>("lattice_planner_path_cardesian", qos_);
    lattice_planner_path_frenet_publisher = this->create_publisher<nav_msgs::msg::Path>("lattice_planner_path_frenet", qos_);

    this->declare_parameter<int>("planner_former_point_of_current_position", former_point_of_current_position);
    this->declare_parameter<int>("planner_working_mode", working_mode);
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
    if (is_global_path_received){
        vehicleState_.ax = msg->pose.covariance[0];
        vehicleState_.ay = msg->pose.covariance[4];
        vehicleState_.x = msg->pose.pose.position.x;
        vehicleState_.y = msg->pose.pose.position.y;
        vehicleState_.vx = msg->twist.twist.linear.x;
        vehicleState_.vy = msg->twist.twist.linear.y;
        vehicleState_.velocity = std::sqrt(vehicleState_.vx * vehicleState_.vx + vehicleState_.vy * vehicleState_.vy + vehicleState_.vz * vehicleState_.vz);    // 本车速度
        vehicleState_.angular_velocity = msg->twist.twist.angular.z;

        RCLCPP_INFO(this->get_logger(), "velocity receiveing from ins: %f", this->v_longitudinal);

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
    RCLCPP_INFO(this->get_logger(), "receiveing global path %lu", msg->poses.size());
    int path_length = msg->poses.size();
    global_path_x.clear();
    global_path_y.clear();
    v_points.clear();
    xy_points.clear();
    global_reference_trajectory.trajectory_points.clear();
    for (int i = 0; i < path_length; i++){
        global_path_x.push_back(msg->poses[i].pose.position.x);
        global_path_y.push_back(msg->poses[i].pose.position.y);
        global_path_s.push_back(msg->poses[i].pose.orientation.w);

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

        // for (size_t i = 0; i < headings.size(); i++) {
        //     std::cout << "pt " << i << " heading: " << headings[i] << " acc_s: " << accumulated_s[i] << " kappa: " << kappas[i] << " dkappas: " << dkappas[i] << std::endl;
        // }

        // size_t _count_points = headings.size();
        // size_t _stop_begin_point = ceil(_count_points * 0.85);
        // size_t _stop_point = ceil(_count_points * 0.95);
        // std::cout << "slow down points:" << _stop_begin_point << "  " << _stop_point << std::endl; 

        int _index_before_stop = 0;
        for (size_t i = 0; i < headings.size(); i++) {
            // cout << i << endl;
            TrajectoryPoint trajectory_pt;
            trajectory_pt.x = xy_points[i].first;
            trajectory_pt.y = xy_points[i].second;

            // wx_.push_back(xy_points[i].first);
            // wy_.push_back(xy_points[i].second);
            trajectory_pt.v = v_points[i];


            // if (i < _stop_begin_point) {
            //     trajectory_pt.v = v_points[i];
            //     _index_before_stop++;
            // } else {
            //     if (trajectory_pt.v > 1.0) {
            //         trajectory_pt.v = v_points[_index_before_stop] * ((double)i / ((double)_stop_begin_point - (double)_stop_point) - (double)_stop_point / ((double)_stop_begin_point - (double)_stop_point));
            //     } else {
            //         trajectory_pt.v = 0;
            //     }
            // }
            trajectory_pt.a = 0.0;
            trajectory_pt.heading = headings[i];
            trajectory_pt.kappa = kappas[i];

            global_reference_trajectory.trajectory_points.push_back(trajectory_pt);
        }
        goal_point = global_reference_trajectory.trajectory_points.back();
        trajectory_points_ = global_reference_trajectory.trajectory_points;

        get_way_points(); // 对全局路径点进行稀疏化，加快构造曲线的速度

        cout << "%%%%%%%%%%%%% " << endl;
        // 构建相对平滑的Frenet曲线坐标系，一个中间暂时方案
        csp_obj_ = new Spline2D(wx_, wy_);

        cout << "124324233￥￥" << endl;
        // 构造全局路径变量
        generate_global_path();
        cout << "555555555" << endl;


        //  Update Obstacle 添加虚拟障碍物
        update_static_obstacle();
    }
}

/*'''**************************************************************************************
- FunctionName: None
- Function    : None
- Inputs      : None
- Outputs     : None
- Comments    : TBD： 刷新障碍物的时候，保留历史10个消息内的障碍物，避免单帧的漏检引起的安全隐患
**************************************************************************************'''*/
void LatticePlanner::sensor_fusion_results_bounding_box_callback(visualization_msgs::msg::MarkerArray::SharedPtr msg) {
    sensor_fusion_results_bounding_box = *msg;
    is_sensor_fusion_results_bounding_box_reveived = true;

    if (is_sensor_fusion_results_label_received && is_global_path_received){
        sensor_fusion_callback_cnt ++ ;
        if (sensor_fusion_callback_cnt < 5){
            sensor_fusion_callback_cnt ++ ;
        }
        else if (sensor_fusion_callback_cnt >= 5){
            // sensor_fusion_callback_cnt = 0;
            obstcle_list_.erase(obstcle_list_.begin(), obstcle_list_.begin() + senson_fusion_obstacles_cnt_[0]);
            senson_fusion_obstacles_cnt_.erase(senson_fusion_obstacles_cnt_.begin());
            obstacle_list_more_information.erase(obstacle_list_more_information.begin(), obstacle_list_more_information.begin() + senson_fusion_obstacles_cnt_[0]);
        }
        RCLCPP_INFO(this->get_logger(), "receiveing sensor fusion bounding box %u", msg->markers[0].header.stamp.nanosec);
        for (uint i = 0; i < sensor_fusion_results_bounding_box.markers.size(); i++){
            cout << "iiiii" << i << endl;
            senson_fusion_obstacles_cnt_.push_back(sensor_fusion_results_bounding_box.markers.size());
            Object_Around sensor_fusion_single = Object_Around(sensor_fusion_results_bounding_box.markers[i],
                                                                        sensor_fusion_results_label.markers[i],
                                                                        global_path_s,
                                                                        global_path_x,
                                                                        global_path_y);
            cout << "999999 current longitudinal velocity:: >> " << sensor_fusion_single.v_longitudinal * 3.6 << ", SSSSSSSSSSS <<" << sensor_fusion_single.s << endl;
            Poi_f obstcles{sensor_fusion_single.px, sensor_fusion_single.py}; 
            obstcle_list_.push_back(obstcles);
            obstacle_list_more_information.push_back(sensor_fusion_single);
            cout << "11212 obstacle_list_more_information size << " << obstacle_list_more_information.size() << endl;

        }
        cout << "$$$$$$$$$$$$$ obstcle_list_ << " <<  obstcle_list_.size() << endl;


    }

    // std::vector<Poi_f> obstcles{}; 
    // obstcle_list_ = obstcles;
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
void LatticePlanner::fsm_behavior_decision_makeing_callback(behavior_decision_interface::msg::FSMDecisionResults::SharedPtr msg){
    RCLCPP_INFO(this->get_logger(), "receiveing behavior decision results: %d", msg->target_behavior);
    this->current_decision_behavior = msg->target_behavior;
    this->target_lane = msg->target_lane;
    this->target_velocity_under_following_from_fsm = msg->target_velocity;
    this->target_distance_under_following_from_fsm_under_frenet = msg->target_distance_under_frenet;
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
    // if (0) {
        if (is_global_path_received && is_ins_data_received && is_sensor_fusion_results_bounding_box_reveived && is_sensor_fusion_results_label_received) {
            // rclcpp::Time now = this->now();
            // double ins_parse_now = now.seconds();
            // double _max_safe_speed = max_safe_speed;
            // int current_index_in_last_iteration_path = 0;
            // int ref_index_in_front_of_host_vehicle;
            // double host_car_s = 0;
            // double resuse_length = 0;
            // cout << "1111111111" << endl;
            // TODO:这里之后可以再被优化，采用更好的Frenet坐标系取点方式。
            const double ego_s = get_nearest_reference_length(vehicleState_);
            const double ego_l = get_nearest_reference_lat_dist(vehicleState_);
            const double ego_speed = vehicleState_.velocity;

            // cout << "222" << endl;

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
            FrenetPath final_path = frenet_optimal_trajectory.frenet_optimal_planning(*csp_obj_, s0_, c_speed_, c_d_, c_d_d_, c_d_dd_, obstcle_list_, 
                                                                                        this->current_decision_behavior,
                                                                                        this->target_lane,
                                                                                        this->target_velocity_under_following_from_fsm,
                                                                                        this->target_distance_under_following_from_fsm_under_frenet);
            if (!final_path.s.empty() && !near_goal_) {
                s0_ = final_path.s[1];
                c_d_ = final_path.d[1];
                c_d_d_ = final_path.d_d[1];
                c_d_dd_ = final_path.d_dd[1];
                c_speed_ = final_path.s_d[1];

                const auto trajectory = get_trajectory_from_frenet_path(final_path);
                planningPublishedTrajectoryDebug_ = trajectory;
                last_trajectory_ = trajectory;

                if (std::abs(final_path.s.back() - end_s_) < 2.0) {
                    RCLCPP_INFO(LOGGER, "Near Goal");
                    near_goal_ = true;
                }
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
                float rrrr =  60.0 / 255.0, gggg = 179.0 / 255.0, bbbb = 113.0 / 255.0;
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
                    cout << "final_path.s_d[i] :: " << final_path.s_d[i] << endl;
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
int LatticePlanner::get_nearest_reference_index(const VehicleState &ego_state) {
    double min_dist = std::numeric_limits<double>::max();
    size_t min_index = 0;

    for (size_t i = 0; i < global_plan_.poses.size(); ++i) {
        const double distance = distance_X_Y(ego_state, global_plan_.poses[i].pose.position);
        if (distance < min_dist) {
            min_dist = distance;
            min_index = i;
        }
    }
    return min_index;
}

/*'''**************************************************************************************
- FunctionName: None
- Function    : None
- Inputs      : None
- Outputs     : None
- Comments    : None
**************************************************************************************'''*/
double LatticePlanner::get_nearest_reference_length(const VehicleState &ego_state) {
    return global_plan_.poses[get_nearest_reference_index(ego_state)].pose.position.z;    // s存在position.z中
}

/*'''**************************************************************************************
- FunctionName: None
- Function    : None
- Inputs      : None
- Outputs     : None
- Comments    : None
**************************************************************************************'''*/
double LatticePlanner::get_nearest_reference_lat_dist(const VehicleState &ego_state) {
    double min_dist = std::numeric_limits<double>::max();
    size_t min_index = 0;

    for (size_t i = 0; i < global_plan_.poses.size() - 1; ++i) {
        const double distance = distance_X_Y(ego_state, global_plan_.poses[i].pose.position);
        if (distance < min_dist) {
            min_dist = distance;
            min_index = i;
        }
    }
    const int sign = left_of_line(ego_state, global_plan_.poses[min_index], global_plan_.poses[min_index + 1]) ? 1 : -1;
    return sign * min_dist;
}
/*'''**************************************************************************************
- FunctionName: None
- Function    : None
- Inputs      : None
- Outputs     : None
- Comments    : None
**************************************************************************************'''*/
bool LatticePlanner::left_of_line(const VehicleState &p, const geometry_msgs::msg::PoseStamped &p1, const geometry_msgs::msg::PoseStamped &p2) {
    const double tmpx = (p1.pose.position.x - p.x) * (p2.pose.position.y-p.y) - (p1.pose.position.y - p.y) * (p2.pose.position.x - p.x);
    if (tmpx > 0.0){
        return true;
    }
    else{
        return false;
    }
}

/*'''**************************************************************************************
- FunctionName: None
- Function    : None
- Inputs      : None
- Outputs     : None
- Comments    : None
**************************************************************************************'''*/
TrajectoryData LatticePlanner::get_trajectory_from_frenet_path(const FrenetPath &path) {
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
    }
    return trajectory;
}

/*'''**************************************************************************************
- FunctionName: None
- Function    : None
- Inputs      : None
- Outputs     : None
- Comments    : None
**************************************************************************************'''*/
void LatticePlanner::update_static_obstacle() {
    // std::vector<Poi_f> obstcles{{144.414, 250.23}, {40.1, 212.7}}; //TODO 
    std::vector<Poi_f> obstcles{}; //TODO 
    obstcle_list_ = obstcles;
}

/*'''**************************************************************************************
- FunctionName: None
- Function    : None
- Inputs      : None
- Outputs     : None
- Comments    : None
**************************************************************************************'''*/
void LatticePlanner::generate_global_path() {
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

}

void LatticePlanner::get_way_points() {
    const int refline_size = global_reference_trajectory.trajectory_points.size();
    const auto &trajectory_pt = global_reference_trajectory.trajectory_points;
    double sum_s = 0;
    wx_.clear();
    wy_.clear();
    wx_.push_back(trajectory_pt[0].x);
    wy_.push_back(trajectory_pt[0].y);
    for (int i = 1; i < refline_size; i++) {
        const double dx = trajectory_pt[i].x - trajectory_pt[i - 1].x;
        const double dy = trajectory_pt[i].y - trajectory_pt[i - 1].y;
        const double s = std::sqrt(dx * dx + dy * dy);
        sum_s += s;
        cout << sum_s << endl;
        // 每隔4米的距离进行采点
        if (sum_s > 4.0) {
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
    auto n = std::make_shared<LatticePlanner>(); 
    rclcpp::spin(n); 
    rclcpp::shutdown();
    return 0;
}