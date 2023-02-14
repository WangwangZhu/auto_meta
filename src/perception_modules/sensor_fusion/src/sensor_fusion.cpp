/*'''*****************************************************************************************************
# FileName    : 
# FileFunction:
# Comments    : 
*****************************************************************************************************'''*/
#include "sensor_fusion/sensor_fusion.h"
#include "sensor_fusion/helpers.h"
#include "sensor_fusion/spline.h"
#include "sensor_fusion/coordinate_transform.h"

static const rclcpp::Logger LOGGER = rclcpp::get_logger("sensor_fusion_publisher");

double inline min_sensor(double a, double b) { return (a < b) ? a : b; }
double inline max_sensor(double a, double b) { return (a > b) ? a : b; }

/*'''**************************************************************************************
- FunctionName: None
- Function    : 构造函数
- Inputs      : None
- Outputs     : None
- Comments    : None
**************************************************************************************'''*/
SensorFusion::SensorFusion() : Node("sensor_fusion_publisher")
{
    sensor_fusion_ins_data_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>("ins_d_of_vehicle_pose", qos_, std::bind(&SensorFusion::sensor_fusion_ins_data_receive_callback, this, _1));
    // sensor_fusion_ins_data_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>("/carla/ego_vehicle/odometry", qos_, std::bind(&SensorFusion::sensor_fusion_ins_data_receive_callback, this, _1));

    sensor_fusion_global_path_subscription_ = this->create_subscription<nav_msgs::msg::Path>("global_path", qos_, std::bind(&SensorFusion::sensor_fusion_global_path_callback, this, _1));

    // sensor_fusion_iteration_timer_ = this->create_wall_timer(100ms, std::bind(&SensorFusion::sensor_fusion_iteration_callback, this));

    sensor_fusion_objects_from_carla = this->create_subscription<derived_object_msgs::msg::ObjectArray>("/carla/objects", qos_, std::bind(&SensorFusion::sensor_fusion_iteration_callback, this, _1));

    sensor_fusion_iteration_time_publisher = this->create_publisher<std_msgs::msg::Float32>("sensor_fusion_iteration_duration", qos_);

    sensor_fusion_results_bounding_box_publisher = this->create_publisher<visualization_msgs::msg::MarkerArray>("sensor_fusion_results_bounding_box", qos_);

    sensor_fusion_results_label_publisher = this->create_publisher<visualization_msgs::msg::MarkerArray>("sensor_fusion_results_label", qos_);
}

/*'''**************************************************************************************
- FunctionName: None
- Function    : None
- Inputs      : None
- Outputs     : None
- Comments    : None
**************************************************************************************'''*/
SensorFusion::~SensorFusion() {}
/*'''**************************************************************************************
- FunctionName: None
- Function    : None
- Inputs      : None
- Outputs     : None
- Comments    : 这个回调函数发出去的psi的值应该是弧度单位的
                定位信息回调函数里面需要计算frenet坐标系下的车辆定位信息，因此一定要等收到全局地图后才能正常进入回调
**************************************************************************************'''*/
void SensorFusion::sensor_fusion_ins_data_receive_callback(nav_msgs::msg::Odometry::SharedPtr msg)
{
    if (is_global_path_received)
    {
        rclcpp::Time now = this->now();
        ins_data_arrive_at_sensor_fusion_through_callback = now.seconds();

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

        if (1) // 除了用四元数转换的方法接收角度外，还有备用信息通过方差矩阵传递过来的
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
void SensorFusion::sensor_fusion_global_path_callback(nav_msgs::msg::Path::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "receiveing global path %lu", msg->poses.size());
    int path_length = msg->poses.size();

    global_path_x.clear();
    global_path_y.clear();
    global_path_s.clear();

    for (int i = 0; i < path_length; i++)
    {
        global_path_x.push_back(msg->poses[i].pose.position.x);
        global_path_y.push_back(msg->poses[i].pose.position.y);
        global_path_s.push_back(msg->poses[i].pose.orientation.w); // 这个变量实际携带的参数为 frenet坐标系下的 S 坐标
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
void SensorFusion::sensor_fusion_object_pack(double object_s,
                                             double object_d,
                                             double object_length,
                                             double object_width,
                                             double object_height,
                                             double object_heading,
                                             double object_v_X,
                                             double object_v_Y,
                                             double object_v_yaw_rate,
                                             string object_label,
                                             uint object_id,
                                             double object_line_sacle,
                                             double object_label_scale,
                                             double object_color_r,
                                             double object_color_g,
                                             double object_color_b,
                                             double object_color_a,
                                             visualization_msgs::msg::MarkerArray &sensor_fusion_results_bounding_box_msg,
                                             visualization_msgs::msg::MarkerArray &sensor_fusion_results_label_msg,
                                             string object_frame_id){
    visualization_msgs::msg::Marker sensor_fusion_single_result_bounding_box_msg;
    visualization_msgs::msg::Marker sensor_fusion_single_result_label_msg;
    // 封装 bounding box
    sensor_fusion_single_result_bounding_box_msg.points.clear();
    sensor_fusion_single_result_bounding_box_msg.id = object_id;
    sensor_fusion_single_result_bounding_box_msg.header.frame_id = object_frame_id; // 障碍物的父坐标系为
    sensor_fusion_single_result_bounding_box_msg.header.stamp = this->get_clock()->now();
    sensor_fusion_single_result_bounding_box_msg.type = visualization_msgs::msg::Marker::LINE_LIST;
    sensor_fusion_single_result_bounding_box_msg.action = visualization_msgs::msg::Marker::ADD;
    sensor_fusion_single_result_bounding_box_msg.lifetime = rclcpp::Duration(0ns);
    sensor_fusion_single_result_bounding_box_msg.scale.x = object_line_sacle; // 宽度
    sensor_fusion_single_result_bounding_box_msg.scale.y = object_line_sacle; // 宽度
    sensor_fusion_single_result_bounding_box_msg.scale.z = object_line_sacle; // 宽度
    sensor_fusion_single_result_bounding_box_msg.color.r = object_color_r;    // 颜色
    sensor_fusion_single_result_bounding_box_msg.color.g = object_color_g;    // 颜色
    sensor_fusion_single_result_bounding_box_msg.color.b = object_color_b;    // 颜色
    sensor_fusion_single_result_bounding_box_msg.color.a = object_color_a;    // 透明度

    vector<geometry_msgs::msg::Point> points = get_bounding_box_label_frenet(object_s, object_d, object_length, object_width, object_height, deg2rad(object_heading), global_path_s, global_path_x, global_path_y);

    for (vector<geometry_msgs::msg::Point>::iterator it = points.begin(); it != points.end() - 1; it++) // 减 1 是因为 最后一个点是label的位置点
    {
        sensor_fusion_single_result_bounding_box_msg.points.push_back(*it);
    }

    // 封装 label
    sensor_fusion_single_result_label_msg.points.clear();
    sensor_fusion_single_result_label_msg.id = object_id;
    sensor_fusion_single_result_label_msg.header.frame_id = object_frame_id; // 障碍物的父坐标系为
    sensor_fusion_single_result_label_msg.header.stamp = this->get_clock()->now();
    sensor_fusion_single_result_label_msg.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    sensor_fusion_single_result_label_msg.action = visualization_msgs::msg::Marker::ADD;
    sensor_fusion_single_result_label_msg.lifetime = rclcpp::Duration(0ns);
    sensor_fusion_single_result_label_msg.color.r = object_color_r; // 颜色
    sensor_fusion_single_result_label_msg.color.g = object_color_g; // 颜色
    sensor_fusion_single_result_label_msg.color.b = object_color_b; // 颜色
    sensor_fusion_single_result_label_msg.color.a = object_color_a; // 透明度
    sensor_fusion_single_result_label_msg.scale.x = object_label_scale;
    sensor_fusion_single_result_label_msg.scale.y = object_label_scale;
    sensor_fusion_single_result_label_msg.scale.z = object_label_scale;
    // 下面的几个不代表其原来的物理含义，复用
    geometry_msgs::msg::Point property_1;
    property_1.x = object_v_X;        // 全局坐标系中沿着X方向速度
    property_1.y = object_v_Y;        // 全局坐标系中沿着Y方向速度
    property_1.z = object_v_yaw_rate; // 全局坐标中横摆角速度
    sensor_fusion_single_result_label_msg.points.push_back(property_1);
    geometry_msgs::msg::Point property_2;
    property_2.x = object_s;       // 全局坐标系中车屁股中间的S坐标
    property_2.y = object_d;       // 全局坐标系中车屁股中间的D坐标
    property_2.z = object_heading; // 全局坐标系中车的航向
    sensor_fusion_single_result_label_msg.points.push_back(property_2);

    label_pose.position.x = points.back().x;
    label_pose.position.y = points.back().y;
    label_pose.position.z = points.back().z;
    sensor_fusion_single_result_label_msg.pose = label_pose;
    ostringstream label_str;
    label_str << object_label;
    sensor_fusion_single_result_label_msg.text = label_str.str();

    sensor_fusion_results_label_msg.markers.push_back(sensor_fusion_single_result_label_msg);
    sensor_fusion_results_bounding_box_msg.markers.push_back(sensor_fusion_single_result_bounding_box_msg);
}
/*'''**************************************************************************************
- FunctionName: None
- Function    : None
- Inputs      : None
- Outputs     : None
- Comments    : None
**************************************************************************************'''*/
void SensorFusion::sensor_fusion_iteration_callback(derived_object_msgs::msg::ObjectArray::SharedPtr msg)
{
    rclcpp::Time start_sensor_fusion;
    rclcpp::Time end_sensor_fusion;
    start_sensor_fusion = this->now();
    double iteration_time_length;
    if (rclcpp::ok()){
        if (is_global_path_received && is_ins_data_received){   
            // 每个障碍物对应一个marker
            sensor_fusion_results_bounding_box_msg.markers.clear();
            sensor_fusion_results_label_msg.markers.clear();

            for (int i = 0; i < msg->objects.size(); i++){
                if (distance_two_point(msg->objects[i].pose.position.x, msg->objects[i].pose.position.y, px, py) > 1.0){
                    cout << "carla objects: " << msg->objects[i].pose.position.x << ", " << msg->objects[i].pose.position.y  << ", " << msg->objects[i].pose.position.z << endl;

                    vector<double> car_s_d = cartesian_to_frenet(msg->objects[i].pose.position.x, msg->objects[i].pose.position.y, psi, global_path_x, global_path_y);
                    car_s = car_s_d[0];
                    car_d = car_s_d[1];

                    
                    double object_s = car_s;
                    double object_d = car_d;
                    double object_length = 4.00; // 纵向
                    double object_width = 1.9;   // 横向
                    double object_height = 1.7;
                    double object_heading = 200;
                    double object_v_X = 0;
                    double object_v_Y = 0;
                    double object_v_yaw_rate = 0;
                    uint object_id = 101 + i;
                    string object_label = "Car" + std::to_string(object_id);
                    string object_frame_id = "odom";
                    double object_line_sacle = 0.04; // TODO:need to be checked
                    double object_label_scale = 0.5;
                    double object_color_r = 1.0;
                    double object_color_g = 0.0;
                    double object_color_b = 0.0;
                    double object_color_a = 1.0;
                    this->sensor_fusion_object_pack(object_s, object_d, object_length, object_width, object_height,
                                                    object_heading, object_v_X, object_v_Y, object_v_yaw_rate,
                                                    object_label, object_id,
                                                    object_line_sacle, object_label_scale, object_color_r, object_color_g, object_color_b, object_color_a,
                                                    sensor_fusion_results_bounding_box_msg, sensor_fusion_results_label_msg,
                                                    object_frame_id);
                }
            }
            if (sensor_fusion_results_label_msg.markers.size() > 0){
                sensor_fusion_results_label_publisher->publish(sensor_fusion_results_label_msg);
                sensor_fusion_results_bounding_box_publisher->publish(sensor_fusion_results_bounding_box_msg); // 存放环境感知结果
            }
        }
    }
    end_sensor_fusion = this->now();
    iteration_time_length = (end_sensor_fusion - start_sensor_fusion).nanoseconds();
    sensor_fusion_iteration_duration_msg.data = iteration_time_length / 1000000;
    sensor_fusion_iteration_time_publisher->publish(sensor_fusion_iteration_duration_msg);
    RCLCPP_INFO(this->get_logger(), "sensor fusion iteration time: %f ms", iteration_time_length / 1000000);
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

    auto n = std::make_shared<SensorFusion>(); // n指向一个值初始化的 SensorFusion

    rclcpp::spin(n); // Create a default single-threaded executor and spin the specified node.

    rclcpp::shutdown();
    return 0;
}


                // }
                // if (first_iterion)
                // {
                //     first_iterion = false;
                //     run_start_time_stamp = this->now().seconds();
                // }

            // object_s = 64.660;
            // object_d = +3.5;
            // object_id += 1; // 102
            // object_label = "Car" + std::to_string(object_id);
            // this->sensor_fusion_object_pack(object_s, object_d, object_length, object_width, object_height,
            //                                 object_heading, object_v_X, object_v_Y, object_v_yaw_rate,
            //                                 object_label, object_id,
            //                                 object_line_sacle, object_label_scale, object_color_r, object_color_g, object_color_b, object_color_a,
            //                                 sensor_fusion_results_bounding_box_msg, sensor_fusion_results_label_msg,
            //                                 object_frame_id);

            // // *************** 模拟目标物 3 ***************
            // object_s = 94.660;
            // object_d = 0.0;
            // object_id += 1; // 103
            // object_label = "Car" + std::to_string(object_id);
            // this->sensor_fusion_object_pack(object_s, object_d, object_length, object_width, object_height,
            //                                 object_heading, object_v_X, object_v_Y, object_v_yaw_rate,
            //                                 object_label, object_id,
            //                                 object_line_sacle, object_label_scale, object_color_r, object_color_g, object_color_b, object_color_a,
            //                                 sensor_fusion_results_bounding_box_msg, sensor_fusion_results_label_msg,
            //                                 object_frame_id);
            // // *************** 模拟目标物 4 ***************
            // object_s = 140.660;
            // object_d = -3.5;
            // object_id += 1; // 104
            // object_label = "Car" + std::to_string(object_id);
            // this->sensor_fusion_object_pack(object_s, object_d, object_length, object_width, object_height,
            //                                 object_heading, object_v_X, object_v_Y, object_v_yaw_rate,
            //                                 object_label, object_id,
            //                                 object_line_sacle, object_label_scale, object_color_r, object_color_g, object_color_b, object_color_a,
            //                                 sensor_fusion_results_bounding_box_msg, sensor_fusion_results_label_msg,
            //                                 object_frame_id);

            // // *************** 模拟目标物 4 ***************
            // // object_s = 90.660;
            // // object_d = -3.5;
            // // object_id += 1;
            // // object_label = "Car" + std::to_string(object_id);
            // // this->sensor_fusion_object_pack(object_s, object_d, object_length, object_width, object_height,
            // //                                 object_heading, object_v_X, object_v_Y, object_v_yaw_rate,
            // //                                 object_label, object_id,
            // //                                 object_line_sacle, object_label_scale, object_color_r, object_color_g, object_color_b, object_color_a,
            // //                                 sensor_fusion_results_bounding_box_msg, sensor_fusion_results_label_msg,
            // //                                 object_frame_id);

            // // *************** 模拟目标物 5 ***************
            // object_s = 190.660;
            // object_d = 0.0;
            // object_id += 1; // 105
            // object_label = "Car" + std::to_string(object_id);
            // this->sensor_fusion_object_pack(object_s, object_d, object_length, object_width, object_height,
            //                                 object_heading, object_v_X, object_v_Y, object_v_yaw_rate,
            //                                 object_label, object_id,
            //                                 object_line_sacle, object_label_scale, object_color_r, object_color_g, object_color_b, object_color_a,
            //                                 sensor_fusion_results_bounding_box_msg, sensor_fusion_results_label_msg,
            //                                 object_frame_id);

            // // *************** 模拟目标物 6 ***************
            // object_s = 189.960;
            // object_d = -3.5;
            // object_id += 1;
            // object_label = "Car" + std::to_string(object_id);
            // this->sensor_fusion_object_pack(object_s, object_d, object_length, object_width, object_height,
            //                                 object_heading, object_v_X, object_v_Y, object_v_yaw_rate,
            //                                 object_label, object_id,
            //                                 object_line_sacle, object_label_scale, object_color_r, object_color_g, object_color_b, object_color_a,
            //                                 sensor_fusion_results_bounding_box_msg, sensor_fusion_results_label_msg,
            //                                 object_frame_id);

            // // *************** 模拟目标物 7 ***************
            // object_s = 330;
            // object_d = -3.5;
            // object_id += 1; // 106
            // object_label = "Car" + std::to_string(object_id);
            // this->sensor_fusion_object_pack(object_s, object_d, object_length, object_width, object_height,
            //                                 object_heading, object_v_X, object_v_Y, object_v_yaw_rate,
            //                                 object_label, object_id,
            //                                 object_line_sacle, object_label_scale, object_color_r, object_color_g, object_color_b, object_color_a,
            //                                 sensor_fusion_results_bounding_box_msg, sensor_fusion_results_label_msg,
            //                                 object_frame_id);
            // // *************** 模拟目标物 8 ***************
            // if (this->now().seconds() - run_start_time_stamp < 50)
            // {
            //     object_s = 330;
            //     object_d = 0;
            //     object_id += 1; // 107
            //     object_label = "Car" + std::to_string(object_id);
            //     this->sensor_fusion_object_pack(object_s, object_d, object_length, object_width, object_height,
            //                                     240, object_v_X, object_v_Y, object_v_yaw_rate,
            //                                     object_label, object_id,
            //                                     object_line_sacle, object_label_scale, object_color_r, object_color_g, object_color_b, object_color_a,
            //                                     sensor_fusion_results_bounding_box_msg, sensor_fusion_results_label_msg,
            //                                     object_frame_id);
            // }
            // // *************** 模拟目标物 9 ***************
            // object_s = 330;
            // object_d = 3.5;
            // object_id += 1; // 108
            // object_label = "Car" + std::to_string(object_id);
            // this->sensor_fusion_object_pack(object_s, object_d, object_length, object_width, object_height,
            //                                 object_heading, object_v_X, object_v_Y, object_v_yaw_rate,
            //                                 object_label, object_id,
            //                                 object_line_sacle, object_label_scale, object_color_r, object_color_g, object_color_b, object_color_a,
            //                                 sensor_fusion_results_bounding_box_msg, sensor_fusion_results_label_msg,
            //                                 object_frame_id);

            // // *************** 模拟目标物 10 ***************
            // object_s = 330;
            // object_d = 7;
            // object_id += 1; // 109
            // object_label = "Car" + std::to_string(object_id);
            // this->sensor_fusion_object_pack(object_s, object_d, object_length, object_width, object_height,
            //                                 object_heading, object_v_X, object_v_Y, object_v_yaw_rate,
            //                                 object_label, object_id,
            //                                 object_line_sacle, object_label_scale, object_color_r, object_color_g, object_color_b, object_color_a,
            //                                 sensor_fusion_results_bounding_box_msg, sensor_fusion_results_label_msg,
            //                                 object_frame_id);
            // // *************** 模拟目标物 11 ***************
            // object_s = 330;
            // object_d = -7;
            // object_id += 1; // 110
            // object_label = "Car" + std::to_string(object_id);
            // this->sensor_fusion_object_pack(object_s, object_d, object_length, object_width, object_height,
            //                                 object_heading, object_v_X, object_v_Y, object_v_yaw_rate,
            //                                 object_label, object_id,
            //                                 object_line_sacle, object_label_scale, object_color_r, object_color_g, object_color_b, object_color_a,
            //                                 sensor_fusion_results_bounding_box_msg, sensor_fusion_results_label_msg,
            //                                 object_frame_id);

            // // *************** 模拟目标物 7 ***************
            // object_s = 480;
            // object_d = -3.5;
            // object_id += 1; // 111
            // object_label = "Car" + std::to_string(object_id);
            // this->sensor_fusion_object_pack(object_s, object_d, object_length, object_width, object_height,
            //                                 object_heading, object_v_X, object_v_Y, object_v_yaw_rate,
            //                                 object_label, object_id,
            //                                 object_line_sacle, object_label_scale, object_color_r, object_color_g, object_color_b, object_color_a,
            //                                 sensor_fusion_results_bounding_box_msg, sensor_fusion_results_label_msg,
            //                                 object_frame_id);
            // // // *************** 模拟目标物 8 ***************
            // // object_s = 480;
            // // object_d = 0;
            // // object_id += 1;
            // // object_label = "Car" + std::to_string(object_id);
            // // this->sensor_fusion_object_pack(object_s, object_d, object_length, object_width, object_height,
            // //                                 object_heading, object_v_X, object_v_Y, object_v_yaw_rate,
            // //                                 object_label, object_id,
            // //                                 object_line_sacle, object_label_scale, object_color_r, object_color_g, object_color_b, object_color_a,
            // //                                 sensor_fusion_results_bounding_box_msg, sensor_fusion_results_label_msg,
            // //                                 object_frame_id);
            // // *************** 模拟目标物 9 ***************
            // object_s = 480;
            // object_d = 3.5;
            // object_id += 1;  // 112
            // object_label = "Car" + std::to_string(object_id);
            // this->sensor_fusion_object_pack(object_s, object_d, object_length, object_width, object_height,
            //                                 object_heading, object_v_X, object_v_Y, object_v_yaw_rate,
            //                                 object_label, object_id,
            //                                 object_line_sacle, object_label_scale, object_color_r, object_color_g, object_color_b, object_color_a,
            //                                 sensor_fusion_results_bounding_box_msg, sensor_fusion_results_label_msg,
            //                                 object_frame_id);

            // // *************** 模拟目标物 10 ***************
            // object_s = 480;
            // object_d = 7;
            // object_id += 1; // 113
            // object_label = "Car" + std::to_string(object_id);
            // this->sensor_fusion_object_pack(object_s, object_d, object_length, object_width, object_height,
            //                                 object_heading, object_v_X, object_v_Y, object_v_yaw_rate,
            //                                 object_label, object_id,
            //                                 object_line_sacle, object_label_scale, object_color_r, object_color_g, object_color_b, object_color_a,
            //                                 sensor_fusion_results_bounding_box_msg, sensor_fusion_results_label_msg,
            //                                 object_frame_id);
            // // *************** 模拟目标物 11 ***************
            // object_s = 480;
            // object_d = -7;
            // object_id += 1;  // 114
            // object_label = "Car" + std::to_string(object_id);
            // this->sensor_fusion_object_pack(object_s, object_d, object_length, object_width, object_height,
            //                                 object_heading, object_v_X, object_v_Y, object_v_yaw_rate,
            //                                 object_label, object_id,
            //                                 object_line_sacle, object_label_scale, object_color_r, object_color_g, object_color_b, object_color_a,
            //                                 sensor_fusion_results_bounding_box_msg, sensor_fusion_results_label_msg,
            //                                 object_frame_id);

            // // *************** 模拟目标物 8 ***************
            // // object_s = 530;
            // // object_d = 0;
            // // object_id += 1;
            // // object_label = "Car" + std::to_string(object_id);
            // // object_heading = 258;
            // // this->sensor_fusion_object_pack(object_s, object_d, object_length, object_width, object_height,
            // //                                 object_heading, object_v_X, object_v_Y, object_v_yaw_rate,
            // //                                 object_label, object_id,
            // //                                 object_line_sacle, object_label_scale, object_color_r, object_color_g, object_color_b, object_color_a,
            // //                                 sensor_fusion_results_bounding_box_msg, sensor_fusion_results_label_msg,
            // //                                 object_frame_id);

            // // object_s = 530;
            // // object_d = -3.5;
            // // object_id += 1;
            // // object_label = "Car" + std::to_string(object_id);
            // // object_heading = 258;
            // // this->sensor_fusion_object_pack(object_s, object_d, object_length, object_width, object_height,
            // //                                 object_heading, object_v_X, object_v_Y, object_v_yaw_rate,
            // //                                 object_label, object_id,
            // //                                 object_line_sacle, object_label_scale, object_color_r, object_color_g, object_color_b, object_color_a,
            // //                                 sensor_fusion_results_bounding_box_msg, sensor_fusion_results_label_msg,
            // //                                 object_frame_id);