#include "visualization/global_path_visualization.h"

/*'''**************************************************************************************
- FunctionName: None
- Function    : 定时广播全局路径出去
- Inputs      : None
- Outputs     : None
- Comments    : None
**************************************************************************************'''*/
GlobalPathVisualization::GlobalPathVisualization() : Node("goobal_path_visualization") {
    RCLCPP_INFO(this->get_logger(), "publishing global path.");
    global_path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("global_path", 10);

    target_velocity_from_csv_publisher = this->create_publisher<std_msgs::msg::Float32>("velocity_target_from_csv", 10);

    global_path_publisher_timer_ = this->create_wall_timer(1000ms, std::bind(&GlobalPathVisualization::global_path_publisher_timer_callback, this));

    target_velocity_from_csv_timer = this->create_wall_timer(25ms, std::bind(&GlobalPathVisualization::target_velocity_publisher_callback, this));

    global_path_multi_lines_publisher = this->create_publisher<visualization_msgs::msg::MarkerArray>("global_path_multi_lines", 10);

    global_path_multi_lines_publisher_timer_ = this->create_wall_timer(2000ms, std::bind(&GlobalPathVisualization::global_path_multi_lines_publisher_timer_callback, this));

    this->declare_parameter<string>("global_map_name_parameter", path_configure_parameter);
    this->declare_parameter<string>("velocity_curve_name_parameter", velocity_wyx_curve);
}

/*'''**************************************************************************************
- FunctionName: None
- Function    : None
- Inputs      : None
- Outputs     : None
- Comments    : None
**************************************************************************************'''*/
GlobalPathVisualization::~GlobalPathVisualization() {}

/*'''**************************************************************************************
- FunctionName: None
- Function    : None
- Inputs      : None
- Outputs     : None
- Comments    : None
**************************************************************************************'''*/
void GlobalPathVisualization::global_path_publisher_timer_callback() {
    if (this->load_map_done_global)
    {
        RCLCPP_INFO(this->get_logger(), "publishing global path");

        global_path_offset.header.stamp = this->get_clock()->now();
        global_path_offset.header.frame_id = "odom";
        
        global_path_publisher_->publish(global_path_offset);
    }
}

/*'''**************************************************************************************
- FunctionName: None
- Function    : None
- Inputs      : None
- Outputs     : None
- Comments    : None
**************************************************************************************'''*/
void GlobalPathVisualization::global_path_multi_lines_publisher_timer_callback(){
    if (this->load_map_done_global)
    {
        // global_path_multi_line_single.header.stamp = this->get_clock()->now();
        // global_path_multi_lines.markers.clear();
        // global_path_multi_lines.markers.push_back(global_path_multi_line_single);
        
        global_path_multi_lines_publisher->publish(global_path_multi_lines);
    }
}

/*'''**************************************************************************************
- FunctionName: None
- Function    : None
- Inputs      : None
- Outputs     : None
- Comments    : None
**************************************************************************************'''*/
void GlobalPathVisualization::target_velocity_publisher_callback()
{
    if (this->load_velocity_curve_done)
    {
        RCLCPP_INFO(this->get_logger(), "publishing target velocity");

        velocity_target_from_csv.data = velocity_curve[i];

        if (i > velocity_curve.size()-2){
            return;
        }
        i++;

        // global_path.header.stamp = this->get_clock()->now();
        // global_path.header.frame_id = "odom";
        
        // global_path_publisher_->publish(global_path);
        target_velocity_from_csv_publisher->publish(velocity_target_from_csv);
    }
}

/*'''**************************************************************************************
- FunctionName: None
- Function    : 模板函数：将string类型变量转换为常用的数值类型（此方法具有普遍适用性）
- Inputs      : None
- Outputs     : None
- Comments    : None
**************************************************************************************'''*/
template <class Type>
Type GlobalPathVisualization::stringToNum(const string &str)
{
    std::istringstream iss(str);
    Type num;
    iss >> num;
    return num;
}
/*'''**************************************************************************************
- FunctionName: None
- Function    : 模板函数
- Inputs      : None
- Outputs     : None
- Comments    : None
**************************************************************************************'''*/
template <class Type>
vector<Type> GlobalPathVisualization::string_split(const string &str, const char &pattern)
{
    vector<Type> res;
    if (str == "")
    {
        return res;
    }
    string strs = str + pattern; // 在字符串末尾加入分割符号，方便截取最后一段。
    size_t pos = strs.find(pattern);
    while (pos != strs.npos)
    {
        string temp_string = strs.substr(0, pos);
        Type temp_number = stringToNum<Type>(temp_string);
        res.push_back(temp_number);
        strs = strs.substr(pos + 1, strs.size()); // 去掉已分割的字符串,在剩下的字符串中进行分割
        pos = strs.find(pattern);
    }
    return res;
}
/*'''**************************************************************************************
- FunctionName: None
- Function    : None
- Inputs      : None
- Outputs     : None
- Comments    : None
**************************************************************************************'''*/
void GlobalPathVisualization::load_map()
{
    char *buffer;
    if ((buffer = getcwd(NULL, 0)) == NULL) {
        perror("getcwd error");
    }
    else {
        string buffer_ = buffer;
        string local_path;
        this->get_parameter<string>("global_map_name_parameter", local_path);
        global_path_path = buffer_ + local_path;
        cout << "global_path_path" << global_path_path << endl;
    }

    ifstream infile(global_path_path);
    string value;
    int i = 0;
    getline(infile, value); // 舍弃头
    while (infile.good()) {
        cout << "加载全局地图" << endl;
        getline(infile, value);
        if (value != "")
        {
            cout << "string value : " << value << endl;
            cout.precision(12);
            vector<double> temp_values = string_split<double>(value, ',');

            // this_pose_stamped.header.frame_id = "odom";             
            // this_pose_stamped.header.stamp = this->get_clock()->now();
            // this_pose_stamped.pose.position.x = temp_values[2];
            // this_pose_stamped.pose.position.y = temp_values[3];
            // this_pose_stamped.pose.position.z = 0;
            // this_pose_stamped.pose.orientation.x = 0;
            // this_pose_stamped.pose.orientation.y = 0;
            // this_pose_stamped.pose.orientation.z = 0;
            // this_pose_stamped.pose.orientation.w = temp_values[7]; // 这里实际上是放的frenet坐标系的S
            
            // global_path.poses.push_back(this_pose_stamped);
            
            global_path_psi.push_back(temp_values[1]);
            global_path_x.push_back(temp_values[2]);
            global_path_y.push_back(temp_values[3]);
            global_path_s.push_back(temp_values[7]);

            

            // if (i % 8 == 0){
            //     global_path_x_down_sample.push_back(temp_values[2]); // ptsx
            //     global_path_y_down_sample.push_back(temp_values[3]); // ptsy
            //     global_path_psi_down_sample.push_back(temp_values[1]); // ptsy
            //     global_path_s_down_sample.push_back(temp_values[7]);
            // }
            i++;
        }
    }
    for (int i = 1; i < global_path_x.size()-20; i++){
        vector<double> line_s_d = cartesian_to_frenet(global_path_x[i], global_path_y[i], global_path_psi[i]/57.29578, global_path_x, global_path_y);
        // vector<double> line_x_y = frenet_to_cartesian(line_s_d[0], line_s_d[1] + 4/2.0, global_path_s, global_path_x, global_path_y); 
        vector<double> line_x_y = frenet_to_cartesian(line_s_d[0], line_s_d[1] , global_path_s, global_path_x, global_path_y); 
        cout << "line_x_y~~~~~~~~~~~~~~~~~~~~~~~~~~: " << line_x_y[0] << ", " << line_x_y[1] << endl;

        this_pose_stamped.header.frame_id = "odom";             
        this_pose_stamped.header.stamp = this->get_clock()->now();
        this_pose_stamped.pose.position.x = line_x_y[0];
        this_pose_stamped.pose.position.y = line_x_y[1];
        this_pose_stamped.pose.position.z = 0;
        this_pose_stamped.pose.orientation.x = 0;
        this_pose_stamped.pose.orientation.y = 0;
        this_pose_stamped.pose.orientation.z = 0;
        this_pose_stamped.pose.orientation.w = global_path_s[i]; // 这里实际上是放的frenet坐标系的S
        
        global_path_offset.poses.push_back(this_pose_stamped);

        if (i % 8 == 0 || i == 1){
            global_path_x_down_sample.push_back(line_x_y[0]); // ptsx
            global_path_y_down_sample.push_back(line_x_y[1]); // ptsy
            global_path_psi_down_sample.push_back(global_path_psi[i]); // ptsy
            global_path_s_down_sample.push_back(global_path_s[i]);
        }
    }

    GlobalPathVisualization::generate_road_structure(-2, 323, 0);
    GlobalPathVisualization::generate_road_structure(-2 - 4, 324, 1);
    // GlobalPathVisualization::generate_road_structure(-1.75 - 3.5 * 2, 325, 1);

    GlobalPathVisualization::generate_road_structure(2, 334, 1);
    // GlobalPathVisualization::generate_road_structure(1.75 + 3.5, 335, 1);
    // GlobalPathVisualization::generate_road_structure(1.75 + 3.5 * 2, 336, 1);
    load_map_done_global = true;
}

/*'''**************************************************************************************
- FunctionName: None
- Function    : None
- Inputs      : None
- Outputs     : None
- Comments    : None
**************************************************************************************'''*/
void GlobalPathVisualization::generate_road_structure(double lane_d_coordinate, int lane_id, int line_type_flag){
    visualization_msgs::msg::Marker global_path_multi_line_single;
    global_path_multi_line_single.id = lane_id;
    global_path_multi_line_single.header.stamp = this->get_clock()->now();
    global_path_multi_line_single.header.frame_id = "odom";
    if (line_type_flag == 0){
        global_path_multi_line_single.type = visualization_msgs::msg::Marker::LINE_LIST;
    }
    else{
        global_path_multi_line_single.type = visualization_msgs::msg::Marker::LINE_STRIP;
    }
    global_path_multi_line_single.action = visualization_msgs::msg::Marker::ADD;
    // global_path_multi_line_single.lifetime = rclcpp::Duration(0s); // 显示所有的
    global_path_multi_line_single.lifetime = rclcpp::Duration(2s);
    global_path_multi_line_single.scale.x = 0.2;
    global_path_multi_line_single.scale.y = 0.2;
    global_path_multi_line_single.scale.z = 0.2;
    global_path_multi_line_single.color.r = 1;
    global_path_multi_line_single.color.g = 1;
    global_path_multi_line_single.color.b = 1;
    global_path_multi_line_single.color.a = 1;

    global_path_multi_line_single.points.clear();
    geometry_msgs::msg::Point global_path_multi_line_single_single_point;
    for (int i = 0; i < global_path_x_down_sample.size(); i++){
        vector<double> line_s_d = cartesian_to_frenet(global_path_x_down_sample[i], global_path_y_down_sample[i], global_path_psi_down_sample[i]/57.29578, global_path_x, global_path_y);
        vector<double> line_x_y = frenet_to_cartesian(line_s_d[0], line_s_d[1] + lane_d_coordinate, global_path_s, global_path_x, global_path_y); 

        global_path_multi_line_single_single_point.x = line_x_y[0];
        global_path_multi_line_single_single_point.y = line_x_y[1];
        global_path_multi_line_single_single_point.z = 0;

        if (global_path_x_down_sample.size() % 2 == 0) {
            if (i >= 2) {
                global_path_multi_line_single.points.push_back(global_path_multi_line_single_single_point);
            }
        }
        else{
            if (i >= 1) {
                global_path_multi_line_single.points.push_back(global_path_multi_line_single_single_point);
            }
        }
    }
    global_path_multi_lines.markers.push_back(global_path_multi_line_single);
}

/*'''**************************************************************************************
- FunctionName: None
- Function    : None
- Inputs      : None
- Outputs     : None
- Comments    : None
**************************************************************************************'''*/
void GlobalPathVisualization::load_velocity_curve()
{
    char *buffer;
    if ((buffer = getcwd(NULL, 0)) == NULL) {
        perror("getcwd error");
    }
    else {
        string buffer_ = buffer;
        string local_path;
        this->get_parameter<string>("velocity_curve_name_parameter", local_path);

        global_path_path = buffer_ + local_path;
        cout << "global_path_path" << global_path_path << endl;
    }

    ifstream infile(global_path_path);
    string value;
    int i = 0;
    getline(infile, value); // 舍弃头
    while (infile.good()) {
        // cout << "load velocity curve" << endl;
        getline(infile, value);
        if (value != "")
        {
            cout << "string value : " << value << endl;
            cout.precision(12);
            vector<double> temp_values = string_split<double>(value, ',');

            velocity_curve.push_back(temp_values[0]);

            i++;
            if (i == 10)
            {
                // break;
            }
        }
    }
    load_velocity_curve_done = true;

    for(int i = 0; i < velocity_curve.size(); i++){
        cout << velocity_curve[i] << endl;
    }
}

/*'''*****************************************************************************************************
# Function Name  : 
# FileFunction   : 
# Comments       :
*****************************************************************************************************'''*/
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto n = std::make_shared<GlobalPathVisualization>();

    RCLCPP_INFO(n->get_logger(), "running");

    n->load_map();
    n->load_velocity_curve();

    rclcpp::spin(n);
    rclcpp::shutdown();

    return 0;
}
