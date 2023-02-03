#include "carla_zww_a_star_planner/a_star_planner.h"

// using namespace std;
using std::placeholders::_1;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("a_star_planner");

AStarPlannerNode::AStarPlannerNode()
    : Node("a_star_planner")
/*'''**************************************************************************************
- FunctionName: None
- Function    : None
- Inputs      : None
- Outputs     : None
- Comments    : None
**************************************************************************************'''*/
{
    RCLCPP_INFO(LOGGER, " ~~~~~~~~~~~~~ a_star_node init finish ~~~~~~~~~~~~~ ");


    _map_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>("/global_map", 10, std::bind(&AStarPlannerNode::rcvPointCloudCallBack, this, _1));
    _pts_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose", 10, std::bind(&AStarPlannerNode::rcvWaypointsCallback, this, _1));

    _grid_map_vis_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("grid_map_vis", 10);
    _grid_path_vis_pub = this->create_publisher<visualization_msgs::msg::Marker>("grid_path_vis", 10);
    _visited_nodes_vis_pub = this->create_publisher<visualization_msgs::msg::Marker>("visited_nodes_vis", 10);

    _cloud_margin = 0.0;
    _resolution = 1.0;
    _x_size = 100.0;
    _y_size = 100.0;
    _z_size = 5.0;
    _start_pt(0) = -18.0;
    _start_pt(1) = -14.0;
    _start_pt(2) = 0.0;

    _map_lower << -_x_size / 2.0, -_y_size / 2.0, 0.0;
    _map_upper << +_x_size / 2.0, +_y_size / 2.0, _z_size;

    _inv_resolution = 1.0 / _resolution;

    _max_x_id = (int)(_x_size * _inv_resolution);
    _max_y_id = (int)(_y_size * _inv_resolution);
    _max_z_id = (int)(_z_size * _inv_resolution);

    _astar_path_finder = new AstarPathFinder();
    _astar_path_finder->initGridMap(_resolution, _map_lower, _map_upper, _max_x_id, _max_y_id, _max_z_id);
}

AStarPlannerNode::~AStarPlannerNode()
/*'''**************************************************************************************
- FunctionName: None
- Function    : None
- Inputs      : None
- Outputs     : None
- Comments    : None
**************************************************************************************'''*/
{
    delete _astar_path_finder;
}

// 订阅到终点信息的回调函数
void AStarPlannerNode::rcvWaypointsCallback(geometry_msgs::msg::PoseStamped::SharedPtr wp) {
    // RCLCPP_INFO(LOGGER, "[node] receive the planning target~~~~~~~~~~~~~~~~~~~~c");

    if (wp->pose.position.z < 0.0 || _has_map == false) return;
    // if (wp->pose.position.z < 0.0) return;

    // 获取交互式界面给出的终点坐标
    Vector3d target_pt;
    target_pt << wp->pose.position.x, wp->pose.position.y,
        0.0;    // wp.poses[0].pose.position.z;

    RCLCPP_INFO(LOGGER, "[node] receive the planning target~~~~~~~~~~~~~~~~~~~~");
    // std::cout << wp->pose.position.x << ", " << wp->pose.position.y << ", " << 0 << std::endl;
    // std::cout << _start_pt << std::endl;
    // std::cout << target_pt << std::endl;
    // 输入起点、终点，调用 pathFind 函数
    pathFinding(_start_pt, target_pt);
}

// 订阅到地图信息的回调函数
void AStarPlannerNode::rcvPointCloudCallBack(sensor_msgs::msg::PointCloud2::SharedPtr pointcloud_map) {
    if (_has_map && received_global_map_count > 4) return;

    RCLCPP_INFO(LOGGER, "[node] receive the global map!!!!!!!!!!!!!c");

    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PointCloud<pcl::PointXYZ> cloud_vis;
    sensor_msgs::msg::PointCloud2 map_vis;

    pcl::fromROSMsg(*pointcloud_map, cloud);

    if ((int)cloud.points.size() == 0) return;

    pcl::PointXYZ pt;
    for (int idx = 0; idx < (int)cloud.points.size(); idx++) {
        pt = cloud.points[idx];

        // set obstalces into grid map for path planning
        // 将障碍物信息设置进入栅格化地图，为后续路径规划做准备
        _astar_path_finder->setObs(pt.x, pt.y, pt.z);

        // 可视化地图部分
        Vector3d cor_round = _astar_path_finder->coordRounding(Vector3d(pt.x, pt.y, pt.z));
        pt.x = cor_round(0);
        pt.y = cor_round(1);
        pt.z = cor_round(2);
        cloud_vis.points.push_back(pt);
    }

    cloud_vis.width = cloud_vis.points.size();
    cloud_vis.height = 1;
    cloud_vis.is_dense = true;

    pcl::toROSMsg(cloud_vis, map_vis);

    map_vis.header.frame_id = "world";
    _grid_map_vis_pub->publish(map_vis);

    received_global_map_count++;

    _has_map = true;
}

void AStarPlannerNode::pathFinding(Vector3d start_pt, const Vector3d target_pt) {
    // Call A* to search for a path
    _astar_path_finder->AstarGraphSearch(start_pt, target_pt);
    // Retrieve the path
    auto grid_path = _astar_path_finder->getPath();
    auto visited_nodes = _astar_path_finder->getVisitedNodes();

    // Visualize the result
    visGridPath(grid_path, false);
    visVisitedNode(visited_nodes);

    // Reset map for next call
    _astar_path_finder->resetUsedGrids();
}

void AStarPlannerNode::visGridPath(vector<Vector3d> nodes, bool is_use_jps) {
    visualization_msgs::msg::Marker node_vis;
    node_vis.header.frame_id = "world";
    node_vis.header.stamp = this->now();

    node_vis.ns = "demo_node/astar_path";

    node_vis.type = visualization_msgs::msg::Marker::CUBE_LIST;
    node_vis.action = visualization_msgs::msg::Marker::ADD;
    node_vis.id = 0;

    node_vis.pose.orientation.x = 0.0;
    node_vis.pose.orientation.y = 0.0;
    node_vis.pose.orientation.z = 0.0;
    node_vis.pose.orientation.w = 1.0;

    node_vis.color.a = 1.0;
    node_vis.color.r = 1.0;
    node_vis.color.g = 0.0;
    node_vis.color.b = 0.0;

    node_vis.scale.x = _resolution;
    node_vis.scale.y = _resolution;
    node_vis.scale.z = _resolution;

    geometry_msgs::msg::Point pt;
    for (int i = 0; i < int(nodes.size()); i++) {
        Vector3d coord = nodes[i];
        pt.x = coord(0);
        pt.y = coord(1);
        pt.z = coord(2);

        node_vis.points.push_back(pt);
    }

    _grid_path_vis_pub->publish(node_vis);
}

void AStarPlannerNode::visVisitedNode(vector<Vector3d> nodes) {
    visualization_msgs::msg::Marker node_vis;
    node_vis.header.frame_id = "world";
    node_vis.header.stamp = this->now();
    node_vis.ns = "demo_node/expanded_nodes";
    node_vis.type = visualization_msgs::msg::Marker::CUBE_LIST;
    node_vis.action = visualization_msgs::msg::Marker::ADD;
    node_vis.id = 0;

    node_vis.pose.orientation.x = 0.0;
    node_vis.pose.orientation.y = 0.0;
    node_vis.pose.orientation.z = 0.0;
    node_vis.pose.orientation.w = 1.0;
    node_vis.color.a = 0.5;
    node_vis.color.r = 0.0;
    node_vis.color.g = 0.0;
    node_vis.color.b = 1.0;

    node_vis.scale.x = _resolution;
    node_vis.scale.y = _resolution;
    node_vis.scale.z = _resolution;

    geometry_msgs::msg::Point pt;
    for (int i = 0; i < int(nodes.size()); i++) {
        Vector3d coord = nodes[i];
        pt.x = coord(0);
        pt.y = coord(1);
        pt.z = coord(2);

        node_vis.points.push_back(pt);
    }

    _visited_nodes_vis_pub->publish(node_vis);
}

int main(int argc, char** argv) {
    RCLCPP_INFO(LOGGER, "Initializa Node~");
    std::cout << argv[0] << std::endl;
    rclcpp::init(argc, argv);
    auto n = std::make_shared<AStarPlannerNode>();
    rclcpp::spin(n);
    rclcpp::shutdown();

    return 0;
}