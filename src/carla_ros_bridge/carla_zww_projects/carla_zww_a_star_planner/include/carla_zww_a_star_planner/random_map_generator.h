#ifndef _RANDOM_MAP_GENERATOR_H_
#define _RANDOM_MAP_GENERATOR_H_

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <math.h>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl_conversions/pcl_conversions.h>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logging.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <Eigen/Eigen>
#include <iostream>
#include <pcl/search/impl/kdtree.hpp>
#include <random>

using namespace std;
using namespace Eigen;

class RandomMapGenerator : public rclcpp::Node{
    public:
        RandomMapGenerator();
        ~RandomMapGenerator();
    
    public:
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _all_map_pub;
        sensor_msgs::msg::PointCloud2 globalMap_pcd;

        void RandomMapGenerate();

        void pubSensedPoints();
        
        pcl::PointCloud<pcl::PointXYZ> cloudMap;

        // pcl::search::KdTree<pcl::PointXYZ> kdtreeMap;
        vector<int> pointIdxSearch;
        vector<float> pointSquaredDistance;

        int _obs_num, _cir_num;
        double _x_size, _y_size, _z_size, _init_x, _init_y, _resolution, _sense_rate;
        double _x_l, _x_h, _y_l, _y_h, _w_l, _w_h, _h_l, _h_h, _w_c_l, _w_c_h;

        bool _has_map = false;

        rclcpp::TimerBase::SharedPtr global_map_publish_timer;

};


#endif