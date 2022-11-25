#ifndef HELPERS_H
#define HELPERS_H

#include <string>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/QR>
#include <vector>
#include <geometry_msgs/msg/point.hpp>
#include "coordinate_transform.h"

using std::vector;
using Eigen::VectorXd;
using std::string;

//
// Helper functions to fit and evaluate polynomials.
//

// Evaluate a polynomial.
double polyeval(const VectorXd &coeffs, double x)
{
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); ++i)
  {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from:
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
VectorXd polyfit(const VectorXd &xvals, const VectorXd &yvals, int order)
{
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);

  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); ++i)
  {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); ++j)
  {
    for (int i = 0; i < order; ++i)
    {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);

  return result;
}

// 航向要求是弧度单位,
// 输入为前方车辆屁股的地面中点的坐标和车辆的长宽高与航向
vector<geometry_msgs::msg::Point> get_bounding_box_label_cartesian(double px, double py, double length, double width, double height, double theta)
{
    vector<geometry_msgs::msg::Point> points;

    double d = width / 2;

    geometry_msgs::msg::Point point_of_bounding_box_1;
    geometry_msgs::msg::Point point_of_bounding_box_2;
    geometry_msgs::msg::Point point_of_bounding_box_3;
    geometry_msgs::msg::Point point_of_bounding_box_4;
    geometry_msgs::msg::Point point_of_bounding_box_5;
    geometry_msgs::msg::Point point_of_bounding_box_6;
    geometry_msgs::msg::Point point_of_bounding_box_7;
    geometry_msgs::msg::Point point_of_bounding_box_8;

    geometry_msgs::msg::Point point_of_label;

    point_of_bounding_box_1.x = px - d * sin(theta);  
    point_of_bounding_box_1.y = py + d * cos(theta);  
    point_of_bounding_box_1.z = 0;

    point_of_bounding_box_2.x = px + d * sin(theta);  
    point_of_bounding_box_2.y = py - d * cos(theta);  
    point_of_bounding_box_2.z = 0;

    point_of_bounding_box_3.x = px + d * sin(theta) + length * cos(theta);  
    point_of_bounding_box_3.y = py - d * cos(theta) + length * sin(theta);  
    point_of_bounding_box_3.z = 0;

    point_of_bounding_box_4.x = px - d * sin(theta) + length * cos(theta);  
    point_of_bounding_box_4.y = py + d * cos(theta) + length * sin(theta);  
    point_of_bounding_box_4.z = 0;

    point_of_bounding_box_5.x = px - d * sin(theta);  
    point_of_bounding_box_5.y = py + d * cos(theta);  
    point_of_bounding_box_5.z = height;

    point_of_bounding_box_6.x = px + d * sin(theta);  
    point_of_bounding_box_6.y = py - d * cos(theta);  
    point_of_bounding_box_6.z = height;

    point_of_bounding_box_7.x = px + d * sin(theta) + length * cos(theta);  
    point_of_bounding_box_7.y = py - d * cos(theta) + length * sin(theta);  
    point_of_bounding_box_7.z = height;

    point_of_bounding_box_8.x = px - d * sin(theta) + length * cos(theta);  
    point_of_bounding_box_8.y = py + d * cos(theta) + length * sin(theta);  
    point_of_bounding_box_8.z = height;

    // 包围框中间
    point_of_label.x = (point_of_bounding_box_1.x + point_of_bounding_box_3.x)/2;
    point_of_label.y = (point_of_bounding_box_2.y + point_of_bounding_box_4.y)/2;
    point_of_label.z = height;

    points.push_back(point_of_bounding_box_1);
    points.push_back(point_of_bounding_box_2);

    points.push_back(point_of_bounding_box_2);
    points.push_back(point_of_bounding_box_3);

    points.push_back(point_of_bounding_box_3);
    points.push_back(point_of_bounding_box_4);

    points.push_back(point_of_bounding_box_4);
    points.push_back(point_of_bounding_box_1);

    points.push_back(point_of_bounding_box_5);
    points.push_back(point_of_bounding_box_6);

    points.push_back(point_of_bounding_box_6);
    points.push_back(point_of_bounding_box_7);

    points.push_back(point_of_bounding_box_7);
    points.push_back(point_of_bounding_box_8);

    points.push_back(point_of_bounding_box_8);
    points.push_back(point_of_bounding_box_5);

    points.push_back(point_of_bounding_box_1);
    points.push_back(point_of_bounding_box_5);

    points.push_back(point_of_bounding_box_2);
    points.push_back(point_of_bounding_box_6);

    points.push_back(point_of_bounding_box_3);
    points.push_back(point_of_bounding_box_7);

    points.push_back(point_of_bounding_box_4);
    points.push_back(point_of_bounding_box_8);

    points.push_back(point_of_label);
    
    return points;
}

vector<geometry_msgs::msg::Point> get_bounding_box_label_frenet(double s, double d, double length, double width, double height, double theta, 
                                                        const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
    vector<geometry_msgs::msg::Point> points;

    vector<double> px_py = frenet_to_cartesian(s, d, maps_s, maps_x, maps_y);

    double px = px_py[0];
    double py = px_py[1];

    points = get_bounding_box_label_cartesian(px, py, length, width, height, theta);
    
    return points;
}

#endif // HELPERS_H