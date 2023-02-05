#ifndef UTILS_H
#define UTILS_H

#include <math.h>
#include <string>
#include <vector>
#include <limits>
#include <string>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/QR>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// for convenience
using Eigen::VectorXd;
using std::string;
using std::vector;

const double lane_width		= 3.5;		// width of a lane					(m)
double safety_margin	= 15.0;		// distance to keep from other cars	(m)
double max_safe_speed	= 8;		// max reference speed in the limit	m/s

/* *****************************************************************************************************************
- FunctionName: 
- Function    : 
- Inputs      : 
- Outputs     : 
- Comments    : 对于修饰Object来说，const并未区分出编译期常量和运行期常量, constexpr限定在了编译期常量 
***************************************************************************************************************** */
constexpr double pi() { return M_PI; }

double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

/* *****************************************************************************************************************
- FunctionName: 
- Function    : Calculate distance between two points
- Inputs      : 
- Outputs     : 
- Comments    : 
***************************************************************************************************************** */
double distance_two_point(double x1, double y1, double x2, double y2)
{
    return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

/* *****************************************************************************************************************
- FunctionName: 
- Function    : Check if a vehicle is is a certain lane
- Inputs      : 
- Outputs     : 
- Comments    : #TODO: 可修改项目
***************************************************************************************************************** */
bool is_in_lane(double d, int lane)
{
    return (d > lane_width * lane) && (d < lane_width * lane + lane_width);
}

// 确定当前车辆位于哪个车道，默认为5车道道路，假设车辆不会冲出道路
int which_lane(double d)
{
    if (d >= -lane_width * 0.5 && d <= lane_width * 0.5)
    {
        return 0;
    }
    else if (d > lane_width * 0.5 && d <= lane_width * 1.5)
    {
        return 1;
    }
    else if (d > lane_width * 1.5 && d <= lane_width * 2.5)
    {
        return 2;
    }
    else if (d >= -lane_width * 1.5 && d < -lane_width * 0.5)
    {
        return -1;
    }
    else if (d >= -lane_width * 2.5 && d < -lane_width * 1.5)
    {
        return -2;
    }
    return 1000;
}

bool is_same_lane(double host_d, double object_d)
{
    return which_lane(host_d) == which_lane(object_d);
}

int get_closest_waypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{
    double closest_len = std::numeric_limits<int>::max();
    int closest_waypoint = 0;

    for (uint i = 0; i < maps_x.size(); ++i)
    {
        double map_x = maps_x[i];
        double map_y = maps_y[i];
        double dist = distance_two_point(x, y, map_x, map_y);
        if (dist < closest_len)
        {
            closest_len = dist;
            closest_waypoint = i;
        }
    }
    return closest_waypoint;
}

/* *****************************************************************************************************************
- FunctionName: 
- Function    : Choose in the map of highway waypoints the closest before the car (that is the next).
                The actual closest waypoint could be behind the car.
- Inputs      : 
- Outputs     : 
- Comments    : 先找到距离最近的点，再根据车辆的航向与车辆当前位置点和最近点的连线方向形成的夹角的范围确定是不是前方的最近的一个，如果不是，就将索引加1
***************************************************************************************************************** */
int get_next_waypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
    uint closest_waypoint = get_closest_waypoint(x, y, maps_x, maps_y);

    double map_x = maps_x[closest_waypoint];
    double map_y = maps_y[closest_waypoint];

    double heading = atan2((map_y - y), (map_x - x));

    double angle = fabs(theta - heading);
    angle = std::min(2 * pi() - angle, angle);

    if (angle > pi() / 2)
    {
        ++closest_waypoint;
        if (closest_waypoint == maps_x.size())
        {
            closest_waypoint = 0;
        }
    }

    return closest_waypoint;
}

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

/* *****************************************************************************************************************
- FunctionName: 
- Function    : Transform from Cartesian x,y coordinates to Frenet s,d coordinates
- Inputs      : 
- Outputs     : 
- Comments    : 
***************************************************************************************************************** */
vector<double> cartesian_to_frenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
    int next_wp = get_next_waypoint(x, y, theta, maps_x, maps_y);

    int prev_wp;
    prev_wp = next_wp - 1;
    if (next_wp == 0)
    {
        prev_wp = maps_x.size() - 1;
    }

    double n_x = maps_x[next_wp] - maps_x[prev_wp];
    double n_y = maps_y[next_wp] - maps_y[prev_wp];
    double x_x = x - maps_x[prev_wp];
    double x_y = y - maps_y[prev_wp];

    // find the projection of x onto n
    double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
    double proj_x = proj_norm * n_x;
    double proj_y = proj_norm * n_y;

    double frenet_d = distance_two_point(x_x, x_y, proj_x, proj_y);

    //see if d value is positive or negative by comparing it to a center point
    double center_x = 1000 - maps_x[prev_wp];
    double center_y = 2000 - maps_y[prev_wp];
    double centerToPos = distance_two_point(center_x, center_y, x_x, x_y);
    double centerToRef = distance_two_point(center_x, center_y, proj_x, proj_y);

    if (centerToPos <= centerToRef)
    {
        frenet_d *= -1;
    }

    // calculate s value
    double frenet_s = 0;
    for (int i = 0; i < prev_wp; ++i)
    {
        frenet_s += distance_two_point(maps_x[i], maps_y[i], maps_x[i + 1], maps_y[i + 1]);
    }

    frenet_s += distance_two_point(0, 0, proj_x, proj_y);

    return {frenet_s, -frenet_d};
}

/* *****************************************************************************************************************
- FunctionName: 
- Function    : Transform from Frenet s,d coordinates to Cartesian x,y
- Inputs      : 
- Outputs     : 
- Comments    : 
***************************************************************************************************************** */
vector<double> frenet_to_cartesian(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
    int prev_wp = -1;

    while (s > maps_s[prev_wp + 1] && (prev_wp < (int)(maps_s.size() - 1)))
    {
        ++prev_wp;
    }

    int wp2 = (prev_wp + 1) % maps_x.size();

    double heading = atan2((maps_y[wp2] - maps_y[prev_wp]), (maps_x[wp2] - maps_x[prev_wp]));

    // the x,y,s along the segment
    double seg_s = (s - maps_s[prev_wp]);

    double seg_x = maps_x[prev_wp] + seg_s * cos(heading);
    double seg_y = maps_y[prev_wp] + seg_s * sin(heading);

    double perp_heading = heading - pi() / 2;

    double x = seg_x + d * cos(perp_heading);
    double y = seg_y + d * sin(perp_heading);

    return {x, y};
}

struct Object_Around
{
    double d;
    double s;
    double px;
    double py;
    double v_X;
    double v_Y;
    double v_longitudinal;
    double v_lateral;
    double yaw;
    double yaw_rate;
    double length;
    double width;
    double height;
    string label;
    Object_Around(visualization_msgs::msg::Marker &sensor_fusion_single_result_bounding_box,
                  visualization_msgs::msg::Marker &sensor_fusion_single_result_label,
                  const vector<double> &maps_s,
                  const vector<double> &maps_x,
                  const vector<double> &maps_y)
    {
        this->v_X      = sensor_fusion_single_result_label.points[0].x;
        this->v_Y      = sensor_fusion_single_result_label.points[0].y;
        this->yaw_rate = sensor_fusion_single_result_label.points[0].z;
        this->s        = sensor_fusion_single_result_label.points[1].x;
        this->d        = sensor_fusion_single_result_label.points[1].y;
        this->yaw      = sensor_fusion_single_result_label.points[1].z;
        this->v_longitudinal = v_X * cos(this->yaw) + v_Y * sin(this->yaw);
        this->v_lateral = v_X * sin(this->yaw) - v_Y * cos(this->yaw);
        this->width = distance_two_point(sensor_fusion_single_result_bounding_box.points[0].x,
                                         sensor_fusion_single_result_bounding_box.points[0].y,
                                         sensor_fusion_single_result_bounding_box.points[1].x,
                                         sensor_fusion_single_result_bounding_box.points[1].y);
        this->length = distance_two_point(sensor_fusion_single_result_bounding_box.points[2].x,
                                          sensor_fusion_single_result_bounding_box.points[2].y,
                                          sensor_fusion_single_result_bounding_box.points[3].x,
                                          sensor_fusion_single_result_bounding_box.points[3].y);
        this->height = sensor_fusion_single_result_bounding_box.points[8].z;
        this->label = sensor_fusion_single_result_label.text;
        vector<double> px_py = frenet_to_cartesian(this->s, this->d, maps_s, maps_x, maps_y);
        this->px = px_py[0];
        this->py = px_py[1];
    }
};
#endif