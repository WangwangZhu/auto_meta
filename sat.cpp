bool ShapeOverlap_SAT(polygon &r1, polygon &r2){
    polygon *poly1 = &r1;
    polygon *poly2 = &r2;

    // 投影的大小：通过将一个多边形上的每个顶点与原点(0,0)组成的向量，投影在某一投影轴上，然后保留该多边形在该投影轴上所有投影中的最大值和最小值，这样即可表示一个多边形在某投影轴上的投影了。
    // 一个边，两个点，分别构造两个从原点出发的向量，分别计算这两个向量在投影轴上的投影结果（与投影轴同方向的单位向量的点积），两个投影点形成的线段就是当前边在当前投影轴上的投影结果

    // 一个边的两个点构成的向量的垂直向量，进行归一化后就是投影轴的向量表达结果。(x, y)的垂直向量（-y, x）

    // 投影轴平行于边缘法向量。投影轴的位置不限，因为其长度是无限的，故而多边形在该轴上的投影是一样的。该轴的方向才是关键的。
    // 判断两多边形的投影重合：projection1.max > projection2.min && project2.max > projection.min

    for(int shape = 0; shape < 2; shape++){
        if (shape == 1){
            poly1 = &r2;
            poly2 = &r1;
        }
        for(int a = 0; a < poly1->p.size(); a++){
            int b = (a+1) % poly1->p.size();
            vec2d axisProj = {-(poly1->p[b].y - poly1->p[a].y), poly1->p[b].x - poly1->p[a].x };
            float d = sqrtf(axisProj.x * axisProj.x + axisProj.y * axisProj.y);
            axisProj = {axisProj.x / d, axisProj.y / d};

            // work out min and max 1D points for r1
            float min_r1 = INFINITY, max_r1 = -INFINITY;
            for(int p = 0; p < poly1->p.size(); p++){
                float q = (poly1->p[p].x * axisProj.x + poly1->p[p].y * axisProj.y);
                min_r1 = std::min(min_r1, q);
                max_r1 = std::max(max_r1, q);
            }

            // Work out min and max 1D points for r2
            float min_r2 = INFINITY, max_r2 = -INFINITY;
            for(int p = 0; p < poly2->p.size(); p++){
                flaot q = (poly2->p[p].x * axisProj.x + poly2->p[p].y * axisProj.y);
                min_r2 = std::min(min_r2, q);
                max_r2 = std::max(max_r2, q);
            }

            // not (A and B) == not A or not B
            // not (A or B) == not A and not B 
            if (!(max_r2 >= min_r1 && max_r1 >= min_r2)){
                return false;
            }
        }
        return true;
    }
}

if (box.max_x() < min_x() || box.min_x() > max_x() || box.max_y() < min_y() || box.min_y() > max_y()) {
    return false;
}

const auto& trajectory_point = discretized_trajectory.TrajectoryPointAt(static_cast<std::uint32_t>(i));
double ego_theta = trajectory_point.path_point().theta();
Box2d ego_box({trajectory_point.path_point().x(), trajectory_point.path_point().y()}, ego_theta, ego_length, ego_width);
// 车辆center和车辆的几何中心不重合，所以box需要校正一下
double shift_distance = ego_length / 2.0 - vehicle_config.vehicle_param().back_edge_to_center();
Vec2d shift_vec{shift_distance * std::cos(ego_theta),  shift_distance * std::sin(ego_theta)};
ego_box.Shift(shift_vec);

const double shift_x = box.center_x() - center_.x();
const double shift_y = box.center_y() - center_.y();

const double dx1 = cos_heading_ * half_length_;
const double dy1 = sin_heading_ * half_length_;
const double dx2 = sin_heading_ * half_width_;
const double dy2 = -cos_heading_ * half_width_;
const double dx3 = box.cos_heading() * box.half_length();
const double dy3 = box.sin_heading() * box.half_length();
const double dx4 = box.sin_heading() * box.half_width();
const double dy4 = -box.cos_heading() * box.half_width();
// 对于OBB边框，使用分离轴定理进行碰撞检测
return std::abs(shift_x * cos_heading_ + shift_y * sin_heading_) <= std::abs(dx3 * cos_heading_ + dy3 * sin_heading_) + std::abs(dx4 * cos_heading_ + dy4 * sin_heading_) + half_length_ &&
       std::abs(shift_x * sin_heading_ - shift_y * cos_heading_) <= std::abs(dx3 * sin_heading_ - dy3 * cos_heading_) + std::abs(dx4 * sin_heading_ - dy4 * cos_heading_) + half_width_ &&
       std::abs(shift_x * box.cos_heading() + shift_y * box.sin_heading()) <= std::abs(dx1 * box.cos_heading() + dy1 * box.sin_heading()) + std::abs(dx2 * box.cos_heading() + dy2 * box.sin_heading()) + box.half_length() &&
       std::abs(shift_x * box.sin_heading() - shift_y * box.cos_heading()) <= std::abs(dx1 * box.sin_heading() - dy1 * box.cos_heading()) + std::abs(dx2 * box.sin_heading() - dy2 * box.cos_heading()) + box.half_width();













































