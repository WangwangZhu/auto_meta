uint16 cls  # 类别 0 人 1 车
uint16 id # 目标物ID
uint16[] img_xyxy # 图片中的目标物像素坐标
float32 width # 目标物尺寸（实际的）
float32 length # 目标物尺寸（实际的）
float32 height # 目标物尺寸（实际的）
geometry_msgs/Point32 center # 目标物中心坐标点，车辆坐标系下
float32 heading # 航向 弧度单位
float32 speed # 纵向速度 m/s
geometry_msgs/Point32[] traj # 预测轨迹
bool is_abs_position # TBD
