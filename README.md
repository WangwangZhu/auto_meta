# 关于工程文件启动的说明

1. 该工作空间一定要放在 `Ubuntu`系统的 `home`文件夹下，因为有些路径处理级别是和 `home`文件见的绝对路径严格对应的。
   如果需要放到其他路径下，需要修改的文件有：底盘通信包，惯导包
2. `key_board_monitor.launch.py` 该文件启动后，软件监控全局键盘输入，这时如果操作键盘的话，尤其是按下 `eq`两个按键时，自动驾驶模式会被意外启动或意外停止。

# 实车代码启动流程：

ros2 run chassis_communication chassis_communication_chasis_to_ros

ros2 run chassis_communication chassis_communication_ros_to_chasis

ros2 run gear_control gear_controller

ros2 run integrated_navigation_system ins_d_data_parse

ros2 launch launch_manager vehicle_path_visualization_rviz_launch.py

rqt

ros2 run key_board_monitor key_board_monitor

ros2 launch launch_manager mpc_trajectory_tracking_dynamics_coupled_launch.py

ros2 launch launch_manager lattice_planner_launch.py

ros2 run fsm_decision_making fsm_decision_making_node

ros2 run sensor_fusion sensor_fusion_node

# 联合仿真代码启动流程（基于ROS galactic neotic)

1. source ros1_terminal_setup.sh + roscore
2. source ros1_bridge_env_setup.sh + ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
3. source ros2_terminal_setup.sh + ros2 launch launch_manager vehicle_path_visualization_rviz_launch.py
4. source ros2_terminal_setup.sh + ros2 run chassis_communication chassis_communication_ros_to_chasis
5. source ros2_terminal_setup.sh + ros2 run chassis_communication chassis_communication_chasis_to_ros
6. source ros2_terminal_setup.sh + ros2 launch launch_manager mpc_trajectory_tracking_dynamics_coupled_launch.py
7. source ros2_terminal_setup.sh + rqt
8. source ros2_terminal_setup.sh + ros2 run fsm_decision_making fsm_decision_making_node
9. source ros2_terminal_setup.sh + ros2 launch launch_manager lattice_planner_launch.py
10. source ros2_terminal_setup.sh + ros2 run sensor_fusion sensor_fusion_node

# 关于坐标系变换的说明:

mpc中预测模型车辆坐标系:为了便于计算,在mpc中,将全局路径变换到车辆坐标系下,该车辆坐标系的 x 指向车辆前方,y指向车辆左侧

## ROS/RVIZ：

符合REP：103标准

在Gazebo或着rviz中显示为红：X  绿：Y  蓝：Z

ROS系统中,rviz里面的 base_link 车辆坐标系: 在 host_vehicle_visualization.cpp 定义了base_link与odom的变换关系,该变换关系来自ins_d的惯导信息,

ROS系统中,rviz里面的 odom 坐标系: 该全局坐标系向x指向正东,y指向正北,

## 局部坐标系（车辆坐标系）：

Carsim：x沿着车辆纵向方向，向前为正，从车屁股看，y指向左侧，坐标原点位于前桥中间在地面上的投影点

白皮书：x沿着车辆纵向方向，向前为正，从车屁股看，y指向左侧，坐标原点位于后桥中间在地面上的投影点

simulink：x沿着车辆纵向方向，向前为正，从车屁股看，y指向左侧，坐标原点位于前桥中间在地面上的投影点

ros/rviz：x沿着车辆纵向方向，向前为正，从车屁股看，y指向左侧，

惯导本体：y沿着车辆纵向方向，向前为正，从车屁股看，x指向右侧，坐标原点位于E100后备箱中间

## 世界坐标系（右手坐标系，拇指X，食指Y,中指Z）：

右手坐标系的旋转正方向：从轴的正方向看向原点，逆时针方向即是旋转正向。或者，伸出右手，拇指指向旋转轴正向，四指弯曲，四指指向的旋转方向就是正向。

右手坐标系：把右手拇指食指中指伸直并正交，拇指X，食指Y，中指Z

Carsim：X指向正东，Y指向正北，Z垂直向上，

白皮书：X指向正东，Y指向正北，Z垂直向上，

Simulink：X指向正东，Y指向正北，Z垂直向上，

INSD原始数据：只有经纬度和高程，不存在世界笛卡尔坐标系

ROS/rviz：X指向正东，Y指向正北，Z垂直向上，

惯导解析节点发出定位信息所使用的全局坐标系：UTM，X指向正东，Y指向正北，Z垂直向上，

## 航向（横摆角）（右手坐标系）：

Carsim：航向起始于X轴正方向（正东），绕Z轴逆时针为正

白皮书：航向起始于X轴正方向（正东），绕Z轴逆时针为正

Simulink：航向起始于X轴正方向（正东），绕Z轴逆时针为正

惯导本体：heading 顺时针为正，0度在正北方向(ins-d)（正东）（根据手册，不满足右手系）

惯导解析节点发出的定位信息：航向起始于X轴正方向（正东），绕Z轴逆时针为正（450deg - raw_heading）

## 俯仰角（右手坐标系）：

其他：车屁股抬起，俯仰角为正

惯导本体：pitch 车头向上抬的时候，角度为正(ins-d)（根据手册，满足右手系）

惯导解析节点发出的定位信息：车屁股抬起，俯仰角为正

## 侧倾角（右手坐标系）：

其他：从车屁股方向看，顺时针绕x轴为正

惯导本体：roll  车身右倾斜的时候，角度为正(ins-d)（根据手册，满足右手系）

惯导解析节点发出的定位信息：roll  车身右倾斜的时候，角度为正(ins-d)（根据手册，满足右手系）

## 横摆角速度/横摆角加速度/俯仰角速度/俯仰角加速度/侧倾角速度/侧倾角加速度：

除了惯导本体与惯导解析节点：都满足右手坐标系定义，伸出右手，拇指指向旋转轴正向，四指弯曲，四指指向的旋转方向就是正向。

## 单位：

Carsim：方向盘转角（deg）heading（deg）pitch（deg）roll（deg）VxVyVz（km/h）三轴角速度（deg/s）三轴线加速度（g）笛卡尔坐标系位置（m）四个轮子的转角（deg）

simulink：方向盘转角（deg）heading（rad）pitch（rad）roll（rad）VxVyVz（m/s）（通过Can发送出去的纵向速度单位km/h）三轴角速度（rad/s）三轴线加速度（m/s^2）笛卡尔坐标系位置（m）四个轮子的转角（deg）

## 方向盘/前轮转角：

Carsim：逆时针为正
simulink：顺时针为正
E100车辆：顺时针为正

NeZha: 顺时针为负

# MPC 里面可以调节的参数

1. 预测模型里的系数以及模型本身
2. 时延补偿模型里面的车辆运动学模型
3. 模型预测控制器本身的参数:各项的权重,时域长度,时域步长
4. 车辆的动力学与运动学几何参数

# FSM+highway 调节

1. 在头文件里面有个 “int preference_lane_id =0; // TODO: 这个0可以调节车辆的期望车道”，这个变量在两个包里面都有，如果要改，两个包里面要一起改。

# 建立Roadrunner地图的时候录制的 CSV 文件信息

2023_02_01_09_19_01_ins_data_map 长直道
2023_02_01_09_24_25_ins_data_map 二餐厅东边道路
2023_02_01_09_29_26_ins_data_map 外环路
2023_02_01_09_38_39_ins_data_map 一餐厅到梅花桩，内环路
2023_02_01_09_46_51_ins_data_map 三餐厅东面直路到学服
2023_02_01_09_50_36_ins_data_map 五餐厅东，包围电院，含凯旋门正对面小弧线
2023_02_01_09_58_16_ins_data_map 北三门进来

# carla 里面方向盘左正右负

# PID & Foxy 启动流程

1. cd /carla-ros-bridge
2. source source_env.sh
3. colcon build
4. source source_env.sh
5. ros2 launch carla_zww_bridge_ego_vis carla_bridge_ego_vehilce.launch.py
6. 在新的终端里面: ros2 run carla_zww_pid_controller carla_zww_pid_controller_node

# Stanley & Foxy 需要完成的内容

1. src/carla_ros_bridge/carla_zww_projects/carla_zww_stanley_pid_controller/src/stanley_controller.cpp 中的 TODO 部分

# Stanley & Foxy 启动流程

1. cd /carla-ros-bridge
2. source source_env.sh
3. colcon build
4. source source_env.sh
5. ros2 launch carla_zww_bridge_ego_vis carla_bridge_ego_vehilce.launch.py
6. 在新的终端里面: ros2 run carla_zww_stanley_pid_controller carla_zww_stanley_pid_controller_node

# LQR & Foxy 需要完成的内容

1. carla-ros-bridge/src/carla_ros_bridge/carla_zww_projects/carla_zww_lqr_pid_controller/src/lqr_controller.cpp 中的 TODO 部分

# LQR & Foxy 启动流程

1. cd /carla-ros-bridge
2. source source_env.sh
3. colcon build
4. source source_env.sh
5. ros2 launch carla_zww_bridge_ego_vis carla_bridge_ego_vehilce.launch.py
6. 在新的终端里面: ros2 launch carla_zww_lqr_pid_controller lqr_launch.py

# MPC & Foxy 需要完成的内容

1. carla-ros-bridge/src/carla_ros_bridge/carla_zww_projects/carla_zww_mpc_controller/src/mpc_controller.cpp 中的 TODO 部分

# MPC & Foxy 启动流程

1. cd /carla-ros-bridge
2. source source_env.sh
3. colcon build
4. source source_env.sh
5. ros2 launch carla_zww_bridge_ego_vis carla_bridge_ego_vehilce.launch.py
6. 在新的终端里面: ros2 launch carla_zww_mpc_controller mpc_launch.py

# Lattice & Foxy 启动流程

1. 需要完成部分： lattice_planner.cpp 中的TODO部分
2. cd /carla-ros-bridge
3. source source_env.sh
4. colcon build
5. source source_env.sh
6. ros2 launch carla_zww_bridge_ego_vis carla_bridge_ego_vehilce.launch.py
7. 在新的终端里面: ros2 launch carla_zww_lattice_planner lattice_launch.py

# A* & Foxy 启动流程

1. cd /carla-ros-bridge
2. source source_env.sh
3. colcon build
4. source source_env.sh
5. ros2 launch carla_zww_a_star_planner a_star_planner.launch.py

## 作业要求

1. carla-ros-bridge/src/carla_ros_bridge/carla_zww_projects/carla_zww_a_star_planner/src/Astar_searcher.cpp 中的 TODO部分
2. 对比分析不同的启发函数的计算耗时，每次运行后在终端内会打印计算时间，需要截图放入文档中上传。

# Carla 联合仿真启动流程（纯仿真）

1. cd /home/zww/carla/Unreal/CarlaUE4/Saved/StagedBuilds/LinuxNoEditor + ./CarlaUE4.sh
2. source ros2_terminal_setup.sh + ros2 launch carla_zww_bridge_ego_vis carla_bridge_ego_vehilce.launch.py
3. source ros2_terminal_setup.sh + ros2 launch launch_manager vehicle_path_visualization_rviz_launch.py
4. source ros2_terminal_setup.sh + ros2 run sensor_fusion sensor_fusion_node
5. source ros2_terminal_setup.sh + ros2 run fsm_decision_making fsm_decision_making_node
6. source ros2_terminal_setup.sh + ros2 launch lattice_planner lattice_planner_launch.py 
7. source ros2_terminal_setup.sh + ros2 launch lqr_pid_trajectory_tracking lqr_pid_trajectory_tracking_dynamics_launch.py

# Carla仿真 到 实车测试的改动

1. mpc_parameters_configuration_dynamics_coupled.yaml 中的控制器参数
2. mpc_parameters_configuration_dynamics_coupled.cpp 中的订阅信号 50 行
3. sensor_fusion.cpp 中订阅信号 25行
4. fsm_decision_making.cpp 中定位订阅信号
5. basic_planner.cpp 中定位订阅信号
