# colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
# colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=true
# colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=Yes
colcon build --packages-ignore matrix_vehicle_chassis_communication ultrasonic_sensor
source install/setup.sh