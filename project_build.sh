# colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
# colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=true
# colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=Yes
colcon build --packages-ignore cpp_pubsub
source install/setup.sh