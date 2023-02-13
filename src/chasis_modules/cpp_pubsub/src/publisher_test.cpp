// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "tutorial_interfaces/msg/msg_to_can.hpp"  

using namespace std::chrono_literals;
using namespace std;
/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("minimal_publisher_test"), count_(0)
  {
    publisher_ = this->create_publisher<tutorial_interfaces::msg::MsgToCan>("to_can", 10);
    timer_ = this->create_wall_timer(
      2000ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = tutorial_interfaces::msg::MsgToCan();
    message.connect = 1;
    message.forward = 1;
    message.back = 0;
    message.buzzer = 0;
    message.clock = 0;
    message.speed = 1000;
    message.acc = 2.5; // 在2.5s内到达指定转速
    message.dec = 1.5; // 没有速度请求的时候在1.5S内减速到0
    message.angle = -30;
    RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: '" << message.connect << " " << message.forward << " " << message.speed <<"'");
    publisher_->publish(message);


    cout<<this->count<<endl;
    this->count ++;
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<tutorial_interfaces::msg::MsgToCan>::SharedPtr publisher_;
  size_t count_;
  int count = 0;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
