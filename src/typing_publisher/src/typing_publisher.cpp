#include "typing_publisher.h"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

MinimalPublisher::MinimalPublisher()
: Node("typing_publisher"), count_(0)
{
  publisher_ = this->create_publisher<std_msgs::msg::String>("typing_topic", 10);
  timer_ = this->create_wall_timer(500ms, std::bind(&MinimalPublisher::timer_callback, this));//*/
  std::cout<<"Type w, a, s, d to move. Use zxrtfgcv to change orientation. Type 'h' to change step size (default is 10 rviz units). Type 'n' to reset. Type 'm' to open/close gripper. Type 'b' to plan to the orange arm in rviz."<<std::endl;
}

void MinimalPublisher::timer_callback()
{
  auto message = std_msgs::msg::String();
  std::cin>>message.data;
  publisher_->publish(message);
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  
  /*rclcpp::executors::SingleThreadedExecutor executor;
  auto node = std::make_shared<MinimalPublisher>();
  executor.add_node(node);
  auto spinner = std::thread([&executor]() {executor.spin(); });
  spinner.join();*/
  rclcpp::shutdown();
  return 0;
}
