#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/msg/motors.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
  public:
    using SetPosition = custom_interfaces::msg::Motors;

    MinimalPublisher()
    : Node("minimal_publisher")
    {
      publisher_ = this->create_publisher<SetPosition>("set_position", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }
    

  private:
    void timer_callback()
    {
      auto message = SetPosition();
      std::cout << "DIgite o ID do motor:";
      std::cin >> message.id;
      std::cout << "Digite a posicao do motor:";
      std::cin >> message.position;
      
      RCLCPP_INFO(this->get_logger(), "Publishing id: '%d'", message.id);
      RCLCPP_INFO(this->get_logger(), "Publishing position: '%d'", message.position);
      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<custom_interfaces::msg::Motors>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}