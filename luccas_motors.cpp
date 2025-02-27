#include <iostream>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/msg/motors.hpp"

class MinimalPublisher : public rclcpp::Node
{
public:
    using SetPosition = custom_interfaces::msg::Motors;

    MinimalPublisher()
        : Node("minimal_publisher")
    {
        publisher_ = this->create_publisher<SetPosition>("set_position", 10);
        run();
    }

private:
    void run()
    {
        while (rclcpp::ok()) 
        {
            auto message = SetPosition();

            
            while (true)
            {
                if (!get_input("Digite o ID do motor (0 a 20): ", message.id) || message.id < 0 || message.id > 20)
                {
                    RCLCPP_ERROR(this->get_logger(), "Erro: O ID do motor deve estar entre 0 e 20.");
                    continue; 
                break; 
            }

            
            while (true)
            {
                if (!get_input("Digite a posição do motor: ", message.position))
                {
                    RCLCPP_ERROR(this->get_logger(), "Erro: A posição deve ser um número inteiro válido.");
                    continue; 
                }
                break; 
            }

           
            RCLCPP_INFO(this->get_logger(), "Publicando - ID: %d, Posição: %d", message.id, message.position);
            publisher_->publish(message);
        }
    }

   
    bool get_input(const std::string &prompt, int &value)
    {
        std::cout << prompt;
        std::string input;
        std::getline(std::cin, input);

        try
        {
            value = std::stoi(input);
            return true;
        }
        catch (const std::exception &e)
        {
            return false; 
        }
    }

    rclcpp::Publisher<SetPosition>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
}
