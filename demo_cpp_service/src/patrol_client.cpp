#include <cstdlib>
#include <ctime>
#include <rclcpp/rclcpp.hpp>
#include "chapt4_interfaces/srv/patrol.hpp"
#include <chrono>
#include "rcl_interfaces/msg/parameter.hpp"
#include "rcl_interfaces/msg/parameter_type.hpp"
#include "rcl_interfaces/msg/parameter_value.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"
using SetP = rcl_interfaces::srv::SetParameters;

using namespace std::chrono_literals;
using Patrol = chapt4_interfaces::srv::Patrol;
class PatrolClient : public rclcpp::Node {
public:
    PatrolClient() : Node("patrol_client") {
        patrol_client_ = this->create_client<Patrol>("patrol");
        timer_ = this->create_wall_timer(10s, std::bind(&PatrolClient::timer_callback_, this));
        std::srand(std::time(nullptr));
    }
    void timer_callback_() {
        while (!patrol_client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
        }
        auto request = std::make_shared<Patrol::Request>();
        request->target_x = static_cast<float>(std::rand() % 15);
        request->target_y = static_cast<float>(std::rand() % 15);
        RCLCPP_INFO(this->get_logger(), "Requesting patrol to (x=%f,y=%f)", request->target_x, request->target_y);
        auto result = patrol_client_->async_send_request(request,
            [&](rclcpp::Client<Patrol>::SharedFuture result_future) ->void {
                auto response = result_future.get();
                if(response->result == Patrol::Response::SUCCESS) {
                    RCLCPP_INFO(this->get_logger(), "Patrol command accepted.");
                }else if(response->result == Patrol::Response::FAIL) {
                    RCLCPP_WARN(this->get_logger(), "Patrol command rejected.");
                }else {
                    RCLCPP_ERROR(this->get_logger(), "Patrol error!!!");
                }
            });
        // RCLCPP_WARN(this->get_logger(), "111");
    }

    std::shared_ptr<SetP::Response> call_set_parameters(rcl_interfaces::msg::Parameter& parameter){
        auto param_client = this->create_client<rcl_interfaces::srv::SetParameters>("turtle_controller/set_parameters");
        while (!param_client->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return nullptr;
            }
            RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
        }
        auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
        request->parameters.push_back(parameter);
        auto result = param_client->async_send_request(request);
        rclcpp::spin_until_future_complete(this->get_node_base_interface(), result);
        return result.get();
    }

    void update_server_param_k(double k) {
        rcl_interfaces::msg::Parameter param;
        param.name = "k";
        param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
        param.value.double_value = k;
        auto response = call_set_parameters(param);
        if(response != nullptr) {
            for(auto result : response->results) {
                if(result.successful) {
                    RCLCPP_INFO(this->get_logger(), "Set parameter k to %f successfully.", k);
                }else {
                    RCLCPP_ERROR(this->get_logger(), "Failed to set parameter k to %f. Reason: %s", k, result.reason.c_str());
                }
            }
        }else {
            RCLCPP_ERROR(this->get_logger(), "Service call failed.");
        }
    }
private:
    rclcpp::Client<Patrol>::SharedPtr patrol_client_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PatrolClient>();
    node->update_server_param_k(2.0);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
