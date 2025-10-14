#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/exceptions.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <chrono>

using namespace std::chrono_literals;

class TFListenerNode : public rclcpp::Node
{
public:
    TFListenerNode()
    : Node("tf_listener_node")
    {
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        timer_ = this->create_wall_timer(5s, std::bind(&TFListenerNode::lookup_transform, this));
    }

    void lookup_transform()
    {
        geometry_msgs::msg::TransformStamped transformStamped;
        try
        {
            transformStamped = tf_buffer_->lookupTransform("base_link", "target_point", this->get_clock()->now(), 1s);
            RCLCPP_INFO(this->get_logger(), "Transform: Translation(%.2f, %.2f, %.2f), Rotation(%.2f, %.2f, %.2f, %.2f)",
                        transformStamped.transform.translation.x,
                        transformStamped.transform.translation.y,
                        transformStamped.transform.translation.z,
                        transformStamped.transform.rotation.x,
                        transformStamped.transform.rotation.y,
                        transformStamped.transform.rotation.z,
                        transformStamped.transform.rotation.w);
        }
        catch (tf2::TransformException & ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
        }
    }
    
private:
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TFListenerNode>();
    rclcpp::spin(node);
    node.reset();
    rclcpp::shutdown();

    return 0;
}