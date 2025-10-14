#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <chrono>

using namespace std::chrono_literals;

class DynamicTFBroadcaster : public rclcpp::Node
{
public:
    DynamicTFBroadcaster()
    : Node("dynamic_tf_broadcaster")
    {
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        timer_ = this->create_wall_timer(10ms, std::bind(&DynamicTFBroadcaster::publish_tf, this));
    }

    void publish_tf()
    {
        geometry_msgs::msg::TransformStamped t;

        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "map";
        t.child_frame_id = "base_link";

        t.transform.translation.x = 2.0;
        t.transform.translation.y = 3.0;
        t.transform.translation.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, 30 * M_PI / 180); // Roll, Pitch, Yaw
        t.transform.rotation = tf2::toMsg(q);

        tf_broadcaster_->sendTransform(t);
    }

private:
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DynamicTFBroadcaster>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}