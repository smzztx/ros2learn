#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class StaticFramePublisher : public rclcpp::Node
{
public:
    StaticFramePublisher()
    : Node("static_frame_publisher")
    {
        static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        this->publish_tf();
    }

    void publish_tf()
    {
        geometry_msgs::msg::TransformStamped t;

        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "map";
        t.child_frame_id = "target_point";

        t.transform.translation.x = 5.0;
        t.transform.translation.y = 3.0;
        t.transform.translation.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, 60 * M_PI / 180); // Roll, Pitch, Yaw
        t.transform.rotation = tf2::toMsg(q);

        static_broadcaster_->sendTransform(t);
    }

private:
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StaticFramePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}