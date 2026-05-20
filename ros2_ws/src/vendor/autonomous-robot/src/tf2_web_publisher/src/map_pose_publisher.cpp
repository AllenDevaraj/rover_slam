#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class MapPosePublisher : public rclcpp::Node
{
public:
    MapPosePublisher() : Node("map_pose_publisher")
    {
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/robot_pose", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&MapPosePublisher::publishPose, this));
    }

private:
    void publishPose()
    {
        try
        {
            auto transform = tf_buffer_->lookupTransform(
                "map", "base_link",
                tf2::TimePointZero);

            auto pose = geometry_msgs::msg::PoseStamped();
            pose.header = transform.header;
            pose.pose.position.x = transform.transform.translation.x;
            pose.pose.position.y = transform.transform.translation.y;
            pose.pose.position.z = transform.transform.translation.z;
            pose.pose.orientation = transform.transform.rotation;

            pose_pub_->publish(pose);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Transform error: %s", ex.what());
        }
    }

    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapPosePublisher>());
    rclcpp::shutdown();
    return 0;
}