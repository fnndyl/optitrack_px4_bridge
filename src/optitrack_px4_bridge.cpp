#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <optitrack_multiplexer_ros2_msgs/msg/rigid_body_stamped.hpp>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class OdometryPublisher : public rclcpp::Node
{
public:
  OdometryPublisher()
  : Node("odometry_publisher")
  {
      mocap_subscription_ = this->create_subscription<optitrack_multiplexer_ros2_msgs::msg::RigidBodyStamped>(
        "/optitrack_multiplexer_node/rigid_body/JohnMocap", 10, std::bind(&OdometryPublisher::mocap_callback, this, _1));

      vehicle_odometry_publisher_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>(
        "/fmu/out/vehicle_visual_odometry", 10);
  }

private:

  rclcpp::Subscription<optitrack_multiplexer_ros2_msgs::msg::RigidBodyStamped>::SharedPtr mocap_subscription_;
  rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odometry_publisher_;

  void mocap_callback(const optitrack_multiplexer_ros2_msgs::msg::RigidBodyStamped & msg)
  {
    // Populate PX4 vehicle visual odometry
    px4_msgs::msg::VehicleOdometry vehicle_odom;
    vehicle_odom.pose_frame = 1; // corresponds to NED earth-fixed frame - see .msg definition
    vehicle_odom.velocity_frame = 1; // corresponds to NED earth-fixed frame - see .msg definition
    vehicle_odom.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  
    // Mapping as per: https://docs.px4.io/main/en/ros/external_position_estimation.html

    // Populate position data
    vehicle_odom.position[0] = msg.rigid_body.pose.position.x; // x
    vehicle_odom.position[1] = msg.rigid_body.pose.position.z; // y
    vehicle_odom.position[2] = -msg.rigid_body.pose.position.y; // z

    // Populate orientation data
    vehicle_odom.q[0] = msg.rigid_body.pose.orientation.q_w; // w
    vehicle_odom.q[1] = msg.rigid_body.pose.orientation.q_x; // x
    vehicle_odom.q[2] = msg.rigid_body.pose.orientation.q_z; // y
    vehicle_odom.q[3] = -msg.rigid_body.pose.orientation.q_y; // z

    vehicle_odometry_publisher_->publish(vehicle_odom);

  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdometryPublisher>());
  rclcpp::shutdown();
  return 0;
}