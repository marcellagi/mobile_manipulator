#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp" 
#include "sensor_msgs/msg/joy.hpp"

class JointControlNode : public rclcpp::Node {
public:
    JointControlNode() : Node("joint_control_node") {
        subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy",
            10,
            std::bind(&JointControlNode::joy_callback, this, std::placeholders::_1)
        );
        
        publisher_1 = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/joint_trajectory_controller/joint_trajectory",
            10
        );

        joint_names = {
            "cobra_body_1_joint", "cobra_body_1_aux_joint",
            "cobra_body_2_joint", "cobra_body_2_aux_joint",
            "cobra_body_3_joint", "cobra_body_3_aux_joint",
            "cobra_body_4_joint", "cobra_body_4_aux_joint",
            "cobra_body_5_joint", "cobra_body_5_aux_joint",
            "cobra_body_6_joint", "cobra_body_6_aux_joint"
        };

    }

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg) {

        auto joint_goal_msg1 = std::make_shared<trajectory_msgs::msg::JointTrajectory>();
        joint_goal_msg1->joint_names = joint_names;

        auto joint_goal_msg2 = std::make_shared<trajectory_msgs::msg::JointTrajectory>();
        joint_goal_msg2->joint_names = joint_names;
        
          std::vector<double> positions1 = {
              joy_msg->axes[0] * 0.5, 
              joy_msg->axes[1] * 0.5,
              joy_msg->axes[0] * 0.5,
              joy_msg->axes[1] * 0.5,
              joy_msg->axes[0] * 0.5,
              joy_msg->axes[1] * 0.5,
              joy_msg->axes[3] * 0.5, 
              joy_msg->axes[4] * 0.5,
              joy_msg->axes[3] * 0.5,
              joy_msg->axes[4] * 0.5,
              joy_msg->axes[3] * 0.5,
              joy_msg->axes[4] * 0.5
          };
      

        trajectory_msgs::msg::JointTrajectoryPoint trajectory_point1;
        trajectory_point1.positions = positions1;
        trajectory_point1.time_from_start.sec = 1; 

        joint_goal_msg1->points.push_back(trajectory_point1);

   
        publisher_1->publish(*joint_goal_msg1);

    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_1;
   
    std::vector<std::string> joint_names;

};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JointControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
