#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp" // Corrigido para importar a mensagem correta
#include "sensor_msgs/msg/joy.hpp"

class JointControlNode : public rclcpp::Node {
public:
    JointControlNode() : Node("joint_control_node") {
        subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy",
            10,
            std::bind(&JointControlNode::joy_callback, this, std::placeholders::_1)
        );
        
        // Corrigido para usar a mensagem correta de trajectory_msgs
        publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/joint_trajectory_controller/joint_trajectory",
            10
        );

        // Nomes das juntas
        joint_names_0 = {
            "cobra_body_1_joint", "cobra_body_1_aux_joint",
            "cobra_body_2_joint", "cobra_body_2_aux_joint",
            "cobra_body_3_joint", "cobra_body_3_aux_joint"
        };

        joint_names_3 = {
            "cobra_body_4_joint", "cobra_body_4_aux_joint",
            "cobra_body_5_joint", "cobra_body_5_aux_joint",
            "cobra_body_6_joint", "cobra_body_6_aux_joint"
        };
    }

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg) {
        // Mapeia os valores dos joysticks para comandos de trajetória
        auto joint_goal_msg = std::make_shared<trajectory_msgs::msg::JointTrajectory>();
        joint_goal_msg->joint_names = (joy_msg->axes[0] != 0) ? joint_names_0 : joint_names_3;

        // Define a posição das juntas com base nos valores dos joysticks
        std::vector<double> positions = {
            joy_msg->axes[0] * 0.5, 
            joy_msg->axes[0] * 0.0,
            joy_msg->axes[0] * 0.5,
            joy_msg->axes[0] * 0.0,
            joy_msg->axes[0] * 0.5,
            joy_msg->axes[0] * 0.0
        };

        // Cria um ponto de trajetória
        trajectory_msgs::msg::JointTrajectoryPoint trajectory_point;
        trajectory_point.positions = positions;
        trajectory_point.time_from_start.sec = 1; // Substitua pelo tempo desejado

        // Define os pontos de trajetória
        joint_goal_msg->points.push_back(trajectory_point);

        publisher_->publish(*joint_goal_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
    std::vector<std::string> joint_names_0;
    std::vector<std::string> joint_names_3;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JointControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
