#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <chrono>
#include <mutex>
#include <random>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <robotiq2f_interfaces/srv/robotiq2_f_cmd.hpp>

class BTNode;

class HandoverBehavior {
    public:
        HandoverBehavior(BTNode *parent_node) : parent_node_(parent_node) {}

        // Gripper Section
        BT::NodeStatus bt_close_gripper();
        BT::NodeStatus bt_open_gripper();

        // Grasp section
        BT::NodeStatus bt_try_grasp_object() {
            bool s1 = get_grasp_live();
            bool s2 = ik(grasp_pose, jointiksol);
            bool s3 = movej_live(jointiksol);
            if (s1 && s2 && s3) {
                return BT::NodeStatus::SUCCESS;
            }
            return BT::NodeStatus::FAILURE;
        }

        bool get_grasp_live() {
            for (int i = 0; i < 6; i++) {
                grasp_pose[i] = 0.0;
            }
            return true;
        }

        // Dropoff section
        BT::NodeStatus bt_try_dropoff() {
            bool s1 = get_drop_off();
            bool s2 = ik(dropoff_pose, jointiksol);
            bool s3 = movej_live(jointiksol);
            if (s1 && s2 && s3) {
                return BT::NodeStatus::SUCCESS;
            }
            return BT::NodeStatus::FAILURE;
        }

        bool get_drop_off() {
            for (int i = 0; i < 6; i++) {
                dropoff_pose[i] = 0.0;
            }
            return true;
        }

        // Helper
        bool ik(double pose[6], double joints[6]) {
            printf("Perform IK on :");
            for (size_t i = 0; i < 6; i++) {
                printf("%.2f, ", pose[i]);
            }
            printf("\nResult Joints :");
            for (size_t i = 0; i < 6; i++) {
                joints[i] = pose[i] + 0.5;
                printf("%.2f, ", joints[i]);
            }
            printf("\n");
            return true;
        }

        bool movej_live(double j[6]) {
            printf("Moving to joints :");
            for (size_t i = 0; i < 6; i++) {
                printf("%.2f, ", j[i]);
            }
            printf("\n");
            return true;
        }

    private:
        BTNode *parent_node_; // Reference to BTNode for calling call_gripper

        double grasp_pose[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        double dropoff_pose[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        double jointiksol[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        double grasp_open = 0.085;
        double grasp_close = 0.0;
        double grasp_force = 5.0;
};

class BTNode : public rclcpp::Node {
    private:
        rclcpp::Client<robotiq2f_interfaces::srv::Robotiq2FCmd>::SharedPtr robotiq_client;
        std::mutex gripper_mutex; // Mutex for thread safety

    public:
        BT::BehaviorTreeFactory factory;
        HandoverBehavior handover;

        BTNode() : Node("bt_node"), handover(this) {
            factory.registerSimpleAction(
                "OpenGripper", std::bind(&HandoverBehavior::bt_open_gripper, &handover));
            factory.registerSimpleAction(
                "CloseGripper", std::bind(&HandoverBehavior::bt_close_gripper, &handover));
            factory.registerSimpleAction(
                "TryGraspObject", std::bind(&HandoverBehavior::bt_try_grasp_object, &handover));
            factory.registerSimpleAction(
                "TryDropOff", std::bind(&HandoverBehavior::bt_try_dropoff, &handover));
            factory.registerBehaviorTreeFromFile(ament_index_cpp::get_package_share_directory("aljnuho_bt") + "/config/" + "bt_handover.xml");
            auto tree = factory.createTree("MainTree");
            tree.tickWhileRunning();
        }

        bool call_gripper(double width, double force) {
            robotiq_client = this->create_client<robotiq2f_interfaces::srv::Robotiq2FCmd>("gripper_command");

            while (!robotiq_client->wait_for_service(std::chrono::seconds(1))) {
                RCLCPP_INFO(this->get_logger(), "The server is currently unavailable !");
            };

            auto req = std::make_shared<robotiq2f_interfaces::srv::Robotiq2FCmd::Request>();
            req->pos = width;
            req->vel = 0.0;
            req->force = force;

            auto future = robotiq_client->async_send_request(req);

            try {
                auto res = future.get();
                RCLCPP_INFO(this->get_logger(), "Req is %s", res->message.c_str());
                return true;
            } catch (const std::exception &e) {
                std::cerr << e.what() << '\n';
                RCLCPP_INFO(this->get_logger(), "Service call failed.");
                return false;
            }
        }
};

// Implement HandoverBehavior methods that depend on BTNode
BT::NodeStatus HandoverBehavior::bt_close_gripper() {
    if (parent_node_->call_gripper(grasp_close, grasp_force)) {
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
}

BT::NodeStatus HandoverBehavior::bt_open_gripper() {
    if (parent_node_->call_gripper(grasp_open, grasp_force)) {
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto exe = rclcpp::executors::MultiThreadedExecutor();
    auto node = std::make_shared<BTNode>();
    exe.add_node(node);
    exe.spin();
    rclcpp::shutdown();
    return 0;
}
