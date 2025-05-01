#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <chrono>
#include <random>

class HandoverBehavior {
    public:
        HandoverBehavior() : gen(rd()), dis(0.0, 1.0) {}

    public:
        void registerNodes(BT::BehaviorTreeFactory &factory);

        // Gripper Section
        BT::NodeStatus bt_close_gripper() {
            if (close_gripper(grasp_width, grasp_force)) {
                return BT::NodeStatus::SUCCESS;
            }
            return BT::NodeStatus::FAILURE;
        }
        BT::NodeStatus bt_open_gripper() {
            if (open_gripper(grasp_open)) {
                return BT::NodeStatus::SUCCESS;
            }
            return BT::NodeStatus::FAILURE;
        }
        bool close_gripper(double width, double force) {
            printf("Close Gripper at %.2f width with %.2f force.\n", width, force);
            // TODO: add sleep to simulate moving.
            return true;
        }
        bool open_gripper(double width) {
            printf("Openning gripper to %.2f.\n", width);
            // TODO: add sleep to simulate moving.
            return true;
        }

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
                grasp_pose[i] = dis(gen);
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
                dropoff_pose[i] = dis(gen);
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
        bool robot_ishome = true;
        bool robot_moving = false;

        double home_joint[6] = {0.0, 0.0, 1.0, 0.0, 0.0, 2.1};

        double grasp_pose[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        double dropoff_pose[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        double jointiksol[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        double grasp_open = 0.5;
        double grasp_width = 0.2;
        double grasp_close = 0.0;
        double grasp_force = 5.0;

        double attempt_grasp_probab = 0.5;
        std::random_device rd;
        std::mt19937 gen;
        std::uniform_real_distribution<double> dis;
};
// Helper method to make registering less painful for the user
void HandoverBehavior::registerNodes(BT::BehaviorTreeFactory &factory) {
    factory.registerSimpleAction(
        "OpenGripper", std::bind(&HandoverBehavior::bt_open_gripper, this));
    factory.registerSimpleAction(
        "CloseGripper", std::bind(&HandoverBehavior::bt_close_gripper, this));
    factory.registerSimpleAction(
        "TryGraspObject", std::bind(&HandoverBehavior::bt_try_grasp_object, this));
    factory.registerSimpleAction(
        "TryDropOff", std::bind(&HandoverBehavior::bt_try_dropoff, this));
};

int main() {
    BT::BehaviorTreeFactory factory;

    HandoverBehavior handover;
    handover.registerNodes(factory);

    factory.registerBehaviorTreeFromFile(ament_index_cpp::get_package_share_directory("aljnuho_bt") + "/config/" + "bt_handover.xml");
    auto tree = factory.createTree("MainTree");

    printTreeRecursively(tree.rootNode());

    tree.tickWhileRunning();

    return 0;
}

/*Expected Result
----------------
root_sequence
   OpenGripper
   TryGraspObject
   CloseGripper
   TryDropOff
   OpenGripper
----------------
Openning gripper to 0.50.

Perform IK on :0.02, 0.65, 0.91, 0.13, 0.40, 0.83,
Result Joints :0.52, 1.15, 1.41, 0.63, 0.90, 1.33,
Moving to joints :0.52, 1.15, 1.41, 0.63, 0.90, 1.33,

Close Gripper at 0.20 width with 5.00 force.

Perform IK on :0.97, 0.71, 0.10, 0.05, 0.23, 0.37,
Result Joints :1.47, 1.21, 0.60, 0.55, 0.73, 0.87,
Moving to joints :1.47, 1.21, 0.60, 0.55, 0.73, 0.87,

Openning gripper to 0.50.
*/