#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_cpp/bt_factory.h>

// Custom
struct Pose2D {
        double x, y, theta;
};

namespace BT {
template <>
Pose2D convertFromString(StringView str) {
    // The input string is expected to be in the format "x;y;theta"
    auto parts = splitString(str, ';');
    if (parts.size() != 3) {
        throw RuntimeError("Invalid input for Pose2D. Expected format: x;y;theta");
    }
    Pose2D pose;
    pose.x = convertFromString<double>(parts[0]);
    pose.y = convertFromString<double>(parts[1]);
    pose.theta = convertFromString<double>(parts[2]);
    return pose;
}
} // namespace BT

namespace chr = std::chrono;

class MoveBaseAction : public BT::StatefulActionNode {
    public:
        // Any TreeNode with ports must have a constructor with this signature
        MoveBaseAction(const std::string &name, const BT::NodeConfig &config)
            : StatefulActionNode(name, config) {}

        // It is mandatory to define this static method.
        static BT::PortsList providedPorts() {
            return {BT::InputPort<Pose2D>("goal")};
        }

        // this function is invoked once at the beginning.
        BT::NodeStatus onStart() override;

        // If onStart() returned RUNNING, we will keep calling
        // this method until it return something different from RUNNING
        BT::NodeStatus onRunning() override;

        // callback to execute if the action was aborted by another node
        void onHalted() override;

    private:
        Pose2D _goal;
        chr::system_clock::time_point _completion_time;
};

//-------------------------

BT::NodeStatus MoveBaseAction::onStart() {
    if (!getInput<Pose2D>("goal", _goal)) {
        throw BT::RuntimeError("missing required input [goal]");
    }
    printf("[ MoveBase: SEND REQUEST ]. goal: x=%f y=%f theta=%f\n",
           _goal.x, _goal.y, _goal.theta);

    // We use this counter to simulate an action that takes a certain
    // amount of time to be completed (200 ms)
    _completion_time = chr::system_clock::now() + chr::milliseconds(220);

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MoveBaseAction::onRunning() {
    // Pretend that we are checking if the reply has been received
    // you don't want to block inside this function too much time.
    std::this_thread::sleep_for(chr::milliseconds(10));

    // Pretend that, after a certain amount of time,
    // we have completed the operation
    if (chr::system_clock::now() >= _completion_time) {
        std::cout << "[ MoveBase: FINISHED ]" << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
}

void MoveBaseAction::onHalted() {
    printf("[ MoveBase: ABORTED ]");
}

// Simple function that return a NodeStatus
BT::NodeStatus CheckBattery() {
    std::cout << "[ Battery: OK ]" << std::endl;
    return BT::NodeStatus::SUCCESS;
}
class SaySomething : public BT::SyncActionNode {
    public:
        // If your Node has ports, you must use this constructor signature
        SaySomething(const std::string &name, const BT::NodeConfig &config)
            : SyncActionNode(name, config) {}

        // It is mandatory to define this STATIC method.
        static BT::PortsList providedPorts() {
            // This action has a single input port called "message"
            return {BT::InputPort<std::string>("message")};
        }

        // Override the virtual function tick()
        BT::NodeStatus tick() override {
            BT::Expected<std::string> msg = getInput<std::string>("message");
            // Check if expected is valid. If not, throw its error
            if (!msg) {
                throw BT::RuntimeError("missing required input [message]: ",
                                       msg.error());
            }
            // use the method value() to extract the valid message.
            std::cout << "Robot says: " << msg.value() << std::endl;
            return BT::NodeStatus::SUCCESS;
        }
};
int main() {
    BT::BehaviorTreeFactory factory;
    factory.registerSimpleCondition("BatteryOK", std::bind(CheckBattery));
    factory.registerNodeType<MoveBaseAction>("MoveBase");
    factory.registerNodeType<SaySomething>("SaySomething");

    auto tree = factory.createTreeFromFile(ament_index_cpp::get_package_share_directory("aljnuho_bt") + "/config/" + "bt_longaction.xml");

    // Here, instead of tree.tickWhileRunning(),
    // we prefer our own loop.
    std::cout << "--- ticking\n";
    auto status = tree.tickOnce();
    std::cout << "--- status: " << toStr(status) << "\n\n";

    while (status == BT::NodeStatus::RUNNING) {
        // Sleep to avoid busy loops.
        // do NOT use other sleep functions!
        // Small sleep time is OK, here we use a large one only to
        // have less messages on the console.
        tree.sleep(std::chrono::milliseconds(100));

        std::cout << "--- ticking\n";
        status = tree.tickOnce();
        std::cout << "--- status: " << toStr(status) << "\n\n";
    }

    return 0;
}