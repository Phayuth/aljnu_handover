#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_cpp/bt_factory.h>

using namespace BT;
// SyncActionNode (synchronous action) with an input port.
class SaySomething : public SyncActionNode {
    public:
        // If your Node has ports, you must use this constructor signature
        SaySomething(const std::string &name, const NodeConfig &config)
            : SyncActionNode(name, config) {}

        // It is mandatory to define this STATIC method.
        static PortsList providedPorts() {
            // This action has a single input port called "message"
            return {InputPort<std::string>("message")};
        }

        // Override the virtual function tick()
        NodeStatus tick() override {
            Expected<std::string> msg = getInput<std::string>("message");
            // Check if expected is valid. If not, throw its error
            if (!msg) {
                throw BT::RuntimeError("missing required input [message]: ",
                                       msg.error());
            }
            // use the method value() to extract the valid message.
            std::cout << "Robot says: " << msg.value() << std::endl;
            return NodeStatus::SUCCESS;
        }
};

class ThinkWhatToSay : public SyncActionNode {
    public:
        ThinkWhatToSay(const std::string &name, const NodeConfig &config)
            : SyncActionNode(name, config) {}

        static PortsList providedPorts() {
            return {OutputPort<std::string>("text")};
        }

        // This Action writes a value into the port "text"
        NodeStatus tick() override {
            // the output may change at each tick(). Here we keep it simple.
            setOutput("text", "The answer is 42");
            return NodeStatus::SUCCESS;
        }
};

int main() {
    BehaviorTreeFactory factory;
    factory.registerNodeType<SaySomething>("SaySomething");
    factory.registerNodeType<ThinkWhatToSay>("ThinkWhatToSay");

    auto tree = factory.createTreeFromFile(ament_index_cpp::get_package_share_directory("aljnuho_bt") + "/config/" + "bt_io.xml");
    tree.tickWhileRunning();
    return 0;
}

/*  Expected output:
  Robot says: hello
  Robot says: The answer is 42
*/