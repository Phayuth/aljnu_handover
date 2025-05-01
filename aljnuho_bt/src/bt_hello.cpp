#include <behaviortree_cpp/bt_factory.h>
#include <iostream>

class SayHello : public BT::SyncActionNode {
    public:
        SayHello(const std::string &name, const BT::NodeConfig &config)
            : BT::SyncActionNode(name, config) {}

        static BT::PortsList providedPorts() { return {}; }

        BT::NodeStatus tick() override {
            std::cout << "Hello, World!" << std::endl;
            return BT::NodeStatus::SUCCESS;
        }
};

class SayHi : public BT::SyncActionNode {
    public:
        SayHi(const std::string &name, const BT::NodeConfig &config) : BT::SyncActionNode(name, config) {}

        static BT::PortsList providedPorts() { return {}; }

        BT::NodeStatus tick() override {
            std::cout << "Say Hi Back!" << std::endl;
            return BT::NodeStatus::SUCCESS;
        }
};

int main() {
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<SayHello>("SayHello");
    factory.registerNodeType<SayHi>("SayHi");

    auto tree = factory.createTreeFromText(R"(
    <root main_tree_to_execute="MainTree" BTCPP_format="4">
        <BehaviorTree ID="MainTree">
            <Sequence>
                <SayHello/>
                <SayHi/>
            </Sequence>
        </BehaviorTree>
    </root>
    )");

    tree.tickOnce();

    return 0;
}
