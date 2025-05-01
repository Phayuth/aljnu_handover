#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_cpp/bt_factory.h>

class CrossDoor {
    public:
        void registerNodes(BT::BehaviorTreeFactory &factory);

        // SUCCESS if _door_open != true
        BT::NodeStatus isDoorClosed() {
            if (_door_open != true) {
                return BT::NodeStatus::SUCCESS;
            }
            return BT::NodeStatus::FAILURE;
        };

        // SUCCESS if _door_open == true
        BT::NodeStatus passThroughDoor() {
            if (_door_open == true) {
                return BT::NodeStatus::SUCCESS;
            }
            return BT::NodeStatus::FAILURE;
        };

        // After 3 attempts, will open a locked door
        BT::NodeStatus pickLock() {
            if (_pick_attempts < 3) {
                _pick_attempts += 1;
                return BT::NodeStatus::FAILURE;
            } else {
                _pick_attempts += 1;
                _door_open = true;
                _door_locked = false;
                return BT::NodeStatus::SUCCESS;
            }
        };

        // FAILURE if door locked
        BT::NodeStatus openDoor() {
            if (_door_locked == true) {
                return BT::NodeStatus::FAILURE;
            }
            return BT::NodeStatus::SUCCESS;
        };

        // WILL always open a door
        BT::NodeStatus smashDoor() {
            _door_open = true;
            _door_locked = false;
            return BT::NodeStatus::SUCCESS;
        };

    private:
        bool _door_open = false;
        bool _door_locked = true;
        int _pick_attempts = 0;
};

// Helper method to make registering less painful for the user
void CrossDoor::registerNodes(BT::BehaviorTreeFactory &factory) {
    factory.registerSimpleCondition(
        "IsDoorClosed", std::bind(&CrossDoor::isDoorClosed, this));

    factory.registerSimpleAction(
        "PassThroughDoor", std::bind(&CrossDoor::passThroughDoor, this));

    factory.registerSimpleAction(
        "OpenDoor", std::bind(&CrossDoor::openDoor, this));

    factory.registerSimpleAction(
        "PickLock", std::bind(&CrossDoor::pickLock, this));

    factory.registerSimpleCondition(
        "SmashDoor", std::bind(&CrossDoor::smashDoor, this));
}

int main() {
    BT::BehaviorTreeFactory factory;

    CrossDoor cross_door;
    cross_door.registerNodes(factory);

    // In this example a single XML contains multiple <BehaviorTree>
    // To determine which one is the "main one", we should first register
    // the XML and then allocate a specific tree, using its ID

    factory.registerBehaviorTreeFromFile(ament_index_cpp::get_package_share_directory("aljnuho_bt") + "/config/" + "bt_subtree.xml");
    auto tree = factory.createTree("MainTree");

    // helper function to print the tree
    printTreeRecursively(tree.rootNode());

    tree.tickWhileRunning();

    return 0;
}
