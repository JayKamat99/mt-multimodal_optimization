#define PI 3.14159
#define tol 1e-2

int C_Dimension;
const arrA Null_arrA = {{}};

enum transition
{
    pick,
    place
};

class keyframeNode
{
private:
public:
    arr state;
    std::shared_ptr<keyframeNode> parent;
    double cost_heuristic; // lower-bound
    double bestCostToCome;
    keyframeNode() = default;
    keyframeNode(arr state, std::shared_ptr<keyframeNode> parent);
    ~keyframeNode() = default;
    double dist(std::shared_ptr<keyframeNode> parent);
};

keyframeNode::keyframeNode(arr state, std::shared_ptr<keyframeNode> parent)
{
    this->state = state;
    this->parent = parent;
}

void growTree(std::vector<std::string> &inputs, std::shared_ptr <keyframeNode> start);