#pragma once

// necessary
#include <fstream>

// komo includes
#include <KOMO/komo.h>
#include <Kin/viewer.h>

// ompl includes
#include <ompl/config.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/goals/GoalStates.h>
#include <ompl/base/Planner.h>

#define PI 3.14159
#define tol 1e-2

namespace ob = ompl::base;
namespace og = ompl::geometric;

int C_Dimension;
const arrA Null_arrA = {{}};
const std::vector<std::string> Null_vector = {{}};

template <typename T>
void printVector(T &vec)
{
    for (const auto &i : vec)
    {
        std::cout << i << std::endl;
    }
    std::cout << '\n';
}

enum transition
{
    pick,
    place
};

class keyframeNode
{
private:
    arr state;
    std::shared_ptr<keyframeNode> parent;
    double costHeuristic; // lower-bound
    double bestCostToCome;
    double bestCostToGo; // cost to reach the final goal.
    double calcDistHeuristic(); // calculates eucledian distance from parent and adds it to the best cost to parent if available, else to the dist heuristic to the parent
    double distFromParent();
    std::shared_ptr<ompl::base::Planner> planner;
public:
    keyframeNode(arr state, std::shared_ptr<keyframeNode> parent);
    ~keyframeNode() = default;
    arr get_state() {return this->state;}
    std::shared_ptr<keyframeNode> get_parent() {return this->parent;}
    double get_costHeuristic()  {return this->costHeuristic;}
    double get_bestCostToCome() {return this->bestCostToCome;}
};

keyframeNode::keyframeNode(arr state, std::shared_ptr<keyframeNode> parent)
{
    this->state = state;
    this->parent = parent;
    if(parent == nullptr)
        this->bestCostToCome = 0;
    else
        this->bestCostToCome = INFINITY;
    this->costHeuristic = calcDistHeuristic();
}

double keyframeNode::distFromParent()
{
    arr childState = this->state;
    arr parentState = this->parent->state;
    return euclideanDistance(childState.resize(C_Dimension), parentState.resize(C_Dimension));
}

double keyframeNode::calcDistHeuristic()
{
    if (this->parent == nullptr)
        return 0;
    if (this->parent->get_bestCostToCome() != INFINITY)
    {
        return (this->parent->get_bestCostToCome() + this->distFromParent());
    }
    else 
    {
        return (this->parent->get_costHeuristic() + this->distFromParent());
    }
}


// Functions used in the algorithm

std::shared_ptr<keyframeNode> makeRootNode(std::vector<std::string> inputs);

void growTree(std::vector<std::string> &inputs, std::shared_ptr <keyframeNode> start);
arrA sampleKeyframeSequence(std::vector<std::string> inputs, std::shared_ptr <keyframeNode> start);
void addToTree(arrA sequence, std::shared_ptr<keyframeNode> start);
int getCurrentPhase(std::shared_ptr<keyframeNode> start);

arrA planMotion(std::vector<std::string> &inputs, arrA keyFrames);

arrA getSequence(std::shared_ptr<keyframeNode> node_ptr);


