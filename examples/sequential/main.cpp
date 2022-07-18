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

// keyframe planner include
#include <path/sktp.h>

#define PI 3.14159
#define tol 1e-2

namespace ob = ompl::base;
namespace og = ompl::geometric;

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

std::vector<std::string> getInitInfo(int argc, char **argv)
{
    // Default inputs
    std::string inputFile = "../examples/Main/manipulationSequence.txt";
    
    if(argc > 1)
    {
        inputFile = argv[1];
    }

    // Read file to get inputs
    std::string line;
    ifstream myfile(inputFile);
    std::vector<std::string> inputs;
    if (myfile.is_open())
    {
        while (getline(myfile, line))
        {
            inputs.push_back(line);
        }
        myfile.close();
    }
    else
    {
        cout << "Unable to open file";
        return Null_vector;
    }

    return inputs;
}

int main(int argc, char **argv)
{
    // get inputs as a vector of strings
    std::vector<std::string> inputs = getInitInfo(argc, argv);
    if (inputs == Null_vector)
        return 0;

    // setup the outer planner
    rai::Configuration C(inputs.at(0).c_str());
	ob::SpaceInformationPtr si(std::make_shared<ob::SpaceInformation>(std::make_shared<ob::RealVectorStateSpace>(C.getJointStateDimension())));
    auto planner = std::make_shared<og::sktp>(si);
    planner->set_inputs(inputs);
    planner->set_subPlanner(og::sktp::BITstar);
    planner->set_branchingFactor(3);

    // attempt to solve the problem
    ob::PlannerStatus solved;
    solved = planner->ob::Planner::solve(30.0);

    return 0;
}