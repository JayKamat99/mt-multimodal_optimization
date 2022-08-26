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
#include <ompl/tools/benchmark/Benchmark.h>

// keyframe planner include
#include <path/sktp.h>
#include <path/SequentialKOMO.h>

#define PI 3.14159
#define tol 1e-2

namespace ob = ompl::base;
namespace og = ompl::geometric;

const std::vector<std::string> Null_vector = {{}};
bool benchmark;

enum PLANNER
{
    sktp,
    SequentialKOMO,
    SequentialMMP,
    SequentialBITstar
};
PLANNER mainPlanner;


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
    
    switch(argc)
    {
    case 4:
        if (argv[3] == (std::string)"SequentialKOMO")
            mainPlanner = SequentialKOMO;
        else if (argv[3] == (std::string)"SequentialMMP")
            mainPlanner = SequentialMMP;
        else if (argv[3] == (std::string)"SequentialBITstar")
            mainPlanner = SequentialBITstar;
        else
            mainPlanner = sktp;

    case 3:
        if (argv[2] == (std::string)"true")
            benchmark = true;
        else
            benchmark = false;

    case 2:
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
    auto space = std::make_shared<ob::RealVectorStateSpace>(C.getJointStateDimension()); 
    space->setBounds(-PI,PI);
    og::SimpleSetup ss(space);
    ss.setStateValidityChecker([](const ob::State *state) {return true;});
    ob::ScopedState<> start(space); ob::ScopedState<> goal(space);
    ss.setStartAndGoalStates(start,goal);
    
    std::shared_ptr<ob::Planner> planner;
    if (mainPlanner == sktp)
    {
        auto sktp_planner = std::make_shared<og::sktp>(ss.getSpaceInformation());
        sktp_planner->setProblemDefinition(ss.getProblemDefinition());
        sktp_planner->set_inputs(inputs);
        sktp_planner->set_subPlanner(og::sktp::BITstar);
        sktp_planner->set_branchingFactor(3); // Every node will have 3 or less than 3 children.
        sktp_planner->set_maxConstraintViolationKOMO(1); // This value is dependent on the experiment and can be changed by the user. Write what values I am using for different experiments
        planner = sktp_planner;
    }
    else if (mainPlanner == SequentialKOMO)
    {
        auto komo_planner = std::make_shared<og::SequentialKOMO>(ss.getSpaceInformation());
        komo_planner->setProblemDefinition(ss.getProblemDefinition());
        komo_planner->set_inputs(inputs);
        komo_planner->set_maxConstraintViolationKOMO(1); // This value is dependent on the experiment and can be changed by the user. Write what values I am using for different experiments
        planner = komo_planner;
    }
    

    if(benchmark)
	{
		// First we create a benchmark class:
		ompl::tools::Benchmark b(ss);
        b.addPlanner(planner);

		ompl::tools::Benchmark::Request req;
		req.maxTime = 15.0;
		// req.maxMem = 100.0;
		req.runCount = 5;
		req.displayProgress = true;

		b.benchmark(req);
		
		// This will generate a .log file
		std::ostringstream oss;
		oss << "data/Sequential/Benchmarks/manipulationSequence/logs/benchmark_" << planner->getName() << ".log";
		b.saveResultsToFile(oss.str().c_str());
	}

    else    // attempt to solve the problem
    {
        ob::PlannerStatus solved;
        solved = planner->ob::Planner::solve(20.0);
    }

    return 0;
}