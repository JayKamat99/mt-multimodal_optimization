/**
 * @file sequential.cpp
 * @author Jay Kamat
 * @brief This code is for generating sequential motion plans for manipulation tasks
 * @version 0.1
 * @date 2022-04-26
 * 
 * @copyright Copyright (c) 2022
 * 
 */

/** 
 * Quim's Proposal
    Pick and Place:

    object: start ---> goal position + orientation


    Start Configuration of robot is fix

    Motion has 2 steps/modes/orbits

    use komo -> Robot configuration picking object

    use ompl to find path from robot start to robot picking (default collision checking, with another komo object)


    -----

    Result: path from start to pick keyframe, with a relative trasnformation object-gripper

    ---- 


    use komo -> Robot configuration place object (compatible with relative transformation we found)

    Implementation: attach the object to the robot with relative trasnformation.

    (possible without addswitch: first attach the object, second add objective "PositionDiff" between object and goal-position of object)

    use ompl to find path from robot picking to robot placing (new collision checking, with another komo object, that takes into account the relative transformation)
**/

// necessary
#include<fstream>

// ompl includes
#include <ompl/config.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
// planner
#include<ompl/geometric/planners/informedtrees/BITstar.h>

// komo includes
#include <KOMO/komo.h>
#include <Kin/viewer.h>

#include "debugFunctions.h"

#define PI 3.14159

namespace ob = ompl::base;
namespace og = ompl::geometric;

int C_Dimension;

struct ValidityCheckWithKOMO {
	KOMO::Conv_KOMO_SparseNonfactored &nlp;
	ValidityCheckWithKOMO(KOMO::Conv_KOMO_SparseNonfactored &nlp) : nlp(nlp){}
	bool check(const ob::State *state)
	{
		const auto *State = state->as<ob::RealVectorStateSpace::StateType>();

		arr x_query;
		for (unsigned int i = 0; i < C_Dimension; i++){
			x_query.append((*State)[i]);
		}

		arr phi;
		nlp.evaluate(phi, NoArr, x_query);
		double tol = 1e-2;

		return std::abs(phi(0)) < tol;
	}
};

arr getTargetConfig(rai::Configuration &C, std::string &ref1, std::string &ref2)
{
    KOMO komo;
    komo.setModel(C);
    komo.setTiming(1,1,1,1);
    komo.addObjective({1,1},FS_distance,{ref2.c_str(),ref1.c_str()}, OT_eq);
    komo.addObjective({1,1},FS_distance,{ref2.c_str(),ref1.c_str()}, OT_ineq);

    komo.optimize();
    // komo.view(true);

    return komo.x;
}

arrA solveMotion(rai::Configuration &C, arr goal_, std::string planner_ = "BITstar")
{
    KOMO komo;
	komo.setModel(C, true);
	komo.setTiming(1, 1, 1, 1);
	komo.addObjective({}, FS_accumulatedCollisions, {}, OT_eq, { 1 });
	komo.run_prepare(0);

	C_Dimension = C.getJointStateDimension();

	//Construct the state space we are planning in
	auto space(std::make_shared<ob::RealVectorStateSpace>(C_Dimension));

	ob::RealVectorBounds bounds(C_Dimension);
	bounds.setLow(-PI);
	bounds.setHigh(PI);
    space->setBounds(bounds);

	//create simple setup
	og::SimpleSetup ss(space);

    // set state validity checking for this space
	auto nlp = std::make_shared<KOMO::Conv_KOMO_SparseNonfactored>(komo, false);
	ValidityCheckWithKOMO checker(*nlp);

	ss.setStateValidityChecker([&checker](const ob::State *state) {
		return checker.check(state);
	});

	// create start and goal states. These states might change from example to example
    ob::ScopedState<> start(space);
	for (unsigned int i=0; i<C.getJointStateDimension(); i++){
	start[i] = komo.getConfiguration_q(0).elem(i);
	}

	ob::ScopedState<> goal(space);
    for (unsigned int i=0; i<C.getJointStateDimension(); i++){
	goal[i] = goal_(i);
	}

    ss.setStartAndGoalStates(start, goal);
	auto si = ss.getSpaceInformation();
    if (planner_ == "BITstar")
    {
        auto planner(std::make_shared<og::BITstar>(si));
	    ss.setPlanner(planner);
    }

    ss.setup();

    // attempt to solve the problem
    ob::PlannerStatus solved = ss.solve(10.0);

    if (solved == ob::PlannerStatus::StatusType::APPROXIMATE_SOLUTION)
        std::cout << "Found solution: APPROXIMATE_SOLUTION" << std::endl;
    else if (solved == ob::PlannerStatus::StatusType::EXACT_SOLUTION)
        std::cout << "Found solution: EXACT_SOLUTION" << std::endl;
    else if (solved == ob::PlannerStatus::StatusType::TIMEOUT)
        std::cout << "Found solution: TIMEOUT" << std::endl;
    else
        std::cout << "No solution found: Invalid " << std::endl;

    auto path = ss.getSolutionPath();
    path.interpolate(15);

    arrA configs;
    for (auto state : path.getStates())
    {
        arr config;
        std::vector<double> reals;
        space->copyToReals(reals, state);
        for (double r : reals){
            config.append(r);
        }
        configs.append(config);
    }

    std::cout << "Solution Path" << configs << std::endl;
    return configs;
}

void visualizePath(rai::Configuration &C, arrA configs){
    KOMO komo;
    komo.verbose = 0;
    komo.setModel(C, true);
    
    komo.setTiming(1., configs.N, 5., 2);
	komo.add_qControlObjective({}, 1, 1.);

    //use configs to initialize with waypoints
	komo.initWithWaypoints(configs, configs.N, false);
    komo.run_prepare(0);
	komo.plotTrajectory();

	rai::ConfigurationViewer V;
	V.setPath(C, komo.x, "result", true);
    V.playVideo();
}


int main(int argc, char ** argv)
{
    // sequentialPlan();
    // return 0;
    // rai::initCmdLine(argc,argv);
    // std::cout << "OMPL version: " << OMPL_VERSION << std::endl;

    // Default inputs
    std::string inputFile = "../examples/Main/manipulationSequence.txt";
    std::string planner_ = "BITstar";

    // Don't like the default inputs? You can input the inputs you want!
    switch(argc) {
        case 3:
        planner_ = argv[2];
        case 2:
        inputFile = argv[1];
    }

    // Read file to get inputs
    std::string line;
    ifstream myfile (inputFile);
    std::vector<std::string> inputs;
    if (myfile.is_open())
    {
        while ( getline (myfile,line) )
        {
            inputs.push_back(line);
        }
        myfile.close();
    }
    else
    {
        cout << "Unable to open file";
        return 0;
    }

    // Set filename (Model) and totalPhases
    std::string filename = inputs.at(0);
    int totalPhases = stoi(inputs.at(1));

    // Set Configuration
    rai::Configuration C;
    C.addFile(filename.c_str());

    for (int phase = 0; phase < totalPhases; ++phase)
    {
        std::string ref1 = inputs.at(2+phase*2), ref2 = inputs.at(3+phase*2);
        arr goalConfig = getTargetConfig(C, ref1, ref2);
        std::cout << "goalConfig[" << phase << "]: " << goalConfig << std::endl;
        arrA Trajectory = solveMotion(C, goalConfig, planner_);
        visualizePath(C, Trajectory);
        C.setJointState(Trajectory.last());
        if(phase%2 == 0)
            C.attach(C.getFrame(ref1.c_str()), C.getFrame(ref2.c_str())); //pick
        else
            C.attach(C.getFrame("world"), C.getFrame(ref1.c_str())); //place
    }
    C.setJointState({0,0,0});
    C.watch(true);
}