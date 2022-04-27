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

arrA solveMotion(rai::Configuration &C, arr goal_)
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
    auto planner(std::make_shared<og::BITstar>(si));
	ss.setPlanner(planner);

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

// void visualizeSequence(std::string filename, std::vector<arrA> &trajectories)
// {
//     rai::Configuration C;
//     C.addFile(filename.c_str());
//     KOMO komo;
//     komo.verbose = 0;
//     komo.setModel(C, true);
    
//     arrA configs = trajectories.at(0);
//     komo.setTiming(2, configs.N, 5., 2);
// 	komo.add_qControlObjective({}, 1, 1.);

//     //use configs to initialize with waypoints
// 	komo.initWithWaypoints(configs, configs.N, false);
//     komo.run_prepare(0);
// 	komo.plotTrajectory();

// 	rai::ConfigurationViewer V;
// 	V.setPath(C, komo.x, "result", true);
//     V.playVideo();
// }

void sequentialPlan(std::string filename = "../examples/Models/s1_2d_manip.g")
{
    rai::Configuration C;
    C.addFile(filename.c_str());

    // visualize environment
    C.watch(true);

    std::string ref1 = "endeff", ref2 = "object";
    arr pickConfig = getTargetConfig(C, ref1, ref2);
    std::cout << "pickConfig: " << pickConfig << std::endl;

    // checkCollision(C);
    arrA pickTrajectory = solveMotion(C, pickConfig);
    visualizePath(C, pickTrajectory);

    // set the joint state for the keyframe, and attach the frame
    C.setJointState(pickTrajectory.last());
    C.attach(C.getFrame(ref1.c_str()), C.getFrame(ref2.c_str()));
    C.watch(true);

    ref1 = "object", ref2 = "target";
    arr placeConfig = getTargetConfig(C, ref1, ref2);
    std::cout << "placeConfig: " << placeConfig << std::endl;

    // checkCollision(C);
    arrA placeTrajectory = solveMotion(C, placeConfig);
    visualizePath(C, placeTrajectory);
    C.setJointState(placeTrajectory.last());
    C.watch(true);

    C.attach(C.getFrame("world"), C.getFrame(ref1.c_str()));
    C.watch(true);
}

int main()
{
    sequentialPlan();
    // visualizeModel();
  
}