/**
 * @file sequentialSampling.cpp
 * @author Jay Kamat
 * @brief This code is trying to replicate the work by will vega-brown
 * @version 0.1
 * @date 2022-05-27
 * 
 * @copyright Copyright (c) 2022
 * 
**/

// necessary
#include<fstream>

// ompl includes
#include <ompl/config.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/goals/GoalStates.h>
// planner
#include<ompl/geometric/planners/informedtrees/BITstar.h>

// komo includes
#include <KOMO/komo.h>
#include <Kin/viewer.h>

#define PI 3.14159

namespace ob = ompl::base;
namespace og = ompl::geometric;

int C_Dimension;
const arrA Null_arrA = {{}};

enum transition {pick,place};

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



// We will have only one implicit graph to be used by our planner in all modes.

int main()
{
    // Simple ompl motion planner by using implicit graph from outside
    // rai::Configuration C("../examples/Models/s2_bugtrap.g");
    rai::Configuration C("../examples/Models/1_kuka_shelf.g");
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

	//create space information pointer
	ob::SpaceInformationPtr si(std::make_shared<ob::SpaceInformation>(space));

    // set state validity checking for this space
	auto nlp = std::make_shared<KOMO::Conv_KOMO_SparseNonfactored>(komo, false);
	ValidityCheckWithKOMO checker(*nlp);

	si->setStateValidityChecker([&checker](const ob::State *state) {
		return checker.check(state);
	});

    ob::ProblemDefinitionPtr pdef(std::make_shared<ob::ProblemDefinition>(si));

	// create start and goal states. These states might change from example to example
    ob::ScopedState<> start(space);
	for (unsigned int i=0; i<C.getJointStateDimension(); i++){
	start[i] = komo.getConfiguration_q(0).elem(i);
	}
    pdef->addStartState(start);

    auto goalStates = std::make_shared<ob::GoalStates>(si);

    // std::vector<arr> goal_ = {{1.0498, 0.0911573, 0.868818},{0.689771, 0.229342, 1.66093},{0.350224, 0.0911714, 2.27218}};
    std::vector<arr> goal_ =  {{0.560603, -1.05486, -1.71583, -1.68994, -0.051403, 0.266908, 0.000754904}};

    for (unsigned int j=0; j<goal_.size(); j++)
    {
        ob::ScopedState<> goal(space);
        for (unsigned int i=0; i<C.getJointStateDimension(); i++){
        goal[i] = goal_.at(j)(i);
        }
        goalStates->addState(goal);
    }

    pdef->setGoal(goalStates);

    // auto implicitGraph_(std::make_shared<og::BITstar::ImplicitGraph>([]() { return "MainGraph"; }));

    auto BITStar_Planner = std::make_shared<og::BITstar>(si);
    // BITStar_Planner->graphPtr_ = implicitGraph_;
    auto planner(BITStar_Planner);
    planner->setProblemDefinition(pdef);
    planner->setup();

    // attempt to solve the problem
    ob::PlannerStatus solved;

    for (int i = 0; i<2; ++i)
{

    solved = planner->ob::Planner::solve(10.0);

    if (solved == ob::PlannerStatus::StatusType::APPROXIMATE_SOLUTION)
        std::cout << "Found solution: APPROXIMATE_SOLUTION" << std::endl;
    else if (solved == ob::PlannerStatus::StatusType::EXACT_SOLUTION)
        std::cout << "Found solution: EXACT_SOLUTION" << std::endl;
    else if (solved == ob::PlannerStatus::StatusType::TIMEOUT)
    {
        std::cout << "Found solution: TIMEOUT" << std::endl;
        return 0;
    }
    else
    {
        std::cout << "No solution found: Invalid " << std::endl;
        return 0;
    }

    ob::PathPtr path = pdef->getSolutionPath();
    path->print(std::cout);
}
    return 0;

}