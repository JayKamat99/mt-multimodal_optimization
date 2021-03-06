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
#define tol 1e-2

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

		return std::abs(phi(0)) < tol;
	}
};

arr getGoalConfig(rai::Configuration C, std::string &ref1, std::string &ref2, transition transition_ = pick)
{

    arr goalConfig;
    bool feasible = false;
    while (!feasible)
    {
        KOMO komo;
        komo.verbose = 0;
        komo.setModel(C,true);
        arr tergetReference;

        komo.setTiming(1,1,1,1);
        komo.addObjective({}, FS_accumulatedCollisions, {}, OT_eq, { 1 });
        komo.add_collision(true, 0.01);
        arr pos_ref2 = C.getFrameState(C.getFrameIDs({(rai::String)ref2})).resize(3);
        std::cout << "pos_ref2: " << pos_ref2 << "\nCSpace_dim: " << C.getJointStateDimension() << std::endl; 
        arr randomConfig_ = pos_ref2.resize(C.getJointStateDimension()) - 0.5 + 2*0.5*rand(C.getJointStateDimension());
        komo.initWithConstant(randomConfig_);
        std::cout << komo.getConfiguration_q(0)<< std::endl;
        std::cout << "ref1, ref2: " << ref2.c_str() <<  ", " <<ref1.c_str() << std::endl;
        komo.addObjective({1.}, FS_distance, {ref1.c_str(),ref2.c_str()}, OT_eq, {1e2});
        komo.addObjective({1.}, FS_vectorZ, {ref1.c_str()}, OT_eq, {1e2}, {0., 0., 1.});

        if(transition_ == pick)
        {
            komo.addObjective({1.}, FS_scalarProductXX, {ref1.c_str(),ref2.c_str()}, OT_eq, {1e2}, {0.});
        }
        else if (transition_ == place)
        {
            std::cout << "I have been asked to place" << std::endl;
            komo.addObjective({1.}, FS_aboveBox, {ref2.c_str(),ref1.c_str()}, OT_ineq, {1e2});
        }

        // komo.addObjective({1,1},FS_distance,{ref2.c_str(),ref1.c_str()}, OT_eq, {}, {0,0,0});
        komo.run_prepare(0);
        // komo.animateOptimization = 2;
        komo.optimize();
        // komo.view(true);
        // std::cout << komo.x << std::endl;
        // Use a feasibility checker here for now. Later check if you can use the same feasibility checker as the planner
        auto nlp = std::make_shared<KOMO::Conv_KOMO_SparseNonfactored>(komo, false);
        ObjectiveTypeA ot;
        nlp->getFeatureTypes(ot);
        arr phi;
        nlp->evaluate(phi, NoArr, komo.x);
        std::cout << komo.x << " " << phi(0) << std::endl;
        if (phi(0) < tol)
            feasible = true;
        else
        {
            feasible = false;
            continue;
        }
        goalConfig.append(komo.x);
        std::cout << "Goal Config: " << komo.x << komo.x.N << std::endl;
    }
    
    // komo.view(true);
    return goalConfig;
}

std::vector<arr> getGoalConfigsSet(rai::Configuration C, std::string &ref1, std::string &ref2, transition transition_)
{
    static int mode = 1;
    std::vector<arr> goalConfigs;
    switch(mode)
    {
        // case(1):
        //     goalConfigs = {{1.0498, 0.0911573, 0.868818},{0.689771, 0.229342, 1.66093},/* {0.350224, 0.0911714, 2.27218} */};
        //     break;
        default:
            for(int i = 0; i< 10; i++)
            {
                goalConfigs.push_back(getGoalConfig(C,ref1,ref2,transition_));
            }
    }
    mode ++;
    return goalConfigs;
}

arrA solveMotion(rai::Configuration &C, std::vector<arr> goal_, std::string planner_ = "BITstar")
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

    for (unsigned int j=0; j<goal_.size(); j++)
    {
        ob::ScopedState<> goal(space);
        for (unsigned int i=0; i<C.getJointStateDimension(); i++){
        goal[i] = goal_.at(j)(i);
        }
        goalStates->addState(goal);
    }

    pdef->setGoal(goalStates);
    auto BITStar_Planner = std::make_shared<og::BITstar>(si);
    auto planner(BITStar_Planner);
    planner->setProblemDefinition(pdef);
    planner->setup();

    // attempt to solve the problem
    ob::PlannerStatus solved;
    solved = planner->ob::Planner::solve(10.0);

    if (solved == ob::PlannerStatus::StatusType::APPROXIMATE_SOLUTION)
        std::cout << "Found solution: APPROXIMATE_SOLUTION" << std::endl;
    else if (solved == ob::PlannerStatus::StatusType::EXACT_SOLUTION)
        std::cout << "Found solution: EXACT_SOLUTION" << std::endl;
    else if (solved == ob::PlannerStatus::StatusType::TIMEOUT)
    {
        std::cout << "Found solution: TIMEOUT" << std::endl;
        return Null_arrA;
    }
    else
    {
        std::cout << "No solution found: Invalid " << std::endl;
        return Null_arrA;
    }

    auto path = static_cast<og::PathGeometric &>(*(pdef->getSolutionPath()));
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

#include "debugFunctions.h"

void Print(const std::vector<arr>& vec) {
  for (const auto& i : vec) {
    std::cout << i << ' ';
  }
  std::cout << '\n';
}

int main(int argc, char ** argv)
{
    // sequentialPlan();
    // runOnlyKOMO();
    // visualizeModel((std::string)argv[1]);
    // return 0;

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
    std::cout << "C_Dim = " << C.getJointStateDimension() << std::endl;

    arrA previousTrajectory;
    std::vector<arr> previousGoals;
    arrA Trajectory;
    arr C_prev = C.getJointState();

    // Loop for iterating over task sequences.
    int phase = 0;
    while (phase < totalPhases)
    {
        std::cout << "phase: " << phase << std::endl;
        std::string ref1 = inputs.at(2+phase*2), ref2 = inputs.at(3+phase*2);
        // for (int i=0; i<50; ++i)
        // {
        //     sampleGoalsandDisplay(C, ref1, ref2);
        // }

        transition transition_{pick};
        if(phase%2 != 0)
        {
            transition_ = place;
        }
        std::cout << "Transition: " << transition_ << std::endl;
        std::vector<arr> goalConfigs = getGoalConfigsSet(C, ref1, ref2, transition_); //inverse kin
        // std::cout << "goalConfigs[" << phase << "]: " << goalConfigs << std::endl;
        Print(goalConfigs);
        bool flag = true;
        while(flag)
        {
            flag = false;
            std::cout << C.getJointState() << std::endl;
            Trajectory = solveMotion(C, goalConfigs, planner_);
            std::cout << C_prev << std::endl;
            if (Trajectory == Null_arrA)
            {
                flag = true;
                if (phase == 0) // If this is the first run. I am sorry we cannot find a solution
                {
                    OMPL_ERROR("Can't find a solution!");
                    return 0;
                }
                /** Ahh, our previous solution might have been the problem. 
                 * We need to revert to the phase and plan again without the currently used goal.
                **/

                // Remove the goal that did not work!
                arr redGoal = C.getJointState();
                previousGoals.erase(std::remove(previousGoals.begin(), previousGoals.end(), redGoal), previousGoals.end());
                Print(previousGoals);
                goalConfigs = previousGoals;

                // Revert configuration
                phase --;
                ref1 = inputs.at(2+phase*2), ref2 = inputs.at(3+phase*2);
                C.attach(C.getFrame("world"), C.getFrame(ref2.c_str())); //revert pick action
                std::cout << C.getJointState() << "  " << C_prev << std::endl;
                C.setJointState(C_prev);
            }
        }
        C_prev = C.getJointState();
        previousGoals = goalConfigs;
        visualizePath(C, Trajectory);
        // std::cout << C.getJointState() << std::endl;
        // C.setJointState(Trajectory.last());
        if(phase%2 == 0)
            C.attach(C.getFrame(ref1.c_str()), C.getFrame(ref2.c_str())); //pick
        else
            C.attach(C.getFrame("world"), C.getFrame(ref1.c_str())); //place
        phase ++;
    }
}