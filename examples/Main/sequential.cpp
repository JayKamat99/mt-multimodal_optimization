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
#include <fstream>

// ompl includes
#include <ompl/config.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/goals/GoalStates.h>
// planner
#include <ompl/geometric/planners/informedtrees/BITstar.h>

// komo includes
#include <KOMO/komo.h>
#include <Kin/viewer.h>

#define PI 3.14159
#define tol 1e-2

namespace ob = ompl::base;
namespace og = ompl::geometric;

int C_Dimension;
const arrA Null_arrA = {{}};

enum transition
{
    pick,
    place
};

struct ValidityCheckWithKOMO
{
    KOMO::Conv_KOMO_SparseNonfactored &nlp;
    ValidityCheckWithKOMO(KOMO::Conv_KOMO_SparseNonfactored &nlp) : nlp(nlp) {}
    bool check(const ob::State *state)
    {
        const auto *State = state->as<ob::RealVectorStateSpace::StateType>();

        arr x_query;
        for (unsigned int i = 0; i < C_Dimension; i++)
        {
            x_query.append((*State)[i]);
        }

        arr phi;
        nlp.evaluate(phi, NoArr, x_query);

        return std::abs(phi(0)) < tol;
    }
};

arrA solveMotion(rai::Configuration &C, std::vector<arr> goal_, std::string planner_ = "BITstar")
{
    KOMO komo;
	komo.setModel(C, true);
	komo.setTiming(1, 1, 1, 1);
	komo.addObjective({}, FS_accumulatedCollisions, {}, OT_eq, { 1 });
	komo.run_prepare(2);

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
	for (unsigned int i = 0; i < C.getJointStateDimension(); i++)
    {
	start[i] = komo.getConfiguration_q(0).elem(i);
	}
    pdef->addStartState(start);

    auto goalStates = std::make_shared<ob::GoalStates>(si);

    for (unsigned int j = 0; j < goal_.size(); j++)
    {
        ob::ScopedState<> goal(space);
        for (unsigned int i = 0; i < C.getJointStateDimension(); i++)
        {
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
    solved = planner->ob::Planner::solve(5.0);

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
        for (double r : reals)
        {
            config.append(r);
        }
        configs.append(config);
    }

    std::cout << "Solution Path" << configs << std::endl;
    return configs;
}

void visualizePath(rai::Configuration &C, arrA configs)
{
    KOMO komo;
    komo.verbose = 0;
    komo.setModel(C, true);

    komo.setTiming(1., configs.N, 5., 2);
    komo.add_qControlObjective({}, 1, 1.);

    // use configs to initialize with waypoints
    komo.initWithWaypoints(configs, configs.N, false);
    komo.run_prepare(0);
    komo.plotTrajectory();

    rai::ConfigurationViewer V;
    V.setPath(C, komo.x, "result", true);
    V.playVideo();
}

template <typename T>
void printVector(T &vec)
{
    for (const auto &i : vec)
    {
        std::cout << i << std::endl;
    }
    std::cout << '\n';
}

arrA sampleKeyFrames(std::vector<std::string> inputs)
{
    arrA keyFrames;
    std::string filename = inputs.at(0);
    int totalPhases = stoi(inputs.at(1));

    bool keyframesValid = false;

    // Set Configuration
    rai::Configuration C;
    C.addFile(filename.c_str());
    C_Dimension = C.getJointStateDimension();

    while(!keyframesValid)
    {
        KOMO komo;
        komo.verbose = 0;
        komo.setModel(C, true);
        komo.setTiming(totalPhases, 1, 5, 2);

        int phase = 0;
        while (phase < totalPhases)
        {
            std::string ref1 = inputs.at(2 + phase * 2), ref2 = inputs.at(3 + phase * 2);

            if (phase == 0)
                komo.addSwitch_stable(1, 2., "", ref1.c_str(), ref2.c_str());
            else if (phase % 2 == 0)
                komo.addSwitch_stable(phase + 1, phase + 2., "", ref1.c_str(), ref2.c_str(), false);
            else
                komo.addSwitch_stable(phase + 1, phase + 2., "", ref2.c_str(), ref1.c_str(), false);
            komo.addObjective({phase + 1.}, FS_distance, {ref1.c_str(), ref2.c_str()}, OT_eq, {1e2});
            komo.addObjective({phase + 1.}, FS_vectorZ, {ref1.c_str()}, OT_eq, {1e2}, {0., 0., 1.});

            std::cout << "ref1,ref2: " << ref1 << ref2 << std::endl;

            if (phase % 2 == 0) // pick
            {
                std::cout << "pick" << std::endl;
                komo.addObjective({phase + 1.}, FS_scalarProductXX, {ref1.c_str(), ref2.c_str()}, OT_eq, {1e2}, {0.});
            }
            else // place
            {
                std::cout << "place" << std::endl;
                komo.addObjective({phase + 1.}, FS_aboveBox, {ref2.c_str(), ref1.c_str()}, OT_ineq, {1e2});
            }
            phase++;
        }

        komo.add_qControlObjective({}, 1);
        komo.add_collision(true, 0.01);

        komo.run_prepare(0);
        komo.optimize();
        keyFrames = komo.getPath_q();
        komo.view(true);
        komo.view_play(true);

        rai::Graph R = komo.getReport(false);
        double constraint_violation = R.get<double>("eq") + R.get<double>("ineq");
        std::cout << constraint_violation << std::endl;

        if (constraint_violation < 1){
            keyframesValid = true;
        }
    }
    return keyFrames;
}

arrA planMotion(std::vector<std::string> &inputs, arrA keyFrames)
{
    arrA finalPath;

    std::string filename = inputs.at(0);
    int totalPhases = stoi(inputs.at(1));

    // Set Configuration
    rai::Configuration C;
    C.addFile(filename.c_str());

    int phase = 0;
    while (phase < totalPhases)
    {
        std::cout << "phase: " << phase << std::endl;
        std::string ref1 = inputs.at(2 + phase * 2), ref2 = inputs.at(3 + phase * 2);

        transition transition_{pick};
        if (phase % 2 != 0)
        {
            transition_ = place;
        }

        std::vector<arr> goalConfigs;
        arr goalKeyFrame = keyFrames(phase);
        std::cout << goalKeyFrame << std::endl;
        goalConfigs.push_back(goalKeyFrame.resize(C_Dimension));
        // goalConfigs.push_back({0.0421136, 0.764693, 0.246095, -1.73399, -0.272772, 2.4651, -0.872665});
        arrA intermediatePath = solveMotion(C, goalConfigs);
        if (intermediatePath == Null_arrA)
        {
            OMPL_ERROR("Can't find a solution!");
        }
        finalPath.append(intermediatePath);
        // std::cout << C.getJointState() << std::endl;
        // C.setJointState(Trajectory.last());
        if (phase % 2 == 0)
            C.attach(C.getFrame(ref1.c_str()), C.getFrame(ref2.c_str())); // pick
        else
            C.attach(C.getFrame("world"), C.getFrame(ref1.c_str())); // place
        phase++;
    }
    return finalPath;
}

void optimizePath(std::vector<std::string> &inputs, arrA &finalPath)
{
    // static int attempt = 0;
    // Set filename (Model) and totalPhases
    std::string filename = inputs.at(0);
    int totalPhases = stoi(inputs.at(1));

    // Set Configuration
    rai::Configuration C;
    C.addFile(filename.c_str());

    // make a KOMO object and write down the whole action sequence
    KOMO komo;
    komo.verbose = 0;
    komo.setModel(C, true);
    komo.setTiming(totalPhases, 15, 5, 2);

    // Define the KOMO problem by iterating over action the sequence
    int phase = 0;
    while (phase < totalPhases)
    {
        std::string ref1 = inputs.at(2 + phase * 2), ref2 = inputs.at(3 + phase * 2);

        if (phase == 0)
            komo.addSwitch_stable(phase + 1, phase + 2., "", ref1.c_str(), ref2.c_str());
        else if (phase % 2 == 0)
            komo.addSwitch_stable(phase + 1, phase + 2., "", ref1.c_str(), ref2.c_str(), false);
        else
            komo.addSwitch_stable(phase + 1, phase + 2., "", ref2.c_str(), ref1.c_str(), false);
        komo.addObjective({phase + 1.}, FS_distance, {ref1.c_str(), ref2.c_str()}, OT_eq, {1e2});
        komo.addObjective({phase + 1.}, FS_vectorZ, {ref1.c_str()}, OT_eq, {1e2}, {0., 0., 1.});

        if (phase % 2 == 0) // pick
        {
            std::cout << "pick" << std::endl;
            komo.addObjective({phase + 1.}, FS_scalarProductXX, {ref1.c_str(), ref2.c_str()}, OT_eq, {1e2}, {0.});
        }
        else // place
        {
            std::cout << "place" << std::endl;
            komo.addObjective({phase + 1.}, FS_aboveBox, {ref2.c_str(), ref1.c_str()}, OT_ineq, {1e2});
        }
        phase++;
    }

    komo.add_qControlObjective({}, 2);
    komo.add_collision(true, 0.01);

    komo.run_prepare(0);
    // if(attempt > 0)
    komo.initWithWaypoints(finalPath, 15, false);

    komo.view(true, "pre-optimization motion");
    for (uint i = 0; i < 2; i++)
        komo.view_play(true);
    
    // komo.animateOptimization = 2;
    komo.optimize();
    // komo.checkGradients();
    // arrA solution = komo.getPath_q();
    // finalPath = solution;
    // std::cout << solution << std::endl;
    // attempt++;

    komo.view(true, "optimized motion");
    for (uint i = 0; i < 2; i++)
        komo.view_play(true);
}

void storePath(std::vector<arrA> &subtrajectories)
{
    ofstream myfile("../examples/Main/FeasiblePath.txt");
    if (myfile.is_open())
    {
        for (const auto &i : subtrajectories)
        {
            myfile << i << "\n";
        }
        myfile.close();
    }
    else
        cout << "Unable to open file";
}

void copyPath(arrA &finalPath)
{
    ifstream myfile("../examples/Main/FeasiblePath.txt");
    std::string line;
    if (myfile.is_open())
    {
        for (int i = 0; i < 2; i++)
        { // keep reading until end-of-file
            // cout << "The next number is " << num << endl;
            arrA temp;
            myfile >> temp; // sets EOF flag if no value found
            finalPath.append(temp);
            // std::cout << temp << "\n\n";
        }
        myfile.close();
    }
    else
        cout << "Unable to open file";
}

void copyPath(std::vector<arrA> &subtrajectories, arrA &finalPath)
{
    for (int i=0; i<subtrajectories.size(); i++)
        finalPath.append(subtrajectories.at(i));
}

void modifyPath(arrA &finalPath, arrA keyFrames)
{
    std::cout << "I enter modify Path" << std::endl;
    std::cout << "final Path length : " << finalPath.N << std::endl;
    int phase = 1;
    while (phase < keyFrames.N)
    {
        arr keyframe = keyFrames(phase-1);
        std::cout << "keyframe" << keyframe << std::endl;
        arr relTransformation = keyframe({C_Dimension, keyframe.N-1});
        std::cout << "relTransformation" << relTransformation << std::endl;
        for (int i = 0; i < 15; i++)
        {
            finalPath(15*phase+i-1).append(relTransformation);
        }
        phase ++;
    }
    finalPath(15*keyFrames.N-1) = keyFrames(keyFrames.N-1);
    std::cout << "finalPath: " << finalPath << std::endl;
}

#include "debugFunctions.h"

int main(int argc, char **argv)
{
    // sequentialPlan();
    // runOnlyKOMO();
    // visualizeModel((std::string)argv[1]);
    // return 0;

    // Default inputs
    std::string inputFile = "../examples/Main/manipulationSequence.txt";
    std::string planner_ = "BITstar";

    // Don't like the default inputs? You can input the inputs you want!
    switch (argc)
    {
    case 3:
        planner_ = argv[2];
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
        return 0;
    }

    // Get sampled path
    std::vector<arrA> subTrajectories;
    arrA keyFrames = sampleKeyFrames(inputs);
    // arrA relTransformation = getRelTransformations(keyFrames);
    std::cout << keyFrames << std::endl;
    arrA finalPath = planMotion(inputs, keyFrames);

    // //Stores path in a file
    // storePath(subTrajectories);

    // // These set of functions are to be called if you want to only debug the optimization.
    // copyPath(finalPath);

    copyPath(subTrajectories,finalPath);
    modifyPath(finalPath, keyFrames);
    std::cout << finalPath << std::endl;

    optimizePath(inputs, finalPath);
}