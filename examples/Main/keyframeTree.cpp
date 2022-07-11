/**
 * todo
 * 1) A tree class
 * 2) Expanding from a random point
 * 3) Replicate the method
 * 
 */

#include "keyframeTree.h"

// planner
#include <ompl/geometric/planners/informedtrees/BITstar.h>

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

std::vector<std::shared_ptr<keyframeNode>> leafNodes; // replace this vector with priority queue

std::vector<std::string> getInitInfo(int argc, char **argv)
{
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
        return Null_vector;
    }

    return inputs;
}

arrA solveMotion(rai::Configuration &C, std::vector<arr> goal_)
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

std::shared_ptr<keyframeNode> makeRootNode(std::vector<std::string> inputs)
{

    std::string filename = inputs.at(0);

    // Get init Configuration
    rai::Configuration C;
    C.addFile(filename.c_str());
    C_Dimension = C.getJointStateDimension();

    // make a root node and return it!
    return std::make_shared<keyframeNode>(C.getJointState(),nullptr);
}

int getCurrentPhase(std::shared_ptr<keyframeNode> start)
{
    int phase = 0;
    auto node = start;
    while (node->get_parent() != nullptr)
    {
        node = node->get_parent();
        phase ++;
    }
    return phase;
}

arrA sampleKeyframeSequence(std::vector<std::string> inputs, std::shared_ptr <keyframeNode> start = nullptr)
{
    arrA keyFrames;
    std::string filename = inputs.at(0);
    int totalPhases = stoi(inputs.at(1));

    int currentPhase = getCurrentPhase(start);
    std::cout << "currentPhase: " << currentPhase << std::endl;

    bool keyframesValid = false;

    // Set Configuration
    rai::Configuration C;
    C.addFile(filename.c_str());

    while(!keyframesValid)
    {
        KOMO komo;
        komo.verbose = 0;
        komo.setModel(C, true);
        komo.setTiming(totalPhases, 1, 5, 2);
        komo.addObjective({(double)currentPhase}, FS_qItself, {}, OT_eq, {}, start->get_state().resize(C_Dimension));

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

            // std::cout << "ref1,ref2: " << ref1 << ref2 << std::endl;

            if (phase % 2 == 0) // pick
            {
                // std::cout << "pick" << std::endl;
                komo.addObjective({phase + 1.}, FS_scalarProductXX, {ref1.c_str(), ref2.c_str()}, OT_eq, {1e2}, {0.});
            }
            else // place
            {
                // std::cout << "place" << std::endl;
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
        // std::cout << constraint_violation << std::endl;

        if (constraint_violation < 1){
            keyframesValid = true;
        }
    }
    return keyFrames;
}

arrA planMotion(std::vector<std::string> &inputs, std::shared_ptr<keyframeNode> &goalKeyframe)
{
    arrA finalPath;

    std::string filename = inputs.at(0);
    int totalPhases = stoi(inputs.at(1));

    // Set Configuration
    rai::Configuration C;
    C.addFile(filename.c_str());

    arrA keyFrames = getSequence(goalKeyframe);

    int phase = 0;
    while (phase < totalPhases)
    {
        // std::cout << "phase: " << phase << std::endl;
        std::string ref1 = inputs.at(2 + phase * 2), ref2 = inputs.at(3 + phase * 2);

        transition transition_{pick};
        if (phase % 2 != 0)
        {
            transition_ = place;
        }

        std::vector<arr> goalConfigs;
        arr goalKeyFrame = keyFrames(phase+1);
        // std::cout << goalKeyFrame << std::endl;
        goalConfigs.push_back(goalKeyFrame.resize(C_Dimension));
        arrA intermediatePath = solveMotion(C, goalConfigs);
        if (intermediatePath == Null_arrA) // path cannot be found! Yo need to sample more points from the previous node
        {
            // Add penalty to the current node.
            // increment a counter.
            // 
            // Your grow tree function comes in here.
        }
        finalPath.append(intermediatePath);
        C.setJointState(goalKeyFrame);
        if (phase % 2 == 0)
            C.attach(C.getFrame(ref1.c_str()), C.getFrame(ref2.c_str())); // pick
        else
            C.attach(C.getFrame("world"), C.getFrame(ref1.c_str())); // place
        phase++;
    }
    return finalPath;
}

void addRelTransformations(arrA &finalPath, arrA keyFrames)
{
    std::cout << "final Path length : " << finalPath.N << std::endl;
    int phase = 1;
    while (phase < keyFrames.N-1)
    {
        arr keyframe = keyFrames(phase);
        std::cout << "keyframe" << keyframe << std::endl;
        arr relTransformation = keyframe({C_Dimension, keyframe.N-1});
        std::cout << "relTransformation" << relTransformation << std::endl;
        for (int i = 0; i < 15; i++)
        {
            finalPath(15*phase+i-1).append(relTransformation);
        }
        phase ++;
    }
    finalPath(15*(keyFrames.N-1)-1) = keyFrames(keyFrames.N-1);
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
            // std::cout << "pick" << std::endl;
            komo.addObjective({phase + 1.}, FS_scalarProductXX, {ref1.c_str(), ref2.c_str()}, OT_eq, {1e2}, {0.});
        }
        else // place
        {
            // std::cout << "place" << std::endl;
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

    komo.view(true, "optimized motion");
    for (uint i = 0; i < 2; i++)
        komo.view_play(true);
}

void addToTree(arrA sequence, std::shared_ptr<keyframeNode> start)
{
    int currentPhase = getCurrentPhase(start);
    auto prevNode_ptr = start;
    for(int i=currentPhase; i<sequence.N; i++)
    {
        auto node = std::make_shared<keyframeNode>(sequence(i), prevNode_ptr);
        prevNode_ptr = node;
    }
    leafNodes.push_back(prevNode_ptr);
}

void growTree(std::vector<std::string> &inputs, std::shared_ptr <keyframeNode> start)
{
    for(int i=0; i<3; i++)
    {
        arrA sequence = sampleKeyframeSequence(inputs, start);
        addToTree(sequence, start);
    }
}

arrA getSequence(std::shared_ptr<keyframeNode> node_ptr)
{
    arrA sequence;
    sequence.append(node_ptr->get_state());
    while(node_ptr->get_parent() != nullptr)
    {
        node_ptr = node_ptr->get_parent();
        sequence.append(node_ptr->get_state());
    }

    sequence.reverse();
    return sequence;
}

int main(int argc, char **argv)
{
    std::vector<std::string> inputs = getInitInfo(argc, argv);
    if (inputs == Null_vector)
        return 0;

    // build a  start node
    auto root = makeRootNode(inputs);

    // // Get sampled path
    // growTree(inputs, root);
    // arrA finalPath = planMotion(inputs, leafNodes.front()); // Plan motion till keyframe. I know the previous points! I don't need to send keyframes!

    // // addRelTransformations(finalPath,keyFrames);
    // // optimizePath(inputs, finalPath);

    // std::cout << "final Path: " << finalPath << std::endl;
}

/**
 * @todo run motion planner within the node. A node has only one parent so graph to the node (parent->node) can be saved.
 * @todo Wrap the whole tree planner inside an ompl planner
 * @todo Can I reuse information from nodes at the same level?
 * 
 */