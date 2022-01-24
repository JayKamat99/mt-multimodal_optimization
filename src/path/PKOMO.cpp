#include <path/PKOMO.h>
#include <bits/stdc++.h>
#include <cmath>
#include <thread>

#include <ompl/tools/config/SelfConfig.h>
#include <ompl/base/goals/GoalState.h>

#define innerRadius delta
#define outerRadius 2*delta

ompl::geometric::PKOMO::PKOMO(const base::SpaceInformationPtr &si, std::string filename) : base::Planner(si, "PKOMO"), filename_(filename)
{
    sampler_ = std::make_shared<base::RealVectorStateSampler>(si_->getStateSpace()->as<base::StateSpace>());
    specs_.optimizingPaths = true;
	addPlannerProgressProperty("best cost REAL", [this] { return bestCostProperty(); });
}

ompl::geometric::PKOMO::~PKOMO()
{
    Planner::clear();
}

void ompl::geometric::PKOMO::freeMemory()
{
    if (nn_)
    {
        std::vector<Motion *> motions;
        nn_->list(motions);
        for (auto &motion : motions)
        {
            if (motion->state != nullptr)
                si_->freeState(motion->state);
            delete motion;
        }
    }
    if (nn_)
        nn_->clear();
    while (!activeList.empty())
        activeList.pop();
    // std::cout << nn_->size() << activeList.empty() << std::endl;
}

void ompl::geometric::PKOMO::clear()
{
    std::cout << "clear called" << std::endl;
	Planner::clear();
    freeMemory();
}

void ompl::geometric::PKOMO::setup()
{
    Planner::setup();
    if (pdef_->hasOptimizationObjective())
        opt_ = pdef_->getOptimizationObjective();
    else
    {
        OMPL_INFORM("%s: No optimization objective specified. Defaulting to optimizing path length for the allowed "
                    "planning time.",
                    getName().c_str());
        opt_ = std::make_shared<base::PathLengthOptimizationObjective>(si_);

        // Store the new objective in the problem def'n
        pdef_->setOptimizationObjective(opt_);
    }

    if (!nn_)
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    nn_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });
}

/**
 * @brief This method is different from the grid method in the sense that this method 
 * does not explicitly construct the grid array. Rather maintains a list of pairs 
 * containing the cell location and pointer to the motion
 * 
 * @param delta 
 * @return ompl::geometric::PathGeometricPtr 
 */
ompl::geometric::PathGeometricPtr ompl::geometric::PKOMO::bestPoissonPath_list(double delta, const base::PlannerTerminationCondition &ptc)
{
    Motion *solution = nullptr;

    /* Set Goal motion */
    ompl::base::State *goal_s = pdef_->getGoal()->as<ompl::base::GoalState>()->getState();
    auto *goal = new Motion(si_);
    si_->copyState(goal->state, goal_s);

    /* Set Start motion */
    ompl::base::State *start_s = pdef_->getStartState(0);
    auto *start = new Motion(si_);
    si_->copyState(start->state, start_s);
    start->cost = opt_->motionCost(start->state, start->state); // Initialize cost to zero
    start->costHeuristic = opt_->motionCost(start->state, goal->state); // Initialize costHeuristic
    activeList.push(start);
    nn_->add(start);

    if (activeList.size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
    }

    /* Try connecting start to goal */
    if(distanceFunction(start, goal) < outerRadius){
        solution = goal;
        solution->parent = start;
    }
    
    while (activeList.size()>0 && solution==nullptr && !ptc)
    {
        // std::cout << "OMPLINFORM: activeList.size = " << activeList.size() << std::endl;
        // choose best motion from activeList
        auto bestState = activeList.top()->state;
        activeList.pop();
        int failedAttempts = 0;
        while (failedAttempts < 10)
        {
            /* Get new sample */
            auto *rmotion = new Motion(si_);
            base::State *rstate = rmotion->state;
            sampler_->sampleShell(rstate, bestState, innerRadius, outerRadius);

            /*  Check if the new sample is feasible
                1) State is valid
                2) The nearest state is more than delta distance away.
                If both conditions are sattisfied add lazy edge.    */
            stateValid = false;
            if(si_->isValid(rstate)) // TODO: Does this check if the sample is within our bounds?
            {
                if(distanceFunction(nn_->nearest(rmotion),rmotion)<delta){
                    if (rmotion->state != nullptr)
                        si_->freeState(rmotion->state);
                    delete rmotion;
                    failedAttempts++;
                    break;
                }

                stateValid = true;

                /*Get a good parent */
                std::vector<Motion*> nMotions;
                nn_->nearestR(rmotion,outerRadius,nMotions);
                double bestMotionCost = std::numeric_limits<double>::infinity();
                Motion* pmotion;
                for (Motion* nmotion : nMotions){
                    base::Cost incCost = opt_->motionCost(rmotion->state, nmotion->state);
                    base::Cost motionCost = opt_->combineCosts(nmotion->cost, incCost);
                    if (motionCost.value() < bestMotionCost){
                        bestMotionCost = motionCost.value();
                        rmotion->parent = nmotion;
                        rmotion->cost = motionCost;
                    }
                }

                base::Cost costToGoal = opt_->motionCost(rmotion->state, goal->state);
                rmotion->costHeuristic = opt_->combineCosts(rmotion->cost, costToGoal);

                activeList.push(rmotion);
                nn_->add(rmotion);

                /* Try connecting to goal */
                if(distanceFunction(rmotion, goal) < outerRadius){
                    solution = goal;
                    solution->parent = rmotion;
                    break;
                }
            }
            if(!stateValid) { failedAttempts++; si_->freeState(rmotion->state); delete rmotion; continue; }
        }
        if (solution != nullptr) {break;}
        if (activeList.size() == 0){
            delete goal;
            return nullptr;
        }
    }

    if (solution != nullptr)
    {
        /* construct the solution path */
        std::vector<Motion *> mpath;
        while (solution != nullptr)
        {
            auto solution1 = solution;
            mpath.push_back(solution);
            solution = solution->parent;
        }

        /* set the solution path */
        auto path(std::make_shared<PathGeometric>(si_));
        for (int i = mpath.size() - 1; i >= 0; --i)
            path->append(mpath[i]->state);
        OMPL_INFORM("Initial guess found!");
        return path;
    }
    delete goal;
    return nullptr;
}

ompl::base::PlannerStatus ompl::geometric::PKOMO::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();

    if (pis_.nextStart() == nullptr)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    // variables
    delta = 1;
    threshold = outerRadius;
    isValid = false;

    // Define Goal and convert it to state and arr
    auto goal_s = pdef_->getGoal()->as<ompl::base::GoalState>()->getState();
    arr goal_;
    std::vector<double> reals;
    si_->getStateSpace()->copyToReals(reals, goal_s);
    for (int i = 0; i < reals.size(); i++){
        goal_.append(reals[i]);
    }

    const base::ReportIntermediateSolutionFn intermediateSolutionCallback = pdef_->getIntermediateSolutionCallback();

    // Setup Configuration
    rai::Configuration C;
	C.addFile(filename_.c_str());

	while (!ptc){
        auto path = bestPoissonPath_list(delta, ptc);
        if (path == nullptr){
            OMPL_INFORM("Failed to find a guess");
            delta = delta/2; continue;
        }
        path->subdivide();
        // path->interpolate(5*path->getStateCount());

        /* Convert path to arrA */
        arrA configs;
        const base::StateSpace *space(si_->getStateSpace().get());
        for (auto state : path->getStates())
            {
                arr config;
                std::vector<double> reals;
                space->copyToReals(reals, state);
                for (double r : reals){
                    config.append(r);
                }
                configs.append(config);
        }

        // Optimize the path using KOMO
        // TODO: Outsourec the KOMO code from some other file so that you can then push your planner to ompl.
        KOMO komo;
        komo.verbose = 0;
        komo.setModel(C, true);
        
        komo.setTiming(1., configs.N, 5., 2);
        komo.add_qControlObjective({}, 1, 1.);
        komo.addObjective({1.}, FS_qItself, {}, OT_eq, {10}, goal_, 0);
		komo.add_collision(true); // TODO: Is there a better function for checking collision?

        // //use configs to initialize with waypoints
        komo.initWithWaypoints(configs, configs.N, false);
        komo.run_prepare(0);
        komo.optimize();
        komo.plotTrajectory();

        configs = komo.getPath_q();

        /* Define a path */
        std::vector<const base::State*> states;
        for (int i=0; i<configs.N; i++)
        {
            std::vector<double> reals;
            for (double r : configs(i)){
                reals.push_back(r);
            }
            base::State* state = si_->allocState();
            space->copyFromReals(state, reals);
            states.push_back(state);
        }

        opti_path = std::make_shared<geometric::PathGeometric>(si_, states);
        if (/* opti_path->check() */ 1){
        OMPL_INFORM("%s: SolutionPath found with %d states", getName(), configs.N);
        bestCost = opti_path->cost(opt_).value();
        // intermediateSolutionCallback(this, states, opti_path->cost(opt_));
        isValid = true;
        }

        freeMemory();
        delta = delta/2;
	}
	if (isValid){
    pdef_->addSolutionPath(opti_path);
    return base::PlannerStatus::EXACT_SOLUTION;
    }
	else return base::PlannerStatus::TIMEOUT;
}