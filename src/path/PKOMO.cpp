#include <path/PKOMO.h>
#include <bits/stdc++.h>
#include <cmath>
#include <thread>

#include <ompl/base/goals/GoalState.h>

ompl::geometric::PKOMO::PKOMO(const base::SpaceInformationPtr &si, std::string filename) : base::Planner(si, "PKOMO"), filename_(filename)
{
    sampler_ = std::make_shared<base::RealVectorStateSampler>(si_->getStateSpace()->as<base::StateSpace>());
    specs_.optimizingPaths = true;
	addPlannerProgressProperty("best cost REAL", [this] { return bestCostProperty(); });
}

ompl::geometric::PKOMO::~PKOMO()
{
    Planner::clear();
	freeMemory();
}

void ompl::geometric::PKOMO::freeMemory()
{
    for (int i=0; i<motionList.size(); i++)
        delete motionList.at(i);
    motionList.clear();
    activeList.clear();
    gridList.clear();
}

void ompl::geometric::PKOMO::clear()
{
	Planner::clear();
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
}

double ompl::geometric::PKOMO::dist(arr p1,arr p2)
{
    double dist = 0;
    for (int i=0; i<dim; i++){
        double diff = p1(i)-p2(i);
        dist = dist+diff*diff;
    }
    dist = sqrt(dist);
    return dist;
}

bool ompl::geometric::PKOMO::compare(arrA path,arrA OptimalPath,double threshold)
{
    // distance between 2 paths is the max of the min of distances
    double d_min[path.N] = {};
    for (int i=0; i<path.N; i++){
        d_min[i] = dist(path(i),OptimalPath(0));
        for (int j=0; j<OptimalPath.N; j++){
            if(dist(path(i),OptimalPath(j))<d_min[i]){
                d_min[i] = dist(path(i),OptimalPath(j));
            }
        }
    }
    double d_max = *std::max_element (d_min, d_min+path.N);
    if (d_max > threshold){
        return true;
    }
    else{
        return false;
    }
}

void ompl::geometric::PKOMO::runNextNestedFor(std::vector<int> counters, int index, Motion* rmotion, 
                std::shared_ptr<std::vector<Motion *>> gridArray)
{
    for(counters[index] = -gridLim; counters[index] <= gridLim; ++counters[index]){
        if (index==0){
            cellCheck = gridCell;
            for (int i=0; i<counters.size(); ++i){
                cellCheck += counters[i]*prod[i];
            }
            if ((*gridArray)[cellCheck]){
                double dist = distanceFunction((*gridArray)[cellCheck], rmotion);
                if (distanceFunction((*gridArray)[cellCheck], rmotion) < delta){
                    stateValid = false;
                }
                if (dist < min_dist)
                min_dist = dist;
                rmotion->parent = (*gridArray)[cellCheck];
            }
        }
        if (!stateValid) {break;}
        if (index!=0){
            runNextNestedFor(counters, index-1, rmotion, gridArray);
        }
    }
}

void ompl::geometric::PKOMO::runNextNestedFor_list(std::vector<int> counters, int index, Motion* rmotion)
{
    for(counters[index] = -gridLim; counters[index] <= gridLim; ++counters[index]){
        if (index==0){
            cellCheck = gridCell;
            for (int i=0; i<counters.size(); ++i){
                cellCheck += counters[i]*prod[i];
            }
            /* If you are entering for the first time, get location of the the first occupied cell with a
            higher value than cellCheck as all cells checked will have higher value than this*/
            if(!flag){
                /* Iterator of cell with value >= cellCheck */
                for (it = gridList.begin(); it != gridList.end(); ++it){
                    if ((*it).first >= cellCheck){
                        break;
                    }
                }
                if (it == gridList.end()) { --it; }
                
                flag = true;
            }

            while ((*it).first < cellCheck){
                ++it;
                if (it == gridList.end()){
                    --it;
                    break;
                }
            } //Increment iterator if not upto mark or not end
            
            if((*it).first == cellCheck){
                double dist = distanceFunction((*it).second, rmotion);
                if (dist < delta){
                    stateValid = false;
                    break;
                }
                else if (dist < min_dist)
                min_dist = dist;
                rmotion->parent = (*it).second;
            }
        }
        if (!stateValid) {break;}
        if (index!=0){
            runNextNestedFor_list(counters, index-1, rmotion);
        }
    }
}

/**
 * @brief This method is different from the grid method in the sense that this method 
 * does not explicitly construct the grid array. Rather maintains a list of pairs 
 * containing the cell location and pointer to the motion
 * 
 * @param delta 
 * @return ompl::geometric::PathGeometricPtr 
 */
ompl::geometric::PathGeometricPtr ompl::geometric::PKOMO::bestPoissonPath_list(double delta) //TODO: change the output type
{
    Motion *solution = nullptr;

    /* Set Goal motion */
    ompl::base::State *goal_s = pdef_->getGoal()->as<ompl::base::GoalState>()->getState();
    auto *goal = new Motion(si_);
    si_->copyState(goal->state, goal_s);

    /* Set Start motion */
    ompl::base::State *start_s = pdef_->getStartState(0);
    auto *start = new Motion(si_);
    motionList.push_back(start);
    si_->copyState(start->state, start_s);
    start->cost = opt_->motionCost(start->state, start->state); // Initialize cost to zero
    activeList.push_back(start);

    if (activeList.size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
    }

    /* Try connecting start to goal */
    if(distanceFunction(start, goal) < 1.2*delta){
        solution = goal;
        solution->parent = start;
    }

    /* How many cells do we need? */
    unsigned long long gridCells = 1;
    double gridCellSize = delta/sqrt(dim);
    int numElements[dim];
    for (int i=0; i<dim; i++){
        numElements[i] = ceil((bh.at(i)-bl.at(i))/gridCellSize);
        gridCells = gridCells*numElements[i];
    }

    /* What is the location of the start motion */
    int stateIndex[dim];
    gridCell = 0;
    prod.clear();
    for (int i=0; i<dim; i++){
        auto currentState = start->state->as<base::RealVectorStateSpace::StateType>();
        stateIndex[i] = ceil(((*currentState)[i] - bl.at(i))/gridCellSize);
        // std::cout << (*currentState)[i] << "index[" << i << "] : "<< stateIndex[i] << std::endl;
        prod.push_back(1);
        for (int j=0; j<i; j++){ prod[i] = prod[i]*numElements[j]; }
        gridCell += (stateIndex[i] - 1)*prod[i];
    }

    /* Insert Start to our gridList */
    gridList.push_back({gridCell,start});
    
    while (activeList.size()>0 && solution==nullptr)
    {
        std::cout << "OMPLINFORM: activeList.size = " << activeList.size() << std::endl;
        // choose best motion from activeList
        auto bestState = activeList.back()->state;
        int failedAttempts = 0;
        while (failedAttempts < 10)
        {
            /* Get new sample */
            auto *rmotion = new Motion(si_);
            base::State *rstate = rmotion->state;
            sampler_->sampleShell(rstate, bestState, delta, 1.2*delta);
            /* Check if the new sample is feasible
                1) State is valid
                2) Check if the states in the gridpoints around are more than delta distance away 
            */
            stateValid = false;
            if(si_->isValid(rstate)) // TODO: Does this check if the sample is within our bounds?
            {
                /* Get the gridCell number */
                stateValid = true;
                gridCell = 0;
                prod.clear();
                for (int i=0; i<dim; i++){
                    auto currentState = rstate->as<base::RealVectorStateSpace::StateType>();
                    stateIndex[i] = ceil(((*currentState)[i] - bl.at(i))/gridCellSize);
                    prod.push_back(1);
                    for (int j=0; j<i; j++){ prod[i] = prod[i]*numElements[j]; }
                    gridCell += (stateIndex[i] - 1)*prod[i];
                }

                /* Check if all gridCells beside it are either empty or point to a state sufficiently away */
                gridLim = ceil(sqrt(dim));

                /* Check if the state is not on the boundary */ //TODO: Make code robust on boundaries
                for (int i=0; i<dim; i++){
                    if ((stateIndex[i] < gridLim) || (stateIndex[i] > numElements[i] - gridLim)){
                        stateValid = false;
                        break;
                    }
                }

                if(!stateValid) { failedAttempts++; delete rmotion; continue; }

                min_dist = 1.2*delta;
                std::cout << "OMPLINFORM: before loops" << std::endl;
                it = gridList.begin();
                flag = false;
                std::vector<int> counters(dim);
                runNextNestedFor_list(counters, dim-1, rmotion);
                std::cout << "OMPLINFORM: after loops" << std::endl;

                if(!stateValid) { failedAttempts++; delete rmotion; continue; }

                for (it=gridList.begin(); it!=gridList.end(); ++it){
                    if ((*it).first>gridCell){
                        break;
                    }
                }

                gridList.insert(it,{gridCell,rmotion});
                motionList.push_back(rmotion);
                base::Cost incCost = opt_->motionCost(rmotion->state, rmotion->parent->state);
                rmotion->cost = opt_->combineCosts(rmotion->parent->cost, incCost);
                base::Cost costToGoal = opt_->motionCost(rmotion->state, goal->state);
                rmotion->costHeuristic = opt_->combineCosts(rmotion->cost, costToGoal);
                for (int i = activeList.size()-1; i>=0; --i){
                    if (activeList.at(i)->costHeuristic.value()>rmotion->costHeuristic.value()){
                        if (i == activeList.size()-1) { activeList.insert(activeList.begin()+i, rmotion); }
                        activeList.insert(activeList.begin()+i+1, rmotion);
                    }
                    else if (i==0){
                        activeList.insert(activeList.begin(), rmotion);
                    }
                }

                /* Try connecting to goal */
                if(distanceFunction(rmotion, goal) < 1.2*delta){
                    solution = goal;
                    solution->parent = rmotion;
                    break;
                }
            }
            if(!stateValid) { failedAttempts++; delete rmotion; continue; }
        }
        activeList.pop_back();
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

ompl::geometric::PathGeometricPtr ompl::geometric::PKOMO::bestPoissonPath_grid(double delta) //TODO: change the output type
{
    ompl::base::State *goal_s = pdef_->getGoal()->as<ompl::base::GoalState>()->getState();
    auto *goal = new Motion(si_);
    si_->copyState(goal->state, goal_s);

    ompl::base::State *start_s = pdef_->getStartState(0);
    auto *start = new Motion(si_);
    motionList.push_back(start);
    si_->copyState(start->state, start_s);
    start->cost = opt_->motionCost(start->state, start->state);
    activeList.push_back(start);

    if (activeList.size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
    }

    /* Build an array to store the pointers to the motions */
    unsigned long long gridCells = 1; //Number of grid cells required
    double gridCellSize = delta/sqrt(dim);
    int numElements[dim];
    for (int i=0; i<dim; i++){
        numElements[i] = ceil((bh.at(i)-bl.at(i))/gridCellSize);
        gridCells = gridCells*numElements[i];
    }

    /** @brief A vector to store the pointers to the motions */
    // std::vector<geometric::PKOMO::Motion*> gridArray(gridCells, nullptr); 
    auto gridArray = std::make_shared<std::vector<geometric::PKOMO::Motion*>>(gridCells, nullptr); 

    int stateIndex[dim];
    gridCell = 0;
    prod.clear();
    for (int i=0; i<dim; i++){
        auto currentState = start->state->as<base::RealVectorStateSpace::StateType>();
        stateIndex[i] = ceil(((*currentState)[i] - bl.at(i))/gridCellSize);
        // std::cout << (*currentState)[i] << "index[" << i << "] : "<< stateIndex[i] << std::endl;
        prod.push_back(1);
        for (int j=0; j<i; j++){ prod[i] = prod[i]*numElements[j]; }
        gridCell += (stateIndex[i] - 1)*prod[i];
    }

    /* Assign grid cell to start motion */
    (*gridArray)[gridCell] = start;
    
    Motion *solution = nullptr;

    while (activeList.size()>0)
    {
        // choose best motion from activeList
        auto bestState = activeList.back()->state;
        int failedAttempts = 0;
        while (failedAttempts < 3)
        {
            /* Get new sample */
            auto *rmotion = new Motion(si_);
            base::State *rstate = rmotion->state;
            sampler_->sampleShell(rstate, bestState, delta, 1.2*delta);

            /* Check if the new sample is feasible
                1) State is valid
                2) Check if the states in the gridpoints around are more than delta distance away 
            */
            stateValid = false;
            if(si_->isValid(rstate)) // TODO: Does this check if the sample is within our bounds?
            {
                /* Get the gridCell number */
                stateValid = true;
                gridCell = 0;
                prod.clear();
                for (int i=0; i<dim; i++){
                    auto currentState = rstate->as<base::RealVectorStateSpace::StateType>();
                    stateIndex[i] = ceil(((*currentState)[i] - bl.at(i))/gridCellSize);
                    // std::cout << (*currentState)[i] << "index[" << i << "] : "<< stateIndex[i] << std::endl;
                    prod.push_back(1);
                    for (int j=0; j<i; j++){ prod[i] = prod[i]*numElements[j]; }
                    gridCell += (stateIndex[i] - 1)*prod[i];
                }

                /* Check if all gridCells beside it are either empty or point to a state sufficiently away */
                gridLim = sqrt(dim) + 1;

                // long long checkings = std::pow(2*gridLim + 1, dim); // this is the number of cells we need to check = (2*gridLim+1)^dim
                // /* Create an array of the cells you need to check */
                // long long cellsToCheck[checkings];

                /* Check if the state is not on the boundary */ //TODO: Make code robust on boundaries
                for (int i=0; i<dim; i++){
                    if ((stateIndex[i] < gridLim) || (stateIndex[i] > numElements[i] - gridLim)){
                        stateValid = false;
                        break;
                    }
                }

                if(!stateValid) { failedAttempts++; delete rmotion; continue; }

                min_dist = 1.2*delta;
                std::vector<int> counters(dim);
                runNextNestedFor(counters, dim-1, rmotion, gridArray);

                if(!stateValid) { failedAttempts++; delete rmotion; continue; }

                (*gridArray)[gridCell] = rmotion;
                motionList.push_back(rmotion);
                base::Cost incCost = opt_->motionCost(rmotion->state, rmotion->parent->state);
                rmotion->cost = opt_->combineCosts(rmotion->parent->cost, incCost);
                base::Cost costToGoal = opt_->motionCost(rmotion->state, goal->state);
                rmotion->costHeuristic = opt_->combineCosts(rmotion->cost, costToGoal);
                for (int i = activeList.size()-1; i>=0; --i){
                    if (activeList.at(i)->costHeuristic.value()>rmotion->costHeuristic.value()){
                        if (i == activeList.size()-1) { activeList.insert(activeList.begin()+i, rmotion); }
                        activeList.insert(activeList.begin()+i+1, rmotion);
                    }
                    else if (i==0){
                        activeList.insert(activeList.begin(), rmotion);
                    }
                }

                /* Try connecting to goal */
                if(distanceFunction(rmotion, goal) < 1.2*delta){
                    solution = goal;
                    solution->parent = rmotion;
                    break;
                }
            }
            if(!stateValid) { failedAttempts++; delete rmotion; continue; }
        }
        activeList.pop_back();
        if (solution != nullptr) {break;}
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

ompl::geometric::PathGeometricPtr ompl::geometric::PKOMO::bestPoissonPath(double delta)
{
    /* Create Goal motion */
    ompl::base::State *goal_s = pdef_->getGoal()->as<ompl::base::GoalState>()->getState();
    auto *goal = new Motion(si_);
    si_->copyState(goal->state, goal_s);

    /* Create Start Motion */
    ompl::base::State *start_s = pdef_->getStartState(0);
    RN->printState(start_s, std::cout);
    RN->printState(goal_s, std::cout);
    auto *start = new Motion(si_);
    motionList.push_back(start);
    si_->copyState(start->state, start_s);
    start->cost = opt_->motionCost(start->state, start->state);
    activeList.push_back(start);

    if (activeList.size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
    } 
    
    Motion *solution = nullptr;

    while (activeList.size()>0)
    {
        // choose best motion from activeList
        auto bestState = activeList.back()->state;
        int failedAttempts = 0;
        std::cout << "size: " << activeList.size() << std::endl; 
        while (failedAttempts < 3)
        {
            /* Get new sample */
            auto *rmotion = new Motion(si_);
            base::State *rstate = rmotion->state;
            sampler_->sampleShell(rstate, bestState, delta, 1.2*delta);

            /* Check if the new sample is feasible
                1) State is valid
                2) Check if the relevant states in the active list more than delta distance away 
            */
            stateValid = false;
            if(si_->isValid(rstate)) // TODO: Does this check if the sample is within our bounds?
            {
                stateValid = true;

                /* Assert that the distance from any node in the motionList is atleast delta */
                double d_min = 1.2*delta;
                for (auto ir = motionList.rbegin(); ir != motionList.rend(); ++ir){
                    double d = distanceFunction(*ir, rmotion);
                    if (d < delta){
                        stateValid = false;
                        break;
                    }
                    if (d>4*delta){  // This function is random as of now. Not sure if correct
                        break;
                    }
                    if (d < d_min){
                        d_min = d;
                        rmotion->parent = *ir;
                    }
                }

                if(!stateValid) { failedAttempts++; delete rmotion; continue; }

                motionList.push_back(rmotion);
                /* Update costs of rmotion and put it in activeList */
                base::Cost incCost = opt_->motionCost(rmotion->state, rmotion->parent->state);
                rmotion->cost = opt_->combineCosts(rmotion->parent->cost, incCost);
                base::Cost costToGoal = opt_->motionCost(rmotion->state, goal->state);
                rmotion->costHeuristic = opt_->combineCosts(rmotion->cost, costToGoal);
                for (int i = activeList.size()-1; i>=0; --i){
                    if (activeList.at(i)->costHeuristic.value()>rmotion->costHeuristic.value()){
                        if (i == activeList.size()-1) { activeList.insert(activeList.begin()+i, rmotion); }
                        activeList.insert(activeList.begin()+i+1, rmotion);
                    }
                    else if (i==0){
                        activeList.insert(activeList.begin(), rmotion);
                    }
                }

                /* Try connecting to goal */
                if(distanceFunction(rmotion, goal) < 1.2*delta){
                    OMPL_INFORM("Initial guess found!");
                    solution = goal;
                    solution->parent = rmotion;
                    motionList.push_back(goal);
                    break;
                }
            }
            if(!stateValid) { failedAttempts++; delete rmotion; continue; }
        }
        if (solution != nullptr) { break; }
        activeList.pop_back();
    }
    if (activeList.size() == 0) { delete goal; }

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
        return path;
    }
    return nullptr;
}

ompl::base::PlannerStatus ompl::geometric::PKOMO::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();

    int count = 0;
    while (const base::State *st = pis_.nextStart())
    {
        count++;
    }

    if (count == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    // variables
    delta = 0.15;
    double threshold = 2*delta;
    bool isValid = false;

    // Define Goal and convert it to state and arr
    auto goal_s = pdef_->getGoal()->as<ompl::base::GoalState>()->getState();
    arr goal_;
    std::vector<double> reals;
    si_->getStateSpace()->copyToReals(reals, goal_s);
    for (int i = 0; i < reals.size(); i++){
        goal_.append(reals[i]);
    }

    // Setup Configuration
    rai::Configuration C;
	C.addFile(filename_.c_str());

	while (!ptc){
        // auto path = bestPoissonPath(delta);
        // auto path = bestPoissonPath_grid(delta);
        auto path = bestPoissonPath_list(delta);
        path->print(std::cout);

        /* Convert path to arrA */
        arrA configs;
        //To copy the path to arrA Configs from states.
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
        
        komo.setTiming(1., 10*configs.N, 5., 2);
        komo.add_qControlObjective({}, 1, 1.);
        komo.addObjective({1.}, FS_qItself, {}, OT_eq, {10}, goal_, 0);
		komo.add_collision(true); // TODO: Is there a better function for checking collision?

        // //use configs to initialize with waypoints
        komo.initWithWaypoints(configs, configs.N, false);
        komo.run_prepare(0);
        komo.optimize();
        komo.plotTrajectory();

        // Viewer for debug purposes only.
        // rai::ConfigurationViewer V;
        // V.setPath(C, komo.x, "result", true);
        // V.playVideo(true, 1.);

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

        geometric::PathGeometricPtr opti_path = std::make_shared<geometric::PathGeometric>(si_, states);
        // base::PlannerSolution psol = base::PlannerSolution::PlannerSolution(opti_path);
        pdef_->addSolutionPath(opti_path); //TODO: Check for path validity
        isValid = true;
        break;

        // bool similar = compare(path,OptimalPath,threshold);
        // if (similar){
        //     isValid = true;
        // }

        // pdef_->addSolutionPath(ompl::geometric::PathGeometric Path);
        freeMemory();
        // delta = delta/2;
	}

	if (isValid) return base::PlannerStatus::EXACT_SOLUTION;
	else return base::PlannerStatus::TIMEOUT;
}