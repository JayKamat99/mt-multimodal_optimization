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
    // lastGoalMotion_ = nullptr;
}

void ompl::geometric::PKOMO::freeMemory()
{
	// lastGoalMotion_ = nullptr;
}

void ompl::geometric::PKOMO::clear()
{
	Planner::clear();
}

double ompl::geometric::PKOMO::costToGoal(std::vector<double> p1)
{
    // This must return cost till state + dist to goal
    double dist = 0;
    for (int i=0; i<dim; i++){
        double diff = p1[i]-goal[i];
        dist = dist+diff*diff;
    }
    dist = sqrt(dist);
    return dist; // This is not what we must return
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

void ompl::geometric::PKOMO::generate_randomUnitVector()
{
    std::normal_distribution<double> normal_distribution(0,1);

    u.clear();
    double length = 0;
    for (int i=0; i<dim; i++){
        u.push_back(normal_distribution(this->generator));
        length = length + u[i]*u[i];
    }

    length = sqrt(length);

    // Normalize vector u
    double normal_length = 0;
    for (int i=0; i<dim; i++){
        u[i] = u[i]/length;
        normal_length = normal_length + u[i]*u[i];
    }
}

ompl::geometric::PathGeometricPtr ompl::geometric::PKOMO::bestPoissonPath(double delta) //TODO: change the output type
{
    ompl::base::State *goal_s = pdef_->getGoal()->as<ompl::base::GoalState>()->getState();

    ompl::base::State *start_s = pdef_->getStartState(0);
    auto *start = new Motion(si_);
    si_->copyState(start->state, start_s);
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
        numElements[i] = (bh.at(i)-bl.at(i))/gridCellSize + 1;
        gridCells = gridCells*numElements[i];
    }

    /** @brief A vector to store the pointers to the motions */
    std::vector<geometric::PKOMO::Motion*> gridArray(gridCells, nullptr); 

    int stateIndex[dim];
    unsigned long long gridCell = 0;
    unsigned long long prod[dim];
    for (int i=0; i<dim; i++){
        auto currentState = start->state->as<base::RealVectorStateSpace::StateType>();
        stateIndex[i] = ((*currentState)[i] - bl.at(i))/gridCellSize + 1;
        // std::cout << (*currentState)[i] << "index[" << i << "] : "<< stateIndex[i] << std::endl;
        prod[i] = 1;
        for (int j=0; j<i; j++){ prod[i] = prod[i]*numElements[j]; }
        gridCell += (stateIndex[i] - 1)*prod[i];
    }
    gridCell++;

    /* Assign grid cell to start motion */
    gridArray[gridCell] = start;
    
    Motion *solution = nullptr;

    while (activeList.size()>0)
    {
        // choose best motion from activeList
        auto bestState = activeList[activeList.size()-1]->state;
        int failedAttempts = 0;
        while (failedAttempts < 30)
        {
            /* Get new sample */
            auto *rmotion = new Motion(si_);
            base::State *rstate = rmotion->state;
            sampler_->sampleShell(rstate, bestState, delta, 2*delta);

            /* Check if the new sample is feasible
                1) State is valid
                2) Check if the states in the gridpoints around are more than delta distance away 
            */
            bool stateValid{false};
            if(si_->isValid(rstate)) // TODO: Does this check if the sample is within our bounds?
            {
                /* Get the gridCell number */
                stateValid = true;
                long long gridCell = 0;
                long long prod[dim];
                for (int i=0; i<dim; i++){
                    auto currentState = rstate->as<base::RealVectorStateSpace::StateType>();
                    stateIndex[i] = ((*currentState)[i] - bl.at(i))/gridCellSize + 1;
                    // std::cout << (*currentState)[i] << "index[" << i << "] : "<< stateIndex[i] << std::endl;
                    prod[i] = 1;
                    for (int j=0; j<i; j++){ prod[i] = prod[i]*numElements[j]; }
                    gridCell += (stateIndex[i] - 1)*prod[i];
                }
                gridCell = gridCell+1;
                // std::cout << gridCell<< std::endl;

                /* Check if all gridCells beside it are either empty or point to a state sufficiently away */
                int gridLim = sqrt(dim) + 1;

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

                if(!stateValid) { failedAttempts++; continue; }


                // TODO: How do I have dim number of for loops????
                if (dim == 7) // This is a very bad way of doing it but how do I generalize to dim dimensions?
                {
                    long long cellCheck;
                    for(int index_6 = -gridLim; index_6 <= gridLim; index_6++){
                        for(int index_5 = -gridLim; index_5 <= gridLim; index_5++){
                            for(int index_4 = -gridLim; index_4 <= gridLim; index_4++){
                                for(int index_3 = -gridLim; index_3 <= gridLim; index_3++){
                                    for(int index_2 = -gridLim; index_2 <= gridLim; index_2++){
                                        for(int index_1 = -gridLim; index_1 <= gridLim; index_1++){
                                            for(int index_0 = -gridLim; index_0 <= gridLim; index_0++){
                                                cellCheck = gridCell + index_6*prod[6]+index_5*prod[5]+index_4*prod[4]
                                                    +index_3*prod[3]+index_2*prod[2]+index_1*prod[1]+index_0*prod[0];
                                                if (gridArray[cellCheck]){
                                                    if (distanceFunction(gridArray[cellCheck], rmotion) < delta){
                                                        stateValid = false;
                                                        rmotion->~Motion();
                                                    }
                                                }
                                                if (!stateValid) {break;}
                                            }
                                            if (!stateValid) {break;}
                                        }
                                        if (!stateValid) {break;}
                                    }
                                    if (!stateValid) {break;}
                                }
                                if (!stateValid) {break;}
                            }
                            if (!stateValid) {break;}
                        }
                        if (!stateValid) {break;}
                    }
                }

                if(!stateValid) { failedAttempts++; continue; }

                /* get to the gridCell;
                if null then for every grid near that state check if a gridCell exists
                if yes then check the distance between the 2 states. 
                if the distance between the 2 states is less than delta{stateValid = false; break;}
                if (stateValid == true){gridArray[gridCell] = motionptrVal}
                */
            }

            // If yes, connect the new state to the nearest motion
            // check for distance to goal and update heuristic.
            // if (distance to goal < 2*delta)
            // Add goal as a motion too and return path!!!!
            // pathFound = true;
            // else faliure++
            failedAttempts++;
        }
        activeList.pop_back();
    }

    if (solution != nullptr)
    {
        /* construct the solution path */
        std::vector<Motion *> mpath;
        while (solution != nullptr)
        {
            mpath.push_back(solution);
            solution = solution->parent;
        }

        /* set the solution path */
        auto path(std::make_shared<PathGeometric>(si_));
        for (int i = mpath.size() - 1; i >= 0; --i)
            path->append(mpath[i]->state);
        OMPL_INFORM("Initial guess found!");
        return path;
        // pdef_->addSolutionPath(path, approximate, approxdif, getName());
    }

    return nullptr;

}

// arrA ompl::geometric::PKOMO::bestPoissonPath(double delta)
// {
//     // Init checks
//     checkValidity();

//     // generate grid
//     int grid_cells = 0;
//     for (int i=0; i<dim; i++){
//         grid_cells += (bh.at(i)-bl.at(i))*sqrt(dim)/delta + 1;
//     }
//     double grid_array[grid_cells];
//     for (int i=0; i<grid_cells; i++){grid_array[i]=-1;}

//     // get Start State, convert it to vector<double> and pass it to the active list
//     base::State *start_s = pdef_->getStartState(0);
//     std::vector<double> start;
//     si_->getStateSpace()->copyToReals(start, start_s);
//     activeList.push_back(start);
//     // costToGoal_list.push_back(costToGoal(start));

//     while (!complete) // while path to goal not found repeat
//     {
//         int k=0;
//         while (k<30)
//         {
//             // Sample a new random state
//             generate_randomUnitVector();
//             std::uniform_real_distribution<double> uniform_distribution(delta,2*delta);
//             double r = uniform_distribution(this->generator);
//             for (int i=0; i<dim; i++){
//                 u[i] = u[i]*r;
//             }
//             // add to best motion and check for feasibility{not close to other samples + valid}
//             // if valid make this a child of the previous motion.
//             // Check which grid this point is ...
//         }
//     }

    
//     // return path;
//     return {{0,0},{0,1},{0,2}};
// }

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
    double delta = 1;
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
        auto path = bestPoissonPath(delta);

        // Optimize the path using KOMO
        // TODO: Outsourec the KOMO code from some other file so that you can then push your planner to ompl.
        // KOMO komo;
        // komo.verbose = 0;
        // komo.setModel(C, true);
        
        // komo.setTiming(1., 10*path.N, 5., 2);
        // komo.add_qControlObjective({}, 1, 1.);
        // komo.addObjective({1.}, FS_qItself, {}, OT_eq, {10}, goal_, 0);
		// komo.add_collision(true); // TODO: Is there a better function for checking collision?

        // //use configs to initialize with waypoints
        // // komo.initWithWaypoints(path, path.N, false);
        // komo.run_prepare(0);
        // komo.optimize();
        // komo.plotTrajectory();

        // Viewer for debug purposes only.
        // rai::ConfigurationViewer V;
        // V.setPath(C, komo.x, "result", true);
        // V.playVideo(true, 1.);

        // bool similar = compare(path,OptimalPath,threshold);
        // if (similar){
        //     isValid = true;
        // }

        // pdef_->addSolutionPath(ompl::geometric::PathGeometric Path);
	}

	if (isValid) return base::PlannerStatus::EXACT_SOLUTION;
	else return base::PlannerStatus::TIMEOUT;
}