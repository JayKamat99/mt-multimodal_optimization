#include <path/PKOMO.h>
#include <bits/stdc++.h>
#include <cmath>
#include <thread>

#include <ompl/base/goals/GoalState.h>

ompl::geometric::PKOMO::PKOMO(const base::SpaceInformationPtr &si, std::string filename) : base::Planner(si, "PKOMO"), filename_(filename)
{
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

double ompl::geometric::PKOMO::dist(arr p1,arr p2)
{
    this->dim = p1.N;
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
    std::uniform_real_distribution<double> uniform_distribution(0,1); // TODO: put this in your poisson distribution code

    // generate a random vector u
    double u[dim] = {};
    double length = 0;
    for (int i=0; i<dim; i++){
        u[i] = normal_distribution(this->generator);
        length = length + u[i]*u[i];
    }

    length = sqrt(length);

    // Normalize vector u
    double normal_length = 0;
    for (int i=0; i<dim; i++){
        u[i] = u[i]/length;
        normal_length = normal_length + u[i]*u[i];
    }

    // TODO: return statement
}

arrA ompl::geometric::PKOMO::bestPoissonPath(double delta)
{
    /**
     * @brief Pseudocode
     * generate_grid();
     * auto start = pdef_.getStartState(0);
     * auto goal = pdef_.getGoal();
     * generate random feasible points near from start
     * calculate cost for each point
     * 
     */

    // generate grid
    int grid_cells = 0;
    for (int i=0; i<dim; i++){
        grid_cells += (bh.at(i)-bl.at(i))*sqrt(dim)/delta + 1;
    }
    double grid_array[grid_cells];
    for (int i=0; i<grid_cells; i++){grid_array[i]=-1;}

    // ompl::base::Goal Goal = pdef_->getGoal().get();


    
    return {{0,0},{0,1},{0,2}};
}

ompl::base::PlannerStatus ompl::geometric::PKOMO::solve(const base::PlannerTerminationCondition &ptc)
{
    // variables
    double delta = 0.1;
    double threshold = 5;
    bool isValid = false;

    // Define Goal and convert it to state
    ompl::base::Goal *goal = pdef_->getGoal().get();
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
        arrA path = bestPoissonPath(delta);

        // Optimize the path using KOMO
        KOMO komo;
        komo.verbose = 0;
        komo.setModel(C, true);
        
        komo.setTiming(1., 10*path.N, 5., 2);
        komo.add_qControlObjective({}, 1, 1.);
        komo.addObjective({1.}, FS_qItself, {}, OT_eq, {10}, goal_, 0);

        //use configs to initialize with waypoints
        // komo.initWithWaypoints(path, path.N, false);
        komo.run_prepare(0);
        komo.optimize();
        komo.plotTrajectory();

        rai::ConfigurationViewer V;
        V.setPath(C, komo.x, "result", true);
        V.playVideo(true, 1.);

        // bool similar = compare(path,OptimalPath,threshold);
        // if (similar){
        //     isValid = true;
        // }
	}

	if (isValid) return base::PlannerStatus::EXACT_SOLUTION;
	else return base::PlannerStatus::INFEASIBLE;
}