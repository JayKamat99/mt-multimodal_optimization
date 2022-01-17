#include <path/PKOMO.h>
#include <bits/stdc++.h>
#include <cmath>
#include <thread>


ompl::geometric::PKOMO::PKOMO(const base::SpaceInformationPtr &si) : base::Planner(si, "PKOMO")
{
	std::cout << "PKOMO object created" << std::endl;
	addPlannerProgressProperty("best cost REAL", [this] { return bestCostProperty(); });
}

ompl::geometric::PKOMO::~PKOMO()
{
	freeMemory();
}

void ompl::geometric::PKOMO::freeMemory()
{
	std::cout << "Memory has been freed" << std::endl;
}

void ompl::geometric::PKOMO::clear()
{
	std::cout << "Clear called" << std::endl;
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
    std::cout <<"d_max: "<<d_max << std::endl;
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
     * 
     * 
     */
    
    return {{0,0},{0,1},{0,2}};
}

ompl::base::PlannerStatus ompl::geometric::PKOMO::solve(const base::PlannerTerminationCondition &ptc)
{
    // variables
    double delta = 0.1;
    double threshold = 5;
    bool isValid = false;
	while (!ptc){
        arrA path = bestPoissonPath(delta); // for this I will need to access
        // arrA path = {{0,0},{0,1},{0,2}};
        arrA OptimalPath = {{0,0},{1,1},{2,2}};
        auto Goal = pdef_->getGoal();
        bool similar = compare(path,OptimalPath,threshold);
        std::cout << "dim: " << dim << std::endl;
        generate_randomUnitVector();
        if (similar){
            isValid = true;
        }
	}

	if (isValid) return base::PlannerStatus::EXACT_SOLUTION;
	else return base::PlannerStatus::INFEASIBLE;
}