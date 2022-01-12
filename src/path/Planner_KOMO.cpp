#include <path/Planner_KOMO.h>
#include <thread>

ompl::geometric::Planner_KOMO::Planner_KOMO(const base::SpaceInformationPtr &si, std::shared_ptr<KOMO> komo_) : base::Planner(si, "Planner_KOMO"), komo_(std::move(komo_))
{
	// std::cout << "Planner_KOMO object created" << std::endl;
	addPlannerProgressProperty("best cost REAL", [this] { return bestCostProperty(); });
}

ompl::geometric::Planner_KOMO::~Planner_KOMO()
{
	freeMemory();
}

void ompl::geometric::Planner_KOMO::freeMemory()
{
	// std::cout << "Memory has been freed" << std::endl;
}

void ompl::geometric::Planner_KOMO::clear()
{
	// std::cout << "Clear called" << std::endl;
	Planner::clear();
}

ompl::base::PlannerStatus ompl::geometric::Planner_KOMO::solve(const base::PlannerTerminationCondition &ptc)
{
	// std::cout << "This will run KOMO.optimize" << std::endl;

    bool isValid = false;
	while (!ptc){
		komo_->run_prepare(1);
		komo_->animateOptimization = 0;
		komo_->optimize(1);
		rai::Graph R = komo_->getReport(false);
		double constraint_violation = R.get<double>("eq") + R.get<double>("ineq");

		arrA configs = komo_->getPath_q();
		double cost = R.get<double>("sos");
		// std::cout << "cost" << cost << std::endl;

		if (constraint_violation < 1){
			isValid = true;
		}

		if (isValid && cost<bestCost){
			this->bestCost = cost;
			//save the path
			const base::StateSpace *space(si_->getStateSpace().get());
			auto path(std::make_shared<PathGeometric>(si_));
			for (uint i=0; i<komo_->stepsPerPhase; i++)
			{
				ompl::base::State* state = si_->allocState();
				std::vector<double> reals;
				for (double r : configs(i)){
					reals.push_back(r);
				}
				space->copyFromReals(state, reals);
				path->append(state);
			}
			pdef_->addSolutionPath(path, false, 0.0, getName());
		}
	}

	if (isValid) return base::PlannerStatus::EXACT_SOLUTION;
	else return base::PlannerStatus::INFEASIBLE;
}