#include <path/Planner_KOMO.h>
#include <thread>

ompl::geometric::Planner_KOMO::Planner_KOMO(const base::SpaceInformationPtr &si, std::shared_ptr<KOMO> komo_) : base::Planner(si, "Planner_KOMO"), komo_(std::move(komo_))
{
	// std::cout << "Planner_KOMO object created" << std::endl;
	addPlannerProgressProperty("best cost REAL", [this] { return bestCostProperty(); });
}

void ompl::geometric::Planner_KOMO::setup()
{
	Planner::setup();
	// See if we have an optimization objective
	if (!Planner::pdef_->hasOptimizationObjective())
	{
		OMPL_INFORM("%s: No optimization objective specified. Defaulting to optimizing path length.",
					Planner::getName().c_str());
		Planner::pdef_->setOptimizationObjective(
			std::make_shared<base::PathLengthOptimizationObjective>(Planner::si_));
	}
	startConfig = komo_->getConfiguration_q(0);
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
	bestCost = std::numeric_limits<double>::infinity();
}

ompl::base::PlannerStatus ompl::geometric::Planner_KOMO::solve(const base::PlannerTerminationCondition &ptc)
{
    bool isValid = false;
	while (!ptc){
		komo_->initWithConstant(startConfig);
		komo_->run_prepare(0);
		komo_->animateOptimization = 0;
		komo_->optimize();

		// rai::Graph R = komo_->getReport(false);
		// double constraint_violation = R.get<double>("eq") + R.get<double>("ineq");
		// double cost = R.get<double>("sos");
		// std::cout << "cost" << cost << std::endl;

		/* Convert arrA to PathGeometric to get the cost and validity */
		arrA configs = komo_->getPath_q();
        std::vector<const base::State*> states;
		const base::StateSpace *space(si_->getStateSpace().get());
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

		double cost = bestCost;

        auto opti_path = std::make_shared<geometric::PathGeometric>(si_, states);
        if (opti_path->check()){
			// OMPL_INFORM("SolutionPath found with states");
			cost = opti_path->cost(pdef_->getOptimizationObjective()).value();
			// std::cout << "cost: " << cost << std::endl;
			// intermediateSolutionCallback(this, states, opti_path->cost(opt_));
			isValid = true;
		}

		if (isValid && cost<bestCost){
			this->bestCost = cost;
			//save the path
			// std::cout << "I reach here1" << std::endl;
			const base::StateSpace *space(si_->getStateSpace().get());
			path = std::make_shared<PathGeometric>(si_);
			for (uint i=0; i<configs.N; i++)
			{
				ompl::base::State* state = si_->allocState();
				std::vector<double> reals;
				for (double r : configs(i)){
					reals.push_back(r);
				}
				space->copyFromReals(state, reals);
				path->append(state);
			}
		}
		std::cout << bestCost << std::endl;
	}

	pdef_->addSolutionPath(path, false, 0.0, getName());
	if (isValid) return base::PlannerStatus::EXACT_SOLUTION;
	else return base::PlannerStatus::INFEASIBLE;
}