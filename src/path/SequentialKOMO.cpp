#include <path/SequentialKOMO.h>
#include <thread>
#include <chrono>

ompl::geometric::SequentialKOMO::SequentialKOMO(const base::SpaceInformationPtr &si) : base::Planner(si, "SequentialKOMO")
{
	addPlannerProgressProperty("best cost REAL", [this] { return bestCostProperty(); });
	C_Dimension = si->getStateDimension();
}

void ompl::geometric::SequentialKOMO::setup()
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
	bestCost = std::numeric_limits<double>::infinity();
}

ompl::geometric::SequentialKOMO::~SequentialKOMO()
{
	freeMemory();
}

void ompl::geometric::SequentialKOMO::freeMemory()
{
}

void ompl::geometric::SequentialKOMO::clear()
{
	// std::cout << "Clear called" << std::endl;
	Planner::clear();
	bestCost = std::numeric_limits<double>::infinity();
	iteration = 0;
}

std::shared_ptr<KOMO> ompl::geometric::SequentialKOMO::setupKOMO()
{
	// initialize the configuration
	rai::Configuration C(inputs.at(0).c_str());
	startConfig = C.getJointState();

	int totalPhases = stoi(inputs.at(1));

	auto komo_ = std::make_shared<KOMO>();
	komo_->setModel(C, true);
	komo_->setTiming(totalPhases, 15, 5, 2);
	komo_->verbose = 0;

	int phase = 0;
	while (phase < totalPhases)
	{
		std::string ref1 = inputs.at(2 + phase * 2), ref2 = inputs.at(3 + phase * 2);

		if (phase == 0)
			komo_->addSwitch_stable(1, 2., "", ref1.c_str(), ref2.c_str());
		else if (phase % 2 == 0)
			komo_->addSwitch_stable(phase + 1, phase + 2., "", ref1.c_str(), ref2.c_str(), false);
		else
			komo_->addSwitch_stable(phase + 1, phase + 2., "", ref2.c_str(), ref1.c_str(), false);
		komo_->addObjective({phase + 1.}, FS_distance, {ref1.c_str(), ref2.c_str()}, OT_eq, {1e2});
		komo_->addObjective({phase + 1.}, FS_vectorZ, {ref1.c_str()}, OT_eq, {1e2}, {0., 0., 1.});

		if (phase % 2 == 0) // pick
		{
			komo_->addObjective({phase + 1.}, FS_scalarProductXX, {ref1.c_str(), ref2.c_str()}, OT_eq, {1e2}, {0.});
		}
		else // place
		{
			komo_->addObjective({phase + 1.}, FS_aboveBox, {ref2.c_str(), ref1.c_str()}, OT_ineq, {1e2});
		}
		phase++;
	}

	komo_->add_qControlObjective({}, 1);
	komo_->add_collision(true, 0.01);

	return komo_;
}

ompl::base::PlannerStatus ompl::geometric::SequentialKOMO::solve(const base::PlannerTerminationCondition &ptc)
{
    bool isValid = false;
	std::cout << "Starting KOMO, bestCost = " << bestCost << std::endl;


	while (!ptc){
		iteration++;
		// Setup KOMO using inuts
		auto komo_ = setupKOMO();
		// komo_->initWithConstant(startConfig);

		komo_->run_prepare(0);
		komo_->optimize();
		arrA configs = komo_->getPath_q();
		// komo_->view(true);
		// komo_->view_play(true);

		rai::Graph R = komo_->getReport(false);
		double constraint_violation = R.get<double>("eq") + R.get<double>("ineq");
		// std::cout << "Constraint Violation: " << constraint_violation << std::endl;

		if (constraint_violation < 20 /* maxConstraintViolationKOMO */)
			isValid = true;
		else
			continue;

		// If better than before; update
		/* Convert arrA to PathGeometric to get the cost and validity */
        std::vector<const base::State*> states;
		const base::StateSpace *space(si_->getStateSpace().get());
        for (int i=0; i<configs.N; i++)
        {
            std::vector<double> reals;
			arr config = configs(i).resize(C_Dimension);
			std::cout << C_Dimension << std::endl;
			std::cout << space->getDimension() << std::endl;
			std::cout << config << std::endl;
            for (double r : config){
                reals.push_back(r);
            }
            ompl::base::State* state = si_->allocState();
            space->copyFromReals(state, reals);
            states.push_back(state);
        }

		double cost;

		auto path = std::make_shared<geometric::PathGeometric>(si_, states);
        if (path->check()){
			cost = path->cost(pdef_->getOptimizationObjective()).value();
			isValid = true;
		}

		if (isValid && cost<bestCost){
			this->bestCost = cost;
			std::cout << "best cost updated: " << bestCost << std::endl;
			// std::cout << "Iteration: " << iteration << std::endl;
			// break;
		}
	}

	pdef_->addSolutionPath(path, false, 0.0, getName());
	if (isValid) {
		std::cout << "Exact solution" << std::endl;
		// Pause for a 10th of a second
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		return base::PlannerStatus::EXACT_SOLUTION;
	}
	else return base::PlannerStatus::INFEASIBLE;
}