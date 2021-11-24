#include <path/Planner_KOMO.h>
#include <thread>

ompl::geometric::Planner_KOMO::Planner_KOMO(const base::SpaceInformationPtr &si, std::shared_ptr<KOMO> komo_) : base::Planner(si, "Planner_KOMO"), komo_(std::move(komo_))
{
	std::cout << "Planner_KOMO object created" << std::endl;
	addPlannerProgressProperty("best cost REAL", [this] { return bestCostProperty(); });
}

ompl::geometric::Planner_KOMO::~Planner_KOMO()
{
	freeMemory();
}

void ompl::geometric::Planner_KOMO::freeMemory()
{
	std::cout << "Memory has been freed" << std::endl;
}

ompl::base::PlannerStatus ompl::geometric::Planner_KOMO::solve(const base::PlannerTerminationCondition &ptc)
{
	std::cout << "This will run KOMO.optimize" << std::endl;
	// setup KOMO
    /* rai::Configuration C;
    C.addFile(filename.c_str());
    KOMO komo;
    komo.verbose = 0;
    komo.setModel(C, true);
    
    komo.setTiming(1., 30, 5., 2);
    komo.add_qControlObjective({}, 1, 2.);

	arr goal, f_vel;
	for (int i=0; i < C.getJointStateDimension(); i++){
		if (i>3) goal.append(0);
		else goal.append(1.5);
		f_vel.append(0);
	}

    komo.addObjective({1.}, FS_qItself, {}, OT_eq, {10}, goal, 0);
	komo.addObjective({0.9,1.}, FS_qItself, {}, OT_eq, {10}, f_vel, 1);
    komo.addObjective({}, FS_accumulatedCollisions, {}, OT_eq, {1.});
    komo.add_collision(true); */

    komo_->run_prepare(0);
    komo_->animateOptimization = 0;
    komo_->optimize();
	rai::Graph R = komo_->getReport(false);
 	double constraint_violation = R.get<double>("eq") + R.get<double>("ineq");

    arrA configs = komo_->getPath_q();
	double cost = R.get<double>("sos");
	std::cout << "cost" << cost << std::endl;
	this->bestCost = cost;

    bool isValid = true;
    if (constraint_violation > 1){
        isValid = false;
    }

	std::chrono::seconds dura(1);
    // std::this_thread::sleep_for( dura );

	//copy the final config to states
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

	if (isValid) return base::PlannerStatus::EXACT_SOLUTION;
	else return base::PlannerStatus::INFEASIBLE;
}