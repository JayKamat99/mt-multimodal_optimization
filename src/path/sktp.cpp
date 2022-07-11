#include <path/sktp.h>

ompl::geometric::sktp::sktp(const base::SpaceInformationPtr &si) : base::Planner(si, "KOMO")
{
	// std::cout << "sktp object created" << std::endl;
	addPlannerProgressProperty("best cost REAL", [this] { return bestCostProperty(); });
}

void ompl::geometric::sktp::setup()
{
	Planner::setup();
	bestCost = std::numeric_limits<double>::infinity();
}

ompl::geometric::sktp::~sktp()
{
	freeMemory();
}

void ompl::geometric::sktp::freeMemory()
{
}

void ompl::geometric::sktp::clear()
{
	// std::cout << "Clear called" << std::endl;
	Planner::clear();
	bestCost = std::numeric_limits<double>::infinity();
}

ompl::base::PlannerStatus ompl::geometric::sktp::solve(const base::PlannerTerminationCondition &ptc)
{
	// the keyframe tree function

	// Make root node
	// loop
	return ompl::base::PlannerStatus::ABORT;
}