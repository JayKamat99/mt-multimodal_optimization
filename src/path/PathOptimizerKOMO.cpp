#include <ompl/geometric/PathOptimizer.h>
#include <path/PathOptimizerKOMO.h>
#include <ompl/multilevel/planners/multimodal/PathSpaceSparse.h>
#include <ompl/multilevel/planners/multimodal/datastructures/PathSpace.h>
#include <chrono>

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace om = ompl::multilevel;

ompl::geometric::PathOptimizerKOMO::PathOptimizerKOMO(base::SpaceInformationPtr si, 
std::shared_ptr<KOMO> komo_) : komo_(std::move(komo_)){
    ompl::geometric::PathOptimizer::setSI(si);
    // KOMO komo; //initialize KOMO here
}

bool ompl::geometric::PathOptimizerKOMO::optimize(PathGeometric &path)
{
    // static std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    // static std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    // begin = std::chrono::steady_clock::now();
    // std::cout << "Non-KOMO time = " << std::chrono::duration_cast<std::chrono::microseconds>(begin-end).count() << "[µs]" << std::endl;
    // std::cout << "Non-KOMO time = " << std::chrono::duration_cast<std::chrono::nanoseconds> (begin-end).count() << "[ns]" << std::endl;
	arrA configs;
	//To copy the path to arrA Configs from states.
	const base::StateSpace *space(si_->getStateSpace().get());
	for (auto state : path.getStates())
		{
			arr config;
			std::vector<double> reals;
			space->copyToReals(reals, state);
			for (double r : reals){
				config.append(r);
			}
			configs.append(config);
	}

    //use configs to initialize with waypoints
    komo_->initWithWaypoints(configs, path.getStateCount(), false);
    komo_->run_prepare(0);
    komo_->animateOptimization = 0;
    komo_->optimize(0);
	rai::Graph R = komo_->getReport(false);
    // double cost = R.get<double>("sos");
 	double constraint_violation = R.get<double>("eq") + R.get<double>("ineq");

    configs = komo_->getPath_q();

    bool isValid = true;
    if (constraint_violation > 1){
        isValid = false;
    }

    //copy the final config back to states
	int i=0;
	for (auto state : path.getStates())
    {
		std::vector<double> reals;
		for (double r : configs(i)){
			reals.push_back(r);
		}
		space->copyFromReals(state, reals);
		i++;
    }
    // end = std::chrono::steady_clock::now();

    // std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[µs]" << std::endl;
    // std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::nanoseconds> (end - begin).count() << "[ns]" << std::endl;
	
    isStepWise = false;
    setPathCost(R.get<double>("sos"));
    return isValid;
}

ompl::geometric::PathGeometricPtr ompl::geometric::PathOptimizerKOMO::optimize_path(PathGeometricPtr &path)
{
    // std::cout << "OptimizingPath" << std::endl;
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

    // Are the steps per phase we have initialized enough?
    if (configs.N > komo_->stepsPerPhase)
    {
        OMPL_ERROR("path has too many way points. Increase steps per phase");        
        return nullptr;
    }
    // use configs to initialize with waypoints
    komo_->initWithWaypoints(configs, configs.N, false);
    komo_->run_prepare(0);
    komo_->animateOptimization = 0;
    komo_->optimize(0);

    configs = komo_->getPath_q();

    /* Define arrA as path */
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

    // std::cout << "PathOptimized" << std::endl;

    geometric::PathGeometricPtr opti_path = std::make_shared<geometric::PathGeometric>(si_, states);
    if (opti_path->check())
    {
        return opti_path;
    }
    else return nullptr;
}