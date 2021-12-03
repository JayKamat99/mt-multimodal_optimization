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

    // setup KOMO
    // rai::Configuration C;
    // C.addFile(filename.c_str());
    // KOMO komo;
    // komo.verbose = 0;
    // komo.setModel(C, true);
    
    // komo.setTiming(1., configs.N, 5., 2);
    // komo.add_qControlObjective({}, 1, 2.);

    // komo.addObjective({1.}, FS_qItself, {}, OT_eq, {10}, configs(configs.N-1), 0);
    // komo.addObjective({}, FS_accumulatedCollisions, {}, OT_eq, {1.});
    // komo.add_collision(true);

    //use configs to initialize with waypoints
    komo_->initWithWaypoints(configs, path.getStateCount(), false);
    komo_->run_prepare(0);
    komo_->animateOptimization = 0;
    komo_->optimize();
	rai::Graph R = komo_->getReport(false);
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
