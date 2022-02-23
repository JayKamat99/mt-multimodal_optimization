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
    // I am getting the path as PathGeometricPtr; however, KOMO  needs it in form of arrA. So, let's convert!
    /* Convert path to arrA */
    arrA configs;
    opti_path = nullptr;
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

    // Are the steps per phase we have initialized enough? If not, I am sorry, you did not supply the right discretization factor. You need to take care about it next time.
    if (configs.N > komo_->stepsPerPhase)
    {
        OMPL_ERROR("path has too many way points. %d to be precise. Increase steps per phase", configs.N);        
        return nullptr;
    }
    // Now, that we have gotten the path in the right form use it to initialize KOMO (with waypoints)
    komo_->run_prepare(0);
    komo_->initWithWaypoints(configs, configs.N, false);
    komo_->animateOptimization = 0; // Make this 1 if you want to see how the path evolves over time.
    bool flag_{false};
    ompl::geometric::PathGeometricPtr partialOptiPath;

    // Is our path in collision? If yes, try getting it out!
    // if (!path->check())
    if(0)
    {
        std::cout << "Path is not valid" << std::endl;
        // Don't optimize fully but rather try only for 10 iterations
        rai::setParameter<double>("opt/stopEvals",10);
        // std::cout << "stopEvals:" << rai::getParameter<double>("opt/stopEvals") << std::endl;;
        // static std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        komo_->optimize(0);
        // static std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        // std::cout << "Partial Optimization time = " << std::chrono::duration_cast<std::chrono::milliseconds>(begin-end).count() << "[ms]" << std::endl;
        rai::setParameter<double>("opt/stopEvals",1000);
        // std::cout << "stopEvals:" << rai::getParameter<double>("opt/stopEvals") << std::endl;;
        configs = komo_->getPath_q();

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
        partialOptiPath = std::make_shared<geometric::PathGeometric>(si_, states);

        // Check again if the path is in collision. If yes, then we were unable to get it out and so return failure
        if (!partialOptiPath->check())
        {
            std::cout << "Path is still not valid, sorry" << std::endl;
            return nullptr; // I am sorry, we cannot help :(
        }
        else 
        {
            flag_ = true;
            std::cout << "The path has successfully moved away from the obstacles!" << std::endl;
        }
    }
    // No else,  we can continure with our planner

    // Now, that we have a green signal, let us optimizer the path fully
    komo_->run_prepare(0);
    // static std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    komo_->optimize(0);
    // static std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    // std::cout << "Full Optimization time = " << std::chrono::duration_cast<std::chrono::milliseconds>(begin-end).count() << "[ms]" << std::endl;
    configs = komo_->getPath_q();

    // Wait! Don't get excited because the optimization is completed. You need to do a few checks to be sure that you have a feasible solution. 
    // Have I reached the goal? Or atleast close enough?
    rai::Graph R = komo_->getReport();
    double eq_constraint = R.get<double>("eq");
    // std::cout << "eq: " << eq_constraint << std::endl;
    if (eq_constraint > 50) // I am sorry but our solution does not respect the equality constraint
    {
        return nullptr;
    }
    // No else, we simply continue. Path respectes the equality constratint  

    // Now, check of the path is feasible, i.e. collision free. But before that you need to convert it back to PathGeometric
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
    opti_path = std::make_shared<geometric::PathGeometric>(si_, states);

    // Check if the path is in collision.
    if (!opti_path->check())
    {
        if (!flag_)
            return nullptr; // I am sorry you are in collision
        else
        {
            std::cout << "You still have a backup!" << std::endl;
            return partialOptiPath; // Hey but I have a path for you!
        }
    }
    // No else continue further checks.

    // Ahh! We are done with all checks! Here is the optimized path we found!
    return opti_path;
}

ompl::geometric::PathGeometricPtr ompl::geometric::PathOptimizerKOMO::getPath() const
{
    return opti_path;
}