#include <iostream>

// Basic Requirements
#include <ompl/config.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/tools/benchmark/Benchmark.h>
#include <ompl/base/goals/GoalLazySamples.h>

// Samplers
#include <ompl/base/samplers/ObstacleBasedValidStateSampler.h>
#include <ompl/base/samplers/informed/PathLengthDirectInfSampler.h>
#include <ompl/base/samplers/InformedStateSampler.h>
#include <ompl/base/samplers/GaussianValidStateSampler.h>
#include <ompl/base/samplers/MinimumClearanceValidStateSampler.h>
#include <ompl/base/samplers/MaximizeClearanceValidStateSampler.h>

// Planners
#include <path/PathOptimizerKOMO.h>
#include <path/Planner_KOMO.h>
#include <path/PKOMO.h>
#include <path/BITKOMO.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/rrt/LBTRRT.h>
#include <ompl/geometric/planners/informedtrees/BITstar.h>
#include <ompl/geometric/planners/informedtrees/AITstar.h>
#include <ompl/geometric/planners/informedtrees/ABITstar.h>
// #include <ompl/geometric/planners/informedtrees/BITKOMO.h>
#include <ompl/geometric/PathSimplifier.h>

// KOMO
#include <KOMO/komo.h>
#include <Kin/viewer.h>

#define PI 3.1412

unsigned int C_Dimension;

namespace ob = ompl::base;
namespace og = ompl::geometric;

struct ValidityCheckWithKOMO {
	KOMO::Conv_KOMO_SparseNonfactored &nlp;
	ValidityCheckWithKOMO(KOMO::Conv_KOMO_SparseNonfactored &nlp) : nlp(nlp){}
	bool check(const ob::State *state)
	{
		const auto *State = state->as<ob::RealVectorStateSpace::StateType>();

		arr x_query;
		for (unsigned int i = 0; i < C_Dimension; i++){
			x_query.append((*State)[i]);
		}

		arr phi;
		nlp.evaluate(phi, NoArr, x_query);
		double tol = 1e-2;

		return std::abs(phi(0)) < tol;
	}
};

class MyGoalRegion : public ob::GoalRegion
{
public:

    MyGoalRegion(const ob::SpaceInformationPtr &si) : ob::GoalRegion(si)
    {
        setThreshold(1e-4);
    }

    // If I can somehow calculate the distance between endeff and target I am done
    double distanceGoal(const ob::State *state) const override
    {
        // goal region is given by states where x = y
        double R1 = state->as<ob::RealVectorStateSpace::StateType>()->operator[](0);
        double R2 = state->as<ob::RealVectorStateSpace::StateType>()->operator[](1);
        double d = (R1 - R2)*(R1 - R2);
        return d;
    }

};

bool regionSampling(const ob::SpaceInformationPtr &si, const ob::ProblemDefinitionPtr &pd, const ob::GoalLazySamples *gls, ob::State *result, std::string filename)
{
    bool cont = false;

    // Use Quim's method to get final config
    // Check for collision, if collision-free cont = true

    rai::Configuration C;
    C.addFile(filename.c_str());

    KOMO komo;
    komo.setModel(C, true);
    komo.setTiming(1., 1, 1., 1);
	komo.addObjective(
      {}, FS_qItself, {},
      OT_sos); // this regularizes with respect to q = zeros, i.e. term q.T q

    if (filename == "../examples/Models/1_kuka_shelf.g"){
        komo.addObjective({}, FS_positionDiff, {">tool0_joint", "target"}, OT_eq, {10});
	}

    // komo.add_collision(true);
	komo.addObjective(
      {}, FS_accumulatedCollisions, {}, OT_eq,
      {1.}); // add collisions between all bodies with keyword contact
	
	komo.optimize();
    cont = true;

	// check validity
	auto nlp = std::make_shared<KOMO::Conv_KOMO_SparseNonfactored>(komo, false);

	ObjectiveTypeA ot;
	nlp->getFeatureTypes(ot);
	arr phi;
    arr config = komo.getConfiguration_q(0);
	nlp->evaluate(phi, NoArr, config);
	if (phi.first() > 0){
		cont = false;
	}

    // convert arr to state
    std::cout << config << std::endl;
    std::vector<double> reals;
    for (double r : config){
        reals.push_back(r);
    }
    const ob::StateSpace *space(si->getStateSpace().get());
    space->copyFromReals(result, reals);

    if (cont)
    {
        std::cout << "Found goal state: " << std::endl;
        si->printState(result);
    }

    // we continue sampling while we are able to find solutions, we have found not more than 2 previous solutions and we have not yet solved the problem
    return cont && gls->maxSampleCount() < 3 && !pd->hasSolution();
}

ob::ValidStateSamplerPtr allocObstacleBasedVSS(const ob::SpaceInformation *si)
{
    return std::make_shared<ob::ObstacleBasedValidStateSampler>(si);
}

ob::InformedSamplerPtr allocPathLengthInformedSS(const ob::ProblemDefinitionPtr pdef)
{
    return std::make_shared<ob::PathLengthDirectInfSampler>(pdef,(unsigned int)10);
}

ob::ValidStateSamplerPtr allocGaussianVSS(const ob::SpaceInformation *si)
{
    return std::make_shared<ob::GaussianValidStateSampler>(si);
}

ob::ValidStateSamplerPtr allocMinimumClearanceVSS(const ob::SpaceInformation *si)
{
    return std::make_shared<ob::MinimumClearanceValidStateSampler>(si);
}

ob::ValidStateSamplerPtr allocMaximizeClearanceVSS(const ob::SpaceInformation *si)
{
    return std::make_shared<ob::MaximizeClearanceValidStateSampler>(si);
}

void VisualizePath(arrA configs, std::string filename = ""){
	static int Trajectory = 1;
	
	// Create an instance of KOMO
    rai::Configuration C;
    C.addFile(filename.c_str());
    KOMO komo;
    komo.verbose = 0;
    komo.setModel(C, true);
    
    komo.setTiming(1., configs.N, 5., 2);
	komo.add_qControlObjective({}, 1, 1.);

    //use configs to initialize with waypoints
	komo.initWithWaypoints(configs, configs.N, false);
    komo.run_prepare(0);
	komo.plotTrajectory();
	filename.erase(0,19);
	filename.erase(filename.length()-2);
	std::string SaveToPath = std::string("../visualize/videos/") + filename + "/";

	rai::ConfigurationViewer V;
	V.setPath(C, komo.x, "result", true);
	V.playVideo(true, 1., SaveToPath.c_str());
	Trajectory ++;
}

void benchmark(std::string filename = "../examples/Models/1_kuka_shelf.g", std::string planner_ = "BITstar", bool benchmark = false)
{
	// set state validity checking based on KOMO
	rai::Configuration C;
	C.addFile(filename.c_str());
	std::cout << "filename : " << filename << std::endl;
	KOMO komo;
	komo.setModel(C, true);
	komo.setTiming(1, 1, 1, 1);
	komo.addObjective({}, FS_accumulatedCollisions, {}, OT_eq, { 1 });
	komo.run_prepare(0);

	C_Dimension = C.getJointStateDimension();

	//Construct the state space we are planning in
	auto space(std::make_shared<ob::RealVectorStateSpace>(C_Dimension));

	ob::RealVectorBounds bounds(C_Dimension);
	bounds.setLow(-PI);
	bounds.setHigh(PI);
	if (filename == "../examples/Models/3_TwoMobileManipulators.g" || filename == "../examples/Models/8_TwoMobileManipulators_hard.g"){ // Special bounds for this example
		bounds.setLow(0, -1.3); bounds.setLow(1, -0.8);
		bounds.setHigh(0, 1.3); bounds.setHigh(1, 0.8);
		bounds.setLow(10, -1.3); bounds.setLow(11, -0.8);
		bounds.setHigh(10, 1.3); bounds.setHigh(11, 0.8);
	}
	else if (filename == "../examples/Models/9_TwoMobileRobots_hard.g")
	{
		bounds.setLow(0, -1.3); bounds.setLow(1, -0.8);
		bounds.setHigh(0, 1.3); bounds.setHigh(1, 0.8);
		bounds.setLow(3, -1.3); bounds.setLow(4, -0.8);
		bounds.setHigh(3, 1.3); bounds.setHigh(4, 0.8);
	}
	else if (filename == "../examples/Models/10_MobileManipulator.g")
	{
		bounds.setLow(0, -1.3); bounds.setLow(1, -0.8);
		bounds.setHigh(0, 1.3); bounds.setHigh(1, 0.8);
	}
	else if (filename == "../examples/Models/7_disc_rooms.g")
	{
		bounds.setLow(0, -1); bounds.setLow(1, -1.5); bounds.setLow(2, -.05);
		bounds.setHigh(0, 2); bounds.setHigh(1, 1.5); bounds.setHigh(2, .05);
	}

	space->setBounds(bounds);

	//create simple setup
	og::SimpleSetup ss(space);

    // set state validity checking for this space
	auto nlp = std::make_shared<KOMO::Conv_KOMO_SparseNonfactored>(komo, false);
	ValidityCheckWithKOMO checker(*nlp);

	ss.setStateValidityChecker([&checker](const ob::State *state) {
		return checker.check(state);
	});

	// create start and goal states. These states might change from example to example
    ob::ScopedState<> start(space);
	for (unsigned int i=0; i<C.getJointStateDimension(); i++){
	start[i] = komo.getConfiguration_q(0).elem(i);
	}

	ob::ScopedState<> goal(space);

	if (filename == "../examples/Models/1_kuka_shelf.g"){
		// goal = {1.05248, -0.982536, -1.70613, -0.816571, -0.0301295, 0.0453272, 0.000650022};
		// This example has a different start state
		C.setJointState({0.718274, -0.388218, -1.83428, -0.971166, -0.322495, 0.284864, 0.00191594});
		start = {0.718274, -0.388218, -1.83428, -0.971166, -0.322495, 0.284864, 0.00191594};
		goal = {0.560603, -1.05486, -1.71583, -1.68994, -0.051403, 0.266908, 0.000754904};
	}
	else if (filename == "../examples/Models/2_Two_Pandas.g"){
		goal =  {-0.24272, 1.2727, 0.131396, -1.10928, -0.795822, 3.0705, 0.00170469, -0.248182, 1.29032, 0.143824, -1.09332, -0.813384, 3.08292, -0.0161561};
	}
	else if (filename == "../examples/Models/3_TwoMobileManipulators.g"){
		goal = {-0.555762, 0.000540429, 1.57074, 0.00188429, 0.764456, -0.000160723, -2.21317, -0.00321155, 2.28468, -0.000332939, 0.555647, -0.00012235, -1.57154, 0.00161455, 0.764632, -0.000429018, -2.21257, 0.00103216, 2.28374, 0.00102819};
	}	
	else if (filename == "../examples/Models/4_kuka_box.g"){
		goal = {0.00241061, 0.872391, -0.00871117, -2.09305, -0.0101917, 0.314683, 0.000963466};
	}
	else if (filename == "../examples/Models/5_disc_obstacle.g"){
		goal = {0.8,-0.25,0};
	}
	else if (filename == "../examples/Models/6_rectangle_opening.g"){
		goal = {0.7,0,0};
	}
	else if (filename == "../examples/Models/7_disc_rooms.g"){
		goal = {0.7,0,0};
	}
	else if (filename == "../examples/Models/8_TwoMobileManipulators_hard.g"){
		goal = {-0.555762, 0.000540429, 1.57074, 0.00188429, 0.764456, -0.000160723, -2.21317, -0.00321155, 2.28468, -0.000332939, 0.555647, -0.00012235, -1.57154, 0.00161455, 0.764632, -0.000429018, -2.21257, 0.00103216, 2.28374, 0.00102819};
	}
	else if (filename == "../examples/Models/9_TwoMobileRobots_hard.g"){
		goal = {-0.755762, 0.000540429, 1.57074, 0.755647, -0.00012235, -1.57154};
	}
	else if (filename == "../examples/Models/10_MobileManipulator.g"){
		goal = {-0.555762, 0.000540429, 1.57074, 0.00188429, 0.764456, -0.000160723, -2.21317, -0.00321155, 2.28468, -0.000332939};
	}
	else{// Default goal
		for (unsigned int i=0; i<C.getJointStateDimension(); i++){
			if (i>3){continue;}
			goal[i] = komo.getConfiguration_q(0).elem(i)+1.5;
		}
	}

	arr goal_;
	for (unsigned int i=0; i<C.getJointStateDimension(); i++){
		goal_.append(goal[i]);
	}

	std::cout << goal << std::endl;

    // Set the start and goal states
    ss.setStartAndGoalStates(start, goal);

	if(benchmark)
	{
		// First we create a benchmark class:
		ompl::tools::Benchmark b(ss, "my experiment");

		auto si = ss.getSpaceInformation();
		std::vector<ob::SpaceInformationPtr> siVec;
		siVec.push_back(si);
		
		if (planner_ == "RRTstar"){
			auto planner1(std::make_shared<og::RRTstar>(si));
			b.addPlanner(planner1);
		}
		else if (planner_ == "BITstar"){
			auto planner1(std::make_shared<og::BITstar>(si));
			b.addPlanner(planner1);
		}
		else if (planner_ == "ABITstar"){
			auto planner1(std::make_shared<og::ABITstar>(si));
			b.addPlanner(planner1);
		}
		else if (planner_ == "AITstar"){
			auto planner1(std::make_shared<og::AITstar>(si));
			b.addPlanner(planner1);
		}
		else if (planner_ == "FMT"){
			auto planner1(std::make_shared<og::FMT>(si));
			b.addPlanner(planner1);
		}
		else if (planner_ == "LBTRRT"){
			auto planner1(std::make_shared<og::LBTRRT>(si));
			b.addPlanner(planner1);
		}
		else if (planner_ == "PKOMO"){
			auto planner1(std::make_shared<og::PKOMO>(si,filename));
			b.addPlanner(planner1);
		}
		else
		{
			//build the KOMO object here:
			auto komo_(std::make_shared<KOMO>());
			komo_->verbose = 0;
			komo_->setModel(C, true);
			
			komo_->setTiming(1., 20, 1., 2);
			komo_->add_qControlObjective({}, 1, 2.);

			// arrA Goal_;
			// Goal_.append(goal_);
			// komo_->initWithWaypoints(Goal_, 1, false);

			komo_->addObjective({1.}, FS_qItself, {}, OT_eq, {100}, goal_, 0);
			if (filename == "../examples/Models/7_disc_rooms.g")
				komo_->add_collision(true, 0.05);
			else
				komo_->add_collision(true, 0.05);

			if (planner_ == "KOMO"){
				auto planner(std::make_shared<og::Planner_KOMO>(si,komo_));
				b.addPlanner(planner);
			}
			else if (planner_ == "BITKOMO"){
				auto planner(std::make_shared<og::BITKOMO>(si));
				og::PathOptimizerPtr optimizer = std::make_shared<og::PathOptimizerKOMO>(si,komo_);
				planner->setOptimizer(optimizer);
				b.addPlanner(planner);
			}
			else if (planner_ == "validBITKOMO"){
				auto planner(std::make_shared<og::BITKOMO>(si,"validBITKOMO"));
				og::PathOptimizerPtr optimizer = std::make_shared<og::PathOptimizerKOMO>(si,komo_);
				planner->setOptimizer(optimizer);
				b.addPlanner(planner);
			}
		}

		ompl::tools::Benchmark::Request req;
		req.maxTime = 10.0;
		// req.maxMem = 100.0;
		req.runCount = 50;
		req.displayProgress = true;
		b.benchmark(req);

		//TODO: add postrunevent ompl - to make sure the path is feasible
		
		// This will generate a .log file
		std::ostringstream oss;
		std::string filename_s(filename);
		filename_s.erase(0,19);
		filename_s.erase(filename_s.length()-2);
		oss << "data/Benchmarks/" << filename_s << "/logs/benchmark_" << planner_ << ".log";
		// oss << "data/Benchmarks/test" << filename_s << "/logs/benchmark_" << planner_ << ".log";
		b.saveResultsToFile(oss.str().c_str());
	}

	else{
		auto si = ss.getSpaceInformation();

		if(planner_ == "RRTstar"){
			auto planner(std::make_shared<og::RRTstar>(si));
			ss.setPlanner(planner);
		}
		else if(planner_ == "BITstar"){
			auto planner(std::make_shared<og::BITstar>(si));
			ss.setPlanner(planner);
		}
		else if(planner_ == "ABITstar"){
			auto planner(std::make_shared<og::ABITstar>(si));
			ss.setPlanner(planner);
		}
		else if(planner_ == "AITstar"){
			auto planner(std::make_shared<og::AITstar>(si));
			ss.setPlanner(planner);
		}
		else if(planner_ == "FMT"){
			auto planner(std::make_shared<og::FMT>(si));
			ss.setPlanner(planner);
		}
		else if(planner_ == "LBTRRT"){
			auto planner(std::make_shared<og::LBTRRT>(si));
			ss.setPlanner(planner);
		}
		else if(planner_ == "PKOMO"){
			auto planner(std::make_shared<og::PKOMO>(si, filename));
			ss.setPlanner(planner);
		}
		else{
			//build the KOMO object here:
			auto komo_(std::make_shared<KOMO>());
			komo_->verbose = 0;
			komo_->setModel(C, true);
			// C.watch(true);
			
			komo_->setTiming(1., 20, 1., 2);
			komo_->add_qControlObjective({}, 1, 2);

			// arrA Goal_;
			// Goal_.append(goal_);
			// komo_->initWithWaypoints(Goal_, 1, false);

			komo_->addObjective({1.}, FS_qItself, {}, OT_eq, {100}, goal_, 0);
			if (filename == "../examples/Models/7_disc_rooms.g")
				komo_->add_collision(true, 0.05);
			else
				komo_->add_collision(true, 0.05);

			if(planner_ == "KOMO"){
				auto planner(std::make_shared<og::Planner_KOMO>(si, komo_));
				ss.setPlanner(planner);
			}
			else if(planner_ == "BITKOMO"){
				auto planner(std::make_shared<og::BITKOMO>(si));
				og::PathOptimizerPtr optimizer = std::make_shared<og::PathOptimizerKOMO>(si,komo_);
				planner->setOptimizer(optimizer);
				ss.setPlanner(planner);
			}
			else if(planner_ == "validBITKOMO"){
				auto planner(std::make_shared<og::BITKOMO>(si,"validBITKOMO"));
				og::PathOptimizerPtr optimizer = std::make_shared<og::PathOptimizerKOMO>(si,komo_);
				planner->setOptimizer(optimizer);
				ss.setPlanner(planner);
			}
		}
		
		ss.setup();

		// attempt to solve the problem
		ob::PlannerStatus solved = ss.solve(10.0);

		if (solved == ob::PlannerStatus::StatusType::APPROXIMATE_SOLUTION)
			std::cout << "Found solution: APPROXIMATE_SOLUTION" << std::endl;
		else if (solved == ob::PlannerStatus::StatusType::EXACT_SOLUTION)
			std::cout << "Found solution: EXACT_SOLUTION" << std::endl;
		else if (solved == ob::PlannerStatus::StatusType::TIMEOUT)
			std::cout << "Found solution: TIMEOUT" << std::endl;
		else{
			std::cout << "No solution found: Invalid " << std::endl;
			return;
		}
	
		// This is for visualization of paths from other planners
		auto path = ss.getSolutionPath();
		arrA configs;
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
		//Visualize in KOMO
		VisualizePath(configs, filename);

	}
}

void plan()
{
	std::string filename = "../examples/Models/1_kuka_shelf.g";
	std::string planner_ = "BITstar";
	// set state validity checking based on KOMO
	rai::Configuration C;
	C.addFile(filename.c_str());
	std::cout << "filename : " << filename << std::endl;
	KOMO komo;
	komo.setModel(C, true);
	komo.setTiming(1, 1, 1, 1);
	komo.addObjective({}, FS_accumulatedCollisions, {}, OT_eq, { 1 });
	komo.run_prepare(0);

	C_Dimension = C.getJointStateDimension();

	//Construct the state space we are planning in
	auto space(std::make_shared<ob::RealVectorStateSpace>(C_Dimension));

	ob::RealVectorBounds bounds(C_Dimension);
	bounds.setLow(-PI);
	bounds.setHigh(PI);
	space->setBounds(bounds);

	//create simple setup
	og::SimpleSetup ss(space);

    // set state validity checking for this space
	auto nlp = std::make_shared<KOMO::Conv_KOMO_SparseNonfactored>(komo, false);
	ValidityCheckWithKOMO checker(*nlp);

	ss.setStateValidityChecker([&checker](const ob::State *state) {
		return checker.check(state);
	});

	// create start and goal states. These states might change from example to example
    ob::ScopedState<> start(space);
	for (unsigned int i=0; i<C.getJointStateDimension(); i++){
	start[i] = komo.getConfiguration_q(0).elem(i);
	}

	ss.setStartState(start);

	// define our goal region
    MyGoalRegion region(ss.getSpaceInformation());

    // bind a sampling function that fills its argument with a sampled state
    // and returns true while it can produce new samples we don't need to
    // check if new samples are different from ones previously computed as
    // this is pefromed automatically by GoalLazySamples
    ob::GoalSamplingFn samplingFunction = [&ss, &region, filename](const ob::GoalLazySamples *gls, ob::State *result)
        {
            return regionSampling(ss.getSpaceInformation(), ss.getProblemDefinition(),
                gls, result, filename);
        };

    // create an instance of GoalLazySamples:
    auto goal(std::make_shared<ob::GoalLazySamples>(ss.getSpaceInformation(), samplingFunction));

    // we set a goal that is sampleable, but it in fact corresponds to a region that is not sampleable by default
    ss.setGoal(goal);

	//Set the planner
    ss.setPlanner(std::make_shared<ompl::geometric::BITstar>(ss.getSpaceInformation()));

    // attempt to solve the problem
    ob::PlannerStatus solved = ss.solve(10.0);

    if (solved)
    {
        std::cout << "Found solution:" << std::endl;
        // print the path to screen
        ss.simplifySolution();
        ss.getSolutionPath().print(std::cout);
    }
    else
        std::cout << "No solution found" << std::endl;

    // the region variable will now go out of scope. To make sure it is not used in the sampling function any more
    // (i.e., the sampling thread was able to terminate), we make sure sampling has terminated
    goal->as<ob::GoalLazySamples>()->stopSampling();
}

int main(int argc, char ** argv)
{
  	rai::initCmdLine(argc,argv);
	std::cout << "OMPL version: " << OMPL_VERSION << std::endl;
	if (argc<2){
		plan();
		// benchmark();
	}
	else{
		std::string filename = argv[1];
		std::string planner_ = argv[2];
		std::string b = argv[3];
		bool benchmark_ = (b == "true");
		benchmark(filename, planner_, benchmark_);
	}
	return 0;
}
