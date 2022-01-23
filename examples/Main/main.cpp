#include <iostream>

// Basic Requirements
#include <ompl/config.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/tools/benchmark/Benchmark.h>

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
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/rrt/LBTRRT.h>
#include <ompl/geometric/planners/informedtrees/BITstar.h>
#include <ompl/geometric/planners/informedtrees/AITstar.h>
#include <ompl/geometric/planners/informedtrees/ABITstar.h>
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
	std::string SaveToPath = std::string("z.vid/DisplayTrajectory_") + std::to_string(Trajectory) + "/";

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
	space->setBounds(bounds);

	//create simple setup
	og::SimpleSetup ss(space);

    // set state validity checking for this space
	auto nlp = std::make_shared<KOMO::Conv_KOMO_SparseNonfactored>(komo, false);
	ValidityCheckWithKOMO checker(*nlp);

	ss.setStateValidityChecker([&checker](const ob::State *state) {
		return checker.check(state);
	});

    // create a start state
    ob::ScopedState<> start(space);
	for (unsigned int i=0; i<C.getJointStateDimension(); i++){
		start[i] = komo.getConfiguration_q(0).elem(i);
	}

	std::cout << start << std::endl;

	// goal state will change from example to example
	ob::ScopedState<> goal(space);
	if (filename == "../examples/Models/1_kuka_shelf.g"){
		goal = {1.05248, -0.982536, -1.70613, -0.816571, -0.0301295, 0.0453272, 0.000650022};
	}
	else if (filename == "../examples/Models/2_Two_Pandas.g"){
		goal = {-0.348907, 1.34251, 0.464246, -0.888288, -0.462429, 2.66979, -0.0416295, 0.0134267, -0.350246, 1.3497, 0.486382, -0.897415, -0.482546, 2.6686, -0.0623223, 0.0197671};
	}
	else if (filename == "../examples/Models/3_TwoMobileManipulators.g"){
		goal = {-0.605107, 0.0441026, 1.59182, 0.0554067, 0.877096, 0.020064, -2.19427, 0.130015, 2.3168, 0.132783, 0.15148, 0.612561, -0.0548847, -1.52128, 0.0674673, 0.873418, 0.0154376, -2.20752, 0.0258739, 2.31415, 0.00219257, 0.0930736};
	}	
	else if (filename == "../examples/Models/4_kuka_box.g"){
		goal = {0.00241061, 0.872391, -0.00871117, -2.09305, -0.0101917, 0.314683, 0.000963466};
		// goal = {-0.00769556, 0.729782, -0.000104823, -2.1528, 0.0961055, 0.173696, 0.000571979};
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
		if (planner_ == "BITstar"){
			auto planner1(std::make_shared<og::BITstar>(si));
			b.addPlanner(planner1);
		}
		if (planner_ == "ABITstar"){
			auto planner1(std::make_shared<og::ABITstar>(si));
			b.addPlanner(planner1);
		}
		if (planner_ == "AITstar"){
			auto planner1(std::make_shared<og::AITstar>(si));
			b.addPlanner(planner1);
		}
		if (planner_ == "FMT"){
			auto planner1(std::make_shared<og::FMT>(si));
			b.addPlanner(planner1);
		}
		if (planner_ == "LBTRRT"){
			auto planner1(std::make_shared<og::LBTRRT>(si));
			b.addPlanner(planner1);
		}
		if (planner_ == "PKOMO"){
			auto planner1(std::make_shared<og::PKOMO>(si,filename));
			b.addPlanner(planner1);
			std::cout << "planner added to benchmark" << std::endl;
		}
		if (planner_ == "KOMO")
		{
			//build the KOMO object here:
			auto komo_(std::make_shared<KOMO>());
			komo_->verbose = 0;
			komo_->setModel(C, true);
			
			komo_->setTiming(1., 20, 1., 2);
			// komo_->add_qControlObjective({}, 1, 2.);
			komo_->add_qControlObjective({}, 1, 1.);

			komo_->addObjective({1.}, FS_qItself, {}, OT_eq, {1000}, goal_, 0);
			// komo_->addObjective({}, FS_qItself, {}, OT_sos, {1.}, {}, 1);
			komo_->add_collision(true);

			if (planner_ == "KOMO"){
				auto planner(std::make_shared<og::Planner_KOMO>(si,komo_));
				b.addPlanner(planner);
			}
		}

		ompl::tools::Benchmark::Request req;
		req.maxTime = 10.0;
		req.maxMem = 100.0;
		req.runCount = 10;
		req.displayProgress = true;
		b.benchmark(req);

		//TODO: add postrunevent ompl - to make sure the path is feasible
		
		// This will generate a .log file
		std::ostringstream oss;
		oss << "data/Benchmarks/benchmark_" << planner_ << ".log";
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
			
			komo_->setTiming(1., 20, 5., 2);
			komo_->add_qControlObjective({}, 1, 2.);

			komo_->addObjective({1.}, FS_qItself, {}, OT_eq, {10}, goal_, 0);
			komo_->add_collision(true);

			if(planner_ == "KOMO"){
				auto planner(std::make_shared<og::Planner_KOMO>(si, komo_));
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

int main(int argc, char ** argv)
{
  	rai::initCmdLine(argc,argv);
	std::cout << "OMPL version: " << OMPL_VERSION << std::endl;
	if (argc<2){
		benchmark();
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
