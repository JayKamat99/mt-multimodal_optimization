#include <iostream>
#include <functional>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/multilevel/planners/multimodal/LocalMinimaSpanners.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/tools/benchmark/Benchmark.h>
#include <ompl/base/objectives/PathLengthSquaredOptimizationObjective.h>

#include <ompl/base/samplers/ObstacleBasedValidStateSampler.h>
#include <ompl/base/samplers/GaussianValidStateSampler.h>
#include <ompl/base/samplers/MinimumClearanceValidStateSampler.h>
#include <ompl/base/samplers/MaximizeClearanceValidStateSampler.h>

// Planners
#include <path/PathOptimizerKOMO.h>
#include <path/Planner_KOMO.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/rrt/LBTRRT.h>
#include <ompl/geometric/planners/informedtrees/BITstar.h>
#include <ompl/geometric/PathSimplifier.h>

#include <ompl/config.h>

#include <KOMO/komo.h>
#include <Kin/viewer.h>

#include <chrono>

#define PI 3.1412

unsigned int C_Dimension;

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace om = ompl::multilevel;

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

    // komo.addObjective({1.}, FS_qItself, {}, OT_eq, {10}, configs(configs.N-1), 0);
    // komo.addObjective({}, FS_accumulatedCollisions, {}, OT_eq, {1.});
    // komo.add_collision(true);

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

void benchmark(std::string filename = "../examples/Models/2D_arm.g", std::string planner_ = "PathOptimizerKOMO", bool benchmark = false)
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
	// std::cout << "C_Dimension: " << C_Dimension << std::endl;
	// return;

	// This must chnage from example to example
	uint stepsPerPhase_ = 15;

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

	// ss.getSpaceInformation()->setValidStateSamplerAllocator(allocObstacleBasedVSS);
	// ss.getSpaceInformation()->setValidStateSamplerAllocator(allocGaussianVSS);
	// ss.getSpaceInformation()->setValidStateSamplerAllocator(allocMinimumClearanceVSS);
	// ss.getSpaceInformation()->setValidStateSamplerAllocator(allocMaximizeClearanceVSS);

    // create a start state
    ob::ScopedState<> start(space);
	for (unsigned int i=0; i<C.getJointStateDimension(); i++){
		start[i] = komo.getConfiguration_q(0).elem(i);
	}

	std::cout << start << std::endl;

	// goal state will change from example to example
	ob::ScopedState<> goal(space);
	if (filename == "../examples/Models/2_Two_Pandas.g"){
		// goal = {-0.119527, -0.715569, -0.0174884, -1.8619, -0.0564344, 2.10074, -0.00834383, 0.0469022, -0.110201, -0.70319, -0.00590797, -1.87372, -0.0644692, 2.10377, -0.0100299, 0.0466833};
		// goal = {-0.375717, 1.04588, 0.337007, -1.04422, -0.401061, 2.59755, -0.0385004, 0.0195842, -0.367046, 1.06017, 0.364706, -1.05986, -0.423203, 2.59865, -0.0562032, 0.0240985};
		goal = {-0.348907, 1.34251, 0.464246, -0.888288, -0.462429, 2.66979, -0.0416295, 0.0134267, -0.350246, 1.3497, 0.486382, -0.897415, -0.482546, 2.6686, -0.0623223, 0.0197671};
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

	//set Optimization Objective
	// ompl::base::OptimizationObjectivePtr opt_ = std::make_shared<ompl::base::PathLengthSquaredOptimizationObjective>(ss.getSpaceInformation());
	// ss.setOptimizationObjective(opt_);

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
		}if (planner_ == "FMT"){
			auto planner1(std::make_shared<og::FMT>(si));
			b.addPlanner(planner1);
		}if (planner_ == "LBTRRT"){
			auto planner1(std::make_shared<og::LBTRRT>(si));
			b.addPlanner(planner1);
		}
		if (planner_ == "PathSimplifier"){
			auto planner(std::make_shared<om::LocalMinimaSpanners>(siVec));
			og::PathOptimizerPtr optimizer = std::make_shared<og::PathSimplifier>(si);
			planner->setOptimizer(optimizer);
			b.addPlanner(planner);
		}
		if (planner_ == "PathOptimizerKOMO" || "KOMO")
		{
			//build the KOMO object here:
			auto komo_(std::make_shared<KOMO>());
			komo_->verbose = 0;
			komo_->setModel(C, true);
			
			komo_->setTiming(1., stepsPerPhase_, 5., 2);
			// komo_->add_qControlObjective({}, 1, 2.);
			komo_->add_qControlObjective({}, 1, 1.);

			komo_->addObjective({1.}, FS_qItself, {}, OT_eq, {1000}, goal_, 0);
			komo_->addObjective({}, FS_qItself, {}, OT_sos, {1.}, {}, 1);
			komo_->add_collision(true);

			if (planner_ == "PathOptimizerKOMO"){
				auto planner(std::make_shared<om::LocalMinimaSpanners>(siVec));
				og::PathOptimizerPtr optimizer = std::make_shared<og::PathOptimizerKOMO>(si,komo_);
				planner->setOptimizer(optimizer);
				b.addPlanner(planner);
			}
			if (planner_ == "KOMO"){
				auto planner(std::make_shared<og::Planner_KOMO>(si,komo_));
				b.addPlanner(planner);
			}
		}

		ompl::tools::Benchmark::Request req;
		req.maxTime = 10.0;
		req.maxMem = 100.0;
		req.runCount = 100;
		req.displayProgress = true;
		b.benchmark(req);
		
		// This will generate a .log file
		std::ostringstream oss;
		oss << "data/Benchmarks/benchmark_" << planner_ << ".log";
		b.saveResultsToFile(oss.str().c_str());
	}

	else{
		auto si = ss.getSpaceInformation();
		std::vector<ob::SpaceInformationPtr> siVec;
		siVec.push_back(si);
		auto planner1 = std::make_shared<om::LocalMinimaSpanners>(siVec);

		if(planner_ == "RRTstar"){
			auto planner(std::make_shared<og::RRTstar>(si));
			ss.setPlanner(planner);
		}
		else if(planner_ == "BITstar"){
			auto planner(std::make_shared<og::BITstar>(si));
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
		else if (planner_ == "PathSimplifier"){
			og::PathOptimizerPtr optimizer = std::make_shared<og::PathSimplifier>(si);
			planner1->setOptimizer(optimizer);
			ss.setPlanner(planner1);
		}
		else{
			//build the KOMO object here:
			auto komo_(std::make_shared<KOMO>());
			komo_->verbose = 0;
			komo_->setModel(C, true);
			
			komo_->setTiming(1., stepsPerPhase_, 5., 2);
			komo_->add_qControlObjective({}, 1, 2.);

			komo_->addObjective({1.}, FS_qItself, {}, OT_eq, {10}, goal_, 0);
			komo_->addObjective({1.}, FS_qItself, {}, OT_sos, {1}, {}, 0);
			// komo_->addObjective({}, FS_accumulatedCollisions, {}, OT_eq, {1.});
			komo_->add_collision(true);

			if (planner_ == "PathOptimizerKOMO"){
				og::PathOptimizerPtr optimizer = std::make_shared<og::PathOptimizerKOMO>(si,komo_);
				planner1->setOptimizer(optimizer);
				ss.setPlanner(planner1);
			}
			else if(planner_ == "KOMO"){
				auto planner(std::make_shared<og::Planner_KOMO>(si, komo_));
				ss.setPlanner(planner);
			}
		}
		
		ss.setup();

		// attempt to solve the problem

		auto startTime = std::chrono::system_clock::now();
		ob::PlannerStatus solved = ss.solve(60.0);

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

		auto endTime = std::chrono::system_clock::now();
	
		if(planner_ == "PathOptimizerKOMO" || planner_ == "PathSimplifier"){ //This code is for visualization of the paths from PathOptimizer
			auto localMinimaTree = planner1->getLocalMinimaTree();
			int NumberOfMinima =  (int)localMinimaTree->getNumberOfMinima();
			int NumberOfLevels =  (int)localMinimaTree->getNumberOfLevel();

		// ofstream myfile;
		// myfile.open ("PathOptimizerKOMO_035_times.txt", std::ios_base::app);
		// myfile << std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count() << ","<< NumberOfMinima <<"\n";
		// myfile.close();
		
		// return;
			for (int i=0; i<NumberOfLevels; i++){
				for (int j=0; j<NumberOfMinima; j++){
					std::cout << "\nNew path[" << i << j+1 << "] \n" << std::endl;
					auto path = std::dynamic_pointer_cast<ompl::geometric::PathGeometric>(localMinimaTree->getPath(i,j)->asPathPtr());
					//convert path to arrA
					arrA configs;
					for (auto state : (*path).getStates())
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
					std::dynamic_pointer_cast<ompl::geometric::PathGeometric>(localMinimaTree->getPath(i,j)->asPathPtr())->print(std::cout);
				}
			}
		}

		else{// This is for visualization of paths from other planners
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
		// for (int i = 0; i<50; i++){
		benchmark(filename, planner_, benchmark_);
		// }
	}
	return 0;
}
