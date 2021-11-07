//
// Created by Jay: 13 oct 2021
//

#include"Optimizer.h"

Optimizer::Optimizer(){
}

void Optimizer::VisualizePath(arrA configs){
	// static int Trajectory = 1;
	// setup KOMO
    rai::Configuration C;
    C.addFile(filename.c_str());
    KOMO komo;
    komo.verbose = 0;
    komo.setModel(C, true);
    
    komo.setTiming(1., configs.N, 5., 2);
	komo.add_qControlObjective({}, 1, 1.);

    // std::cout << configs << std::endl;
    komo.addObjective({1.}, FS_qItself, {}, OT_eq, {10}, configs(configs.N-1), 0);
    komo.addObjective({}, FS_accumulatedCollisions, {}, OT_eq, {1.});
    komo.add_collision(true);

	std::cout << configs << std::endl;

    //use configs to initialize with waypoints
	komo.initWithWaypoints(configs, configs.N, false); // no clue why this isn't working
    komo.run_prepare(0);
	komo.plotTrajectory();
	// std::string SaveToPath = std::string("../visualization/DisplayTrajectory_") + std::to_string(Trajectory) + "/";

	rai::ConfigurationViewer V;
	V.setPath(C, komo.x, "result", true);
	V.playVideo(true, 1./* , SaveToPath.c_str() */);
	// Trajectory ++;
}

og::SimpleSetup Optimizer::createSimpleSetup(ValidityCheckWithKOMO checker)
{
	//Construct the state space we are planning in
	auto space(std::make_shared<ob::RealVectorStateSpace>(CSpace_Dimension));

	ob::RealVectorBounds bounds(CSpace_Dimension);
	bounds.setLow(-PI);
	bounds.setHigh(PI);
	space->setBounds(bounds);

	//create simple setup
	og::SimpleSetup ss(space);

	ss.setStateValidityChecker([&checker](const ob::State *state) {
		return checker.check(state);
	});

	//can put samplers here

    // Set the start and goal states
	ob::ScopedState<> start(space);

    ob::ScopedState<> goal(space);
	goal = {2,0};

    ss.setStartAndGoalStates(start, goal);

	return ss;
}

void Optimizer::visualize_random()
{
	rai::Configuration C;
	C.addFile(filename.c_str());
	KOMO komo;
	komo.setModel(C, true);
	komo.setTiming(1, 1, 1, 1);
	komo.addObjective({}, FS_accumulatedCollisions, {}, OT_eq, { 1 });
	komo.run_prepare(0);

	auto nlp = std::make_shared<KOMO::Conv_KOMO_SparseNonfactored>(komo,false);

	// ObjectiveTypeA ot;
	// nlp->getFeatureTypes(ot);
	// komo.view(true);

	arr phi;
	size_t N = 20;
	for (size_t i = 0; i < N; i++) {
		arr pose = rand(2);
		C.setJointState(pose);
		nlp->evaluate(phi, NoArr, pose);
		std::cout << pose << " " << phi << std::endl;
		// std::cout << komo.getConfiguration_q(0) << std::endl;
		komo.view(true);
	}
}

void Optimizer::plan(){
	//setup KOMO
	rai::Configuration C;
	C.addFile(filename.c_str());
	KOMO komo;
	komo.setModel(C, true);
	komo.setTiming(1, 1, 1, 1);
	komo.addObjective({}, FS_accumulatedCollisions, {}, OT_eq, { 1 });
	komo.run_prepare(0);

	CSpace_Dimension = C.getJointStateDimension();

	auto nlp = std::make_shared<KOMO::Conv_KOMO_SparseNonfactored>(komo, false);

	ValidityCheckWithKOMO checker(*nlp);

	for (unsigned int i=0; i<CSpace_Dimension; i++){
		startPosition.push_back(komo.getConfiguration_q(0).elem(i));
	}

	//Construct the state space we are planning in
	auto space(std::make_shared<ob::RealVectorStateSpace>(CSpace_Dimension));

	ob::RealVectorBounds bounds(CSpace_Dimension);
	bounds.setLow(-PI);
	bounds.setHigh(PI);
	space->setBounds(bounds);

	// auto ss = Optimizer::createSimpleSetup(checker);
	og::SimpleSetup ss(space);

	ss.setStateValidityChecker([&checker](const ob::State *state) {
		return checker.check(state);
	});

	ob::ScopedState<> start(space);

    ob::ScopedState<> goal(space);
	goal = {2,0};

    ss.setStartAndGoalStates(start, goal);	

	auto si = ss.getSpaceInformation();
	// std::vector<ob::SpaceInformationPtr> siVec;
	// siVec.push_back(si);
	// auto planner1 = std::make_shared<om::LocalMinimaSpanners>(siVec);

	if (planner == pathOptimizerKOMO){
		// og::PathOptimizerPtr optimizer = std::make_shared<og::PathOptimizerKOMO>(si);
		// planner1->setOptimizer(optimizer);
		// ss.setPlanner(planner1);
	}
	else{
		auto planner2(std::make_shared<og::RRTstar>(si));
		ss.setPlanner(planner2);
	}
	
	ss.setup();

	// attempt to solve the problem
	ob::PlannerStatus solved = ss.solve(solveTime);

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

	if(planner == pathOptimizerKOMO){ //This code is for visualization of the paths from PathOptimizer
		// auto localMinimaTree = planner1->getLocalMinimaTree();
		// int NumberOfMinima =  (int)localMinimaTree->getNumberOfMinima();
		// int NumberOfLevels =  (int)localMinimaTree->getNumberOfLevel();

		// for (int i=0; i<NumberOfLevels; i++){
		// 	for (int j=0; j<NumberOfMinima; j++){
		// 		std::cout << "\nNew path[" << i << j+1 << "] \n" << std::endl;
		// 		auto path = std::dynamic_pointer_cast<ompl::geometric::PathGeometric>(localMinimaTree->getPath(i,j)->asPathPtr());
		// 		//convert path to arrA
		// 		arrA configs;
		// 		for (auto state : (*path).getStates())
		// 		{
		// 			arr config;
		// 			std::vector<double> reals;
		// 			space->copyToReals(reals, state); //Error: This line does not work!
		// 			std::cout << reals.size() << "size" << std::endl;;
		// 			for (double r : reals){
		// 				config.append(r);
		// 			}
		// 			configs.append(config);
		// 		}
		// 		//Visualize in KOMO
		// 		VisualizePath(configs);
		// 		std::dynamic_pointer_cast<ompl::geometric::PathGeometric>(localMinimaTree->getPath(i,j)->asPathPtr())->print(std::cout);
		// 	}
		// }
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
		VisualizePath(configs);
	}
}

void Optimizer::setConfigurationFilename(std::string filename){
	ifstream MyReadFile(filename);
	getline (MyReadFile, this->filename);
	MyReadFile.close();
}