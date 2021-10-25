//
// Created by Jay: 13 oct 2021
//

#include"Optimizer.h"

struct ValidityCheckWithKOMO {
	KOMO::Conv_KOMO_SparseNonfactored &nlp;
	ValidityCheckWithKOMO(KOMO::Conv_KOMO_SparseNonfactored &nlp) : nlp(nlp){}
	bool check(const ob::State *state)
	{
		const auto *State = state->as<ob::RealVectorStateSpace::StateType>();

		arr x_query;
		for (unsigned int i = 0; i < CSpace_Dimension; i++){
			x_query.append((*State)[i]);
		}

		arr phi;
		nlp.evaluate(phi, NoArr, x_query);
		double tol = 1e-2;

		return std::abs(phi(0)) < tol;
	}
};

void Optimizer::VisualizePath(arrA configs){
	static int Trajectory = 1;
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
    ss.setStartAndGoalStates(this->startPosition, this->goalPosition);

	return ss;
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

	for (unsigned int i=0; i<C.getJointStateDimension(); i++){
		this->startPosition[i] = komo.getConfiguration_q(0).elem(i);
	}

	auto ss = Optimizer::createSimpleSetup(checker);

	std::cout << ss.getGoal() << std::endl;

}

void Optimizer::setConfigurationFilename(std::string filename){
	ifstream MyReadFile(filename);
	getline (MyReadFile, filename);
	MyReadFile.close();
}