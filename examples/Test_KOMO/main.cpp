// The goal of this program is to read the trajectory and optimize the path.

#include <KOMO/komo.h>
#include <Kin/viewer.h>
#include <fstream>

#include <ompl/config.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <path/PathOptimizerKOMO.h>
#include <ompl/geometric/PathOptimizer.h>

#define PI 3.142

namespace ob = ompl::base;

uint C_Dimension;

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

void optimize(std::string filename){
    // // set state validity checking based on KOMO
	// rai::Configuration C;
	// C.addFile(filename.c_str());
	// std::cout << "filename : " << filename << std::endl;
	// KOMO komo;
	// komo.setModel(C, true);
	// komo.setTiming(1, 1, 1, 1);
	// komo.addObjective({}, FS_accumulatedCollisions, {}, OT_eq, { 1 });
	// komo.run_prepare(0);

	// C_Dimension = C.getJointStateDimension();

	// //Construct the state space we are planning in
	// auto space(std::make_shared<ob::RealVectorStateSpace>(C_Dimension));

	// ob::RealVectorBounds bounds(C_Dimension);
	// bounds.setLow(-PI);
	// bounds.setHigh(PI);
	// space->setBounds(bounds);

	// //create simple setup
	// og::SimpleSetup ss(space);

    // // set state validity checking for this space
	// auto nlp = std::make_shared<KOMO::Conv_KOMO_SparseNonfactored>(komo, false);
	// ValidityCheckWithKOMO checker(*nlp);

	// ss.setStateValidityChecker([&checker](const ob::State *state) {
	// 	return checker.check(state);
	// });

    // Load the configuration
    rai::Configuration C;
    C.addFile(filename.c_str());
    cout <<"configuration space dim=" <<C_Dimension <<endl;
    std::string file;
    arr goal_;

    if (filename == "../examples/Models/2_Two_Pandas.g"){
        file = "../debug/initPathConfigs.txt";
        // file = "../debug/finalPathConfigs.txt";
        // goal_ = {-0.348907, 1.34251, 0.464246, -0.888288, -0.462429, 2.66979, -0.0416295, 0.0134267, -0.350246, 1.3497, 0.486382, -0.897415, -0.482546, 2.6686, -0.0623223, 0.0197671, 0, 0};
    }
    else if (filename == "../examples/Models/2D_bot.g"){
        file = "../debug/7_configs.txt";
        goal_ = {0,1,0};
    }
    else if (filename == "../examples/Models/7_disc_rooms.g"){
        file = "../debug/2d_debug.txt";
        goal_ = {0.7,0,0};
    }
    else if (filename == "../examples/Models/1_kuka_shelf.g"){
        file = "../debug/2d_debug.txt";
		goal_ = {1.05248, -0.982536, -1.70613, -0.816571, -0.0301295, 0.0453272, 0.000650022};
    }

    std::ifstream myfile;
    arrA configs;
    myfile.open (file.c_str());
    myfile >> configs;
    myfile.close();

    // std::cout << "Configs: " << configs << std::endl;
    // goal_ = configs(configs.N-1);
    // std::cout << "Goal: " << goal_ << std::endl;

    // auto komo_ = std::make_shared<KOMO>();
    // komo_->setModel(C, true);
    // komo_->setTiming(1., 20 /* configs.N */, 1, 2);
    // komo_->add_qControlObjective({}, 2, 1.);
    // komo_->addObjective({1.}, FS_qItself, {}, OT_eq, {10}, goal_, 0);
    // komo_->add_collision(true);

    // // Instead of building running komo.optimize here run it using path optimizer.
    // auto si = ss.getSpaceInformation();
	// auto optimizer = std::make_shared<og::PathOptimizerKOMO>(si,komo_);

    // ompl::geometric::PathGeometricPtr path = std::make_shared<ompl::geometric::PathGeometric>(si);

    // optimizer->optimize_path(path);

    KOMO komo;
    komo.setModel(C, true);
    komo.setTiming(1., 20 /* configs.N */, 1, 2);
    komo.add_qControlObjective({}, 2, 1.);
    komo.addObjective({1.}, FS_qItself, {}, OT_eq, {10}, goal_, 0);
    komo.add_collision(true);
    
    komo.initWithWaypoints(configs, configs.N, false);
    komo.run_prepare(0);
    komo.animateOptimization = 2;
    komo.optimize();
	komo.plotTrajectory();

    // arrA configs = komo.getPath_q();
    // std::ofstream myfile;
    // myfile.open (file.c_str());
    // myfile << configs;
    // myfile.close();

	rai::ConfigurationViewer V;
	V.setPath(C, komo.x, "result", true);
	while(V.playVideo(true, 1.));
}


int main(int argc, char ** argv)
{
  	rai::initCmdLine(argc,argv);   
    std::string filename = argv[1]; 
    optimize(filename);
	return 0;
}