// The goal of this program is to read the trajectory and optimize the path.

#include <KOMO/komo.h>
#include <Kin/viewer.h>
#include <fstream>

void optimize(std::string filename){
    // Load the configuration
    rai::Configuration C;
    C.addFile(filename.c_str());
    cout <<"configuration space dim=" <<C.getJointStateDimension() <<endl;
    std::string file;
    arr goal_;

    if (filename == "../examples/Models/2_Two_Pandas.g"){
        file = "../debug/initPathConfigs.txt";
        // file = "../debug/finalPathConfigs.txt";
        // goal_ = {-0.348907, 1.34251, 0.464246, -0.888288, -0.462429, 2.66979, -0.0416295, 0.0134267, -0.350246, 1.3497, 0.486382, -0.897415, -0.482546, 2.6686, -0.0623223, 0.0197671, 0, 0};
    }
    else if (filename == "../examples/Models/7_disc_rooms.g"){
        // file = "../debug/7_configs.txt";
    }
    else if (filename == "../examples/Models/2D_bot.g"){
        file = "../debug/7_configs.txt";
        goal_ = {0,1,0};
    }

    std::ifstream myfile;
    arrA configs;
    myfile.open (file.c_str());
    myfile >> configs;
    myfile.close();

    std::cout << "Configs: " << configs << std::endl;
    goal_ = configs(configs.N-1);
    std::cout << "Goal: " << goal_ << std::endl;

    KOMO komo;
    komo.setModel(C, true);
    komo.setTiming(1., 20 /* configs.N */, 5., 2);
    komo.add_qControlObjective({}, 2, 1.);
    komo.addObjective({1.}, FS_qItself, {}, OT_eq, {10}, goal_, 0);
    komo.add_collision(true);

    // komo.initWithWaypoints(configs, configs.N, false);
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
	V.playVideo(true, 1.);
}


int main(int argc, char ** argv)
{
  	rai::initCmdLine(argc,argv);   
    std::string filename = argv[1]; 
    optimize(filename);
	return 0;
}