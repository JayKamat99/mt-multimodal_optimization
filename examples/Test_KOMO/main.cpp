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

    if (filename == "../examples/Models/2_Two_Pandas.g"){
        file = "../debug/initPathConfigs.txt";
    }
    else if (filename == "../examples/Models/7_disc_rooms.g"){
        file = "../debug/7_configs.txt";
    }

    std::ifstream myfile;
    arrA configs;
    myfile.open (file.c_str());
    myfile >> configs;
    myfile.close();

    std::cout << configs << std::endl;
    arr goal_ = configs(configs.N-1);

    KOMO komo;
    komo.setModel(C, true);
    komo.setTiming(1., configs.N, 5., 2);
    komo.add_qControlObjective({}, 2, 1.);
    komo.addObjective({1.}, FS_qItself, {}, OT_eq, {10}, goal_, 0);
    komo.add_collision(true);

    komo.initWithWaypoints(configs, configs.N, false);
    komo.run_prepare(0);
    komo.animateOptimization = 1;
    komo.optimize();
	komo.plotTrajectory();

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