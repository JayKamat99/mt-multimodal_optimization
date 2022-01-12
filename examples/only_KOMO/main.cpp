#include <KOMO/komo.h>
#include <Kin/viewer.h>
#include <fstream>

void plan(const char* filename = "../examples/Models/2D_arm.g"){
    // filename = "../examples/Models/2_Two_Pandas.g";
    filename = "../examples/Models/4_kuka_box.g";
    // filename = "../examples/Models/5_disc_obstacle.g";
    // filename = "../examples/Models/3_TwoMobileManipulators.g";
    // Load the configuration
    rai::Configuration C;
    C.addFile(filename);
    cout <<"configuration space dim=" <<C.getJointStateDimension() <<endl;

    KOMO komo;
    komo.setModel(C, true);
    komo.setTiming(1., 10, 5., 2);
    komo.add_qControlObjective({}, 2, 1.);

    // komo.addObjective({1.}, FS_positionDiff, {"l_panda_hand_joint", "r_panda_link0"}, OT_sos, {1e1});
    // komo.addObjective({1.}, FS_positionDiff, {"r_panda_hand_joint", "l_panda_link0"}, OT_sos, {1e1});
    komo.addObjective({1.}, FS_positionDiff, {">tool0_joint", "target"}, OT_sos, {1e1});
    // komo.addObjective({1.}, FS_positionDiff, {"bot", "target"}, OT_sos, {1e1});
    // komo.addObjective({1.}, FS_positionDiff, {"l_panda_hand_joint", "r_target"}, OT_sos, {1e1});
    // komo.addObjective({1.}, FS_positionDiff, {"r_panda_hand_joint", "l_target"}, OT_sos, {1e1});
    komo.addObjective({.98,1.}, FS_qItself, {}, OT_sos, {1e1}, {}, 1);  //small velocity in that time frame
    komo.addObjective({}, FS_accumulatedCollisions, {}, OT_eq, {1.});
    komo.add_collision(true);

    // komo.animateOptimization = 1;
    // komo.initWithWaypoints({{1,1},{2,2}} , 2, false); //false-> it's linear interpolation and not sine
    
    // for (int i=0; i<5; ++i){
    //   komo.optimize();
    //   komo.plotTrajectory();
    //   komo.checkGradients();
    //   std::cout << i <<"\n";
    //   rai::ConfigurationViewer V;
    //   V.setPath(C, komo.x, "result", false);
    //   while(V.playVideo());
    // }
    komo.optimize();

    std::ofstream myfile;
    myfile.open ("FinalConfig.txt");
    myfile << komo.getPath_q();
    myfile.close();

    arr FinalConfig = komo.getConfiguration_q(1);
    komo.plotTrajectory();
    komo.checkGradients();
    rai::ConfigurationViewer V;
    V.setPath(C, komo.x, "result", false);
    while(V.playVideo());
    // std::cout << "\n Final Config: " << komo.getConfiguration_q(1) << std::endl;
}


int main(int argc, char ** argv)
{
  	rai::initCmdLine(argc,argv);
    plan();
	return 0;
}