//The purpose of this code is to check if I can access Rai from this folder.


#include <KOMO/komo.h>
#include <Kin/viewer.h>

void test()
{
	rai::Configuration C;
	C.addFile("../Models/2D_arm.g");
	KOMO komo;
	komo.verbose = 0;
  komo.setModel(C, true);
    
  komo.setTiming(1., 30, 5., 2);
	komo.add_qControlObjective({}, 1, 1.);
  komo.addObjective({}, FS_accumulatedCollisions, {}, OT_eq, {1.});
  komo.add_collision(true);

  komo.run_prepare(0);
  komo.optimize();
	komo.plotTrajectory();

  rai::ConfigurationViewer V;
	V.setPath(C, komo.x, "result", true);
	V.playVideo(true, 1.);
}

int main(int /*argc*/,char** /*argv*/){

  test();

  return 0;
}