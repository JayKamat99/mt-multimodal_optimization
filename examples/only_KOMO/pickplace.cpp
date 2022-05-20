

#include <KOMO/komo.h>
#include <KOMO/komo-ext.h>
#include <Kin/viewer.h>
#include <fstream>

#include <iostream>

void pick_place_multipleKOMO(std::string filename = "../examples/Models/s1_2d_manip.g")
{



  rai::Configuration C;
  C.addFile(filename.c_str());


  // visualize environment
  C.watch(true);

  std::cout << "Joint state" << C.getJointState() << std::endl;


  // solve for keyframe picking, without adding a mode switch
  // Note, there are not collisions...
  KOMO komo;
  komo.setModel(C);
  komo.setTiming(1,1,1,1);
  komo.addObjective({1,1},FS_distance,{"object","endeff"}, OT_eq);
  komo.addObjective({1,1},FS_distance,{"object","endeff"}, OT_ineq);

  komo.optimize();
  komo.view(true);

  // set the joint state for the keyframe, and attach the frame
  C.setJointState(komo.x);
  C.getFrameState();

   C.attach(C.getFrame("endeff"), C.getFrame("object"));

   // Now, if we move robot, the object moves
  //  C.setJointState( arr({0, 0, 0, 0, 1, 0, 0, 0, 0, 0}) );
   C.setJointState( arr({0, 0, 0}) );
   C.watch(true);


  // solve for keyframe picking, without adding a mode switch
  // Note, there are not collisions...
  KOMO komo2;
  komo2.setModel(C);
  komo2.setTiming(1,1,1,1);
  komo2.addObjective({1,1},FS_poseDiff,{"object","target"}, OT_eq);

  komo2.optimize();
  komo2.view(true);
}

void testPickAndPlace(std::string filename, bool keyframesOnly)
{
	rai::Configuration C(filename.c_str());

  // rai::ConfigurationViewer V;
  // V.setConfiguration(C, "initial model", false);

	KOMO_ext komo;

	komo.setModel(C, false);
	if (!keyframesOnly)
	{
		komo.setTiming(2.5, 30, 5., 2);
		komo.add_qControlObjective({}, 2);
	}
	else
	{
		komo.setTiming(3., 1, 5., 1);
		komo.add_qControlObjective({}, 1, 1e-1);
	}
	komo.addSquaredQuaternionNorms();

	// grasp
	komo.addSwitch_stable(1., 2., "table", "gripper", "box");
	komo.addObjective({1.}, FS_positionDiff, {"gripper", "box"}, OT_eq, {1e2});
	komo.addObjective({1.}, FS_scalarProductXX, {"gripper", "box"}, OT_eq, {1e2}, {0.});
	komo.addObjective({1.}, FS_vectorZ, {"gripper"}, OT_eq, {1e2}, {0., 0., 1.});

	// if (!keyframesOnly)
	// {
	// 	// slow - down - up
	// 	komo.addObjective({1.}, FS_qItself, {}, OT_eq, {}, {}, 1);
	// 	komo.addObjective({.9, 1.1}, FS_position, {"gripper"}, OT_eq, {}, {0., 0., .1}, 2);
	// }

	// place
	komo.addSwitch_stable(2., -1., "gripper", "table2", "box", false);
	// komo.addObjective({2.}, FS_aboveBox, {"box", "table"}, OT_ineq, {1e2}, {0,0,0,0});
  // komo.setPlace(2., "gripper", "box", "table");
  // komo.addObjective({2.}, FS_aboveBox, {"box" , "table"}, OT_ineq, {1e1});
  komo.addSwitch_stableOn(2., 2., "table2", "box");

  // komo.addObjective({2.}, FS_aboveBox, {"box" , "table"}, OT_ineq, {1e2});
  komo.add_collision(true, 0.1);
  // komo.addObjective({2.}, FS_positionDiff, {"box", "table"}, OT_eq, {1e2}, {0,0,.08}); //arr({1,3},{0,0,1e2})
	komo.addObjective({2.}, FS_vectorZ, {"gripper"}, OT_eq, {1e2}, {0., 0., 1.});

	// if (!keyframesOnly)
	// {
	// 	// slow - down - up
	// 	komo.addObjective({2.}, FS_qItself, {}, OT_eq, {}, {}, 1);
	// 	komo.addObjective({1.9, 2.1}, FS_position, {"gripper"}, OT_eq, {}, {0., 0., .1}, 2);
	// }

	komo.verbose = 4;
	komo.optimize();
	//  komo.checkGradients();

	komo.view(true, "optimized motion");
	for (uint i = 0; i < 2; i++)
		komo.view_play(true);
}

int main(int argc,char** argv) {
  // pick_place_multipleKOMO(filename);

  std::string filename;
  if (argc > 1)
  {
    filename = argv[1];
    std::cout << filename << std::endl;
  }
  testPickAndPlace(filename,false);

  return 0;
}
