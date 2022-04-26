

#include <KOMO/komo.h>
#include <Kin/viewer.h>
#include <fstream>

#include <iostream>


int main() {

  std::cout << "hello world" << std::endl;

  auto filename = "../examples/only_KOMO/model.g";


  rai::Configuration C;
  C.addFile(filename);


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
   C.setJointState( arr({0, 0, 0, 0, 1, 0, 0, 0, 0, 0}) );
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
