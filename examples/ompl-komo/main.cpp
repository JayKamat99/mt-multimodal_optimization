#include <iostream>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

#include <ompl/config.h>

#include "KOMO/komo.h"
#include "Kin/kin.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

void compute_collisions_with_KOMO() {

  auto filename = "../examples/Models/two_boxes.g";
  rai::Configuration C;
  C.addFile(filename);
  KOMO komo;
  komo.setModel(C, true);
  komo.setTiming(1, 1, 1, 1);
  komo.addObjective({}, FS_accumulatedCollisions, {}, OT_eq, {1});
  komo.run_prepare(0);

  auto nlp = std::make_shared<KOMO::Conv_KOMO_SparseNonfactored>(komo, false);

  ObjectiveTypeA ot;
  nlp->getFeatureTypes(ot);
  std::cout << ot << std::endl;

  int C_Dim = C.getJointStateDimension();
  std::cout << C_Dim << std::endl;

  arr phi;
  // arr x_query = arr{.35, 0, 0};
  // nlp->evaluate(phi, NoArr, x_query);
  // std::cout << phi << std::endl;
  // komo.view(true);

  // x_query = arr{.15, 0, 0};
  // nlp->evaluate(phi, NoArr, x_query);
  // std::cout << phi << std::endl;
  // komo.view(true);

  size_t N = 20;
  for (size_t i = 0; i < N; i++) {
    arr pose(3);
    arr position = {(double)-0.3*i/15,.01,0.2};
    pose({0, 2}) = position;

    nlp->evaluate(phi, NoArr, pose);
    std::cout << pose << " " << phi << std::endl;
    komo.view(true);
  }
}

int main(int /*argc*/, char ** /*argv*/) {
  std::cout << "OMPL version: " << OMPL_VERSION << std::endl;

  compute_collisions_with_KOMO();

  return 0;
}
