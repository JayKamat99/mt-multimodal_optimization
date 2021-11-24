#include <iostream>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/special/TorusStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/prm/SPARS.h>

#include <ompl/config.h>

#include <KOMO/komo.h>
#include <Kin/viewer.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

struct ValidityCheckWithKOMO {
  KOMO::Conv_KOMO_SparseNonfactored &nlp;
  ValidityCheckWithKOMO(KOMO::Conv_KOMO_SparseNonfactored &nlp) : nlp(nlp) {}
  bool check(const ob::State *state) {

    // rai: uses quaternion with real part first
    // ompl: uses quaternion with real part last
    // const auto *RealState = state->as<ob::RealVectorStateSpace::StateType>();
    const auto *TorusState = state->as<ob::TorusStateSpace::StateType>();
    // const auto *se3state = state->as<ob::SE3StateSpace::StateType>();

    /*arr x_query = arr
    {
      RealState->operator[](0),
      RealState->operator[](1)
    };*/

    arr x_query = arr
      {
        TorusState->getS1(),
        TorusState->getS2()
      };

    arr phi;
    nlp.evaluate(phi, NoArr, x_query);
    double tol = 1e-2;

    // std::cout << "Check at: " << x_query << " " << ( std::abs(phi(0)) < tol)
    // nlp->komo.watch(true);

    return std::abs(phi(0)) < tol;
  }
};

void planWithSimpleSetupKOMOinR2(){
  //construct the configuration space we will be working in
  auto space(std::make_shared<ob::RealVectorStateSpace>(2));

  // set the bounds
  ob::RealVectorBounds bounds(2);
  bounds.setLow(-3.14);
  bounds.setHigh(3.14);
  space->setBounds(bounds);

  // define a simple setup class
  og::SimpleSetup ss(space);

  // set state validity checking based on KOMO
  auto filename = "2D.g";
  rai::Configuration C;
  C.addFile(filename);
  KOMO komo;
  komo.setModel(C, true);
  komo.setTiming(1, 1, 1, 1);
  komo.addObjective({}, FS_accumulatedCollisions, {}, OT_eq, {1});
  komo.run_prepare(0);
  auto nlp = std::make_shared<KOMO::Conv_KOMO_SparseNonfactored>(komo, false);
  ValidityCheckWithKOMO checker(*nlp);

  ss.setStateValidityChecker(
      [&checker](const ob::State *state) { return checker.check(state); });

  // create a random start state
  ob::ScopedState<> start(space);

  start = std::vector<double>{1, 0};
  // start.random();

  ob::ScopedState<> goal(space);
  // goal.random();

  goal = std::vector<double>{-2.32849, 1.38262}; 

  std::cout << start << std::endl;
  std::cout << goal << std::endl;
  // throw -1;

  // set the start and goal states
  ss.setStartAndGoalStates(start, goal);

  //Define planner
  auto planner(std::make_shared<og::SPARS>(ss.getSpaceInformation()));
  ss.setPlanner(planner);

  // this call is optional, but we put it in to get more output information
  ss.setup();
  ss.print();

  // attempt to solve the problem within one second of planning time
  ob::PlannerStatus solved = ss.solve(10.0);

  if (solved) {
    std::cout << "Found solution:" << std::endl;
    // print the path to screen
    ss.simplifySolution();
    ss.getSolutionPath().print(std::cout);

    // lets visualize the waypoints of the path (the trajectory is linear
    // interpolation between them)
    auto states = ss.getSolutionPath().getStates();
    for (auto &s : states) {
      const auto *RealState = s->as<ob::RealVectorStateSpace::StateType>();

      arr x_query = arr
      {
        RealState->operator[](0),
        RealState->operator[](1)
      };

      C.setJointState(x_query);
      C.watch(true);
    }

  } else
    std::cout << "No solution found" << std::endl;
}

void planWithSimpleSetupKOMOinT2(){
  //construct the configuration space we will be working in
  auto space(std::make_shared<ob::TorusStateSpace>());

  // define a simple setup class
  og::SimpleSetup ss(space);

  // set state validity checking based on KOMO
  auto filename = "2D.g";
  rai::Configuration C;
  C.addFile(filename);
  KOMO komo;
  komo.setModel(C, true);
  komo.setTiming(1, 1, 1, 1);
  komo.addObjective({}, FS_accumulatedCollisions, {}, OT_eq, {1});
  komo.run_prepare(0);

  auto nlp = std::make_shared<KOMO::Conv_KOMO_SparseNonfactored>(komo, false);
  ValidityCheckWithKOMO checker(*nlp);

  ss.setStateValidityChecker(
      [&checker](const ob::State *state) { return checker.check(state); });

  // create a start state
  ob::ScopedState<> start(space);
  start = std::vector<double>{-1, 0};

  // create a goal state
  ob::ScopedState<> goal(space);
  goal = std::vector<double>{1, 0};

  // set the start and goal states
  ss.setStartAndGoalStates(start, goal);

  //Define planner
  auto planner(std::make_shared<og::SPARS>(ss.getSpaceInformation()));
  ss.setPlanner(planner);

  // attempt to solve the problem within one second of planning time
  ob::PlannerStatus solved = ss.solve(10.0);

  if (solved) {
    std::cout << "Found solution:" << std::endl;
    // print the path to screen
    ss.simplifySolution();   //This uses the shortcutting algo..
    ss.getSolutionPath().print(std::cout); //This prints out the states

    // lets visualize the waypoints of the path (the trajectory is linear
    // interpolation between them)

    auto states = ss.getSolutionPath().getStates();
    std::vector<arr> configs;
    for (auto &s : states) {
      const auto *TorusState = s->as<ob::TorusStateSpace::StateType>();

      arr x_query = arr
      {
        TorusState->getS1(),
        TorusState->getS2()
      };
      // std::cout << x_query << std::endl;
      configs.push_back(x_query);
      C.setJointState(x_query);
      C.watch(true);
    }
    for (auto it = configs.begin(); it != configs.end(); it++)
      cout << *it << " ";
  }
  else
    std::cout << "No solution found" << std::endl;

  // komo.animateOptimization = 1;
  // // komo.initWithWaypoints({{1,1},{2,2}} , 2, false);
  // komo.optimize();
  // komo.plotTrajectory();
  // komo.checkGradients();
  // rai::ConfigurationViewer V;
  // V.setPath(C, komo.x, "result", false);
  // while(V.playVideo());
}

void visualize_random() {
  auto filename = "2D.g";
  rai::Configuration C;
  C.addFile(filename);
  C.watch(true);

  // random configurations

  size_t N = 20;
  for (size_t i = 0; i < N; i++) {
    arr pose = rand(2);
    std::cout << pose << std::endl;
    C.setJointState(pose);
    C.watch(true);
  }
}

void compute_collisions_with_KOMO() {

  const char* filename = "2D.g";
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

  arr phi;
  arr x_query = arr{-1, 0};
  nlp->evaluate(phi, NoArr, x_query);
  std::cout << x_query << " " << phi << std::endl;

  x_query = arr{1, 0};
  nlp->evaluate(phi, NoArr, x_query);
  std::cout << x_query << " " << phi << std::endl;

  size_t N = 20;
  for (size_t i = 0; i < N; i++) {
    arr pose = rand(2);
    C.setJointState(pose);

    nlp->evaluate(phi, NoArr, pose);
    std::cout << pose << " " << phi << std::endl;
    komo.view(true);
  }
}

int main(int /*argc*/,char** /*argv*/){
  // rai::initCmdLine(argc,argv); // This logs the inputs..
  // std::cout << "OMPL version: " << OMPL_VERSION << std::endl;

  // visualize_random();
  // compute_collisions_with_KOMO();
  planWithSimpleSetupKOMOinT2();
  // planWithSimpleSetupKOMOinR2();

  return 0;
}