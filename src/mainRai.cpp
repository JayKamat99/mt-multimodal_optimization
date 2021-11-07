//The purpose of this code is to check if I can access Rai from this folder.


#include <KOMO/komo.h>
#include <Kin/viewer.h>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

#define PI 3.1412

namespace ob = ompl::base;
namespace og = ompl::geometric;

  struct ValidityCheckWithKOMO {
	KOMO::Conv_KOMO_SparseNonfactored &nlp;
	ValidityCheckWithKOMO(KOMO::Conv_KOMO_SparseNonfactored &nlp) : nlp(nlp){}
	bool check(const ob::State *state)
	{
		const auto *State = state->as<ob::RealVectorStateSpace::StateType>();

		arr x_query;
		for (unsigned int i = 0; i < 2; i++){
			x_query.append((*State)[i]);
		}

		arr phi;
		nlp.evaluate(phi, NoArr, x_query);
		double tol = 1e-2;

		return std::abs(phi(0)) < tol;
	}
};

void VisualizePath(arrA configs){
	// setup KOMO
  rai::Configuration C;
  C.addFile("../Models/2D_arm.g");
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

	rai::ConfigurationViewer V;
	V.setPath(C, komo.x, "result", true);
	V.playVideo(true, 1.);
}

void test()
{
	rai::Configuration C;
	C.addFile("../Models/2D_arm.g");
	KOMO komo;
	komo.verbose = 0;
  komo.setModel(C, true);
  komo.setTiming(1, 1, 1, 1);
	komo.addObjective({}, FS_accumulatedCollisions, {}, OT_eq, { 1 });
	komo.run_prepare(0);

  int C_Dimension = C.getJointStateDimension();

  auto space(std::make_shared<ob::RealVectorStateSpace>(C_Dimension));

	ob::RealVectorBounds bounds(C_Dimension);
	bounds.setLow(-PI);
	bounds.setHigh(PI);
	space->setBounds(bounds);

  og::SimpleSetup ss(space);

	auto nlp = std::make_shared<KOMO::Conv_KOMO_SparseNonfactored>(komo, false);
	ValidityCheckWithKOMO checker(*nlp);

	ss.setStateValidityChecker([&checker](const ob::State *state) {
		return checker.check(state);
	});

  ob::ScopedState<> start(space); //start is automatically assumed

  ob::ScopedState<> goal(space);
	goal = {2,0};

  ss.setStartAndGoalStates(start, goal);

  auto si = ss.getSpaceInformation();
  auto planner(std::make_shared<og::RRTstar>(si));
	ss.setPlanner(planner);

  ss.setup();

  ss.solve(10.0);
    
  auto path = ss.getSolutionPath();
  arrA configs;
  for (auto state : path.getStates())
  {
    arr config;
    std::vector<double> reals;
    space->copyToReals(reals, state);
    for (double r : reals){
      config.append(r);
    }
    configs.append(config);
  }
  //Visualize in KOMO
  VisualizePath(configs);
}

int main(int /*argc*/,char** /*argv*/){
  
  test();

  return 0;
}