// Basic Requirements
#include <ompl/config.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include <ompl/base/goals/GoalLazySamples.h>
#include <ompl/geometric/GeneticSearch.h>

#include <ompl/geometric/planners/informedtrees/BITstar.h>

#include <KOMO/komo.h>
#include <string>
#include <map>
#include <Core/graph.h>
#include <Kin/switch.h>
#include <Kin/viewer.h>

#define PI 3.1412

unsigned int C_Dimension;

namespace ob = ompl::base;
namespace og = ompl::geometric;

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

/**
 * @brief The graph structure maintains 
 * 
 */
// typedef struct graph
// {
// private:

// public:
//   graph();
//   ~graph();
// }graph;

// typedef struct ConfigurationSpaceTree
// {
// private:
//   ob::SpaceInformationPtr si_;
//   ConfigurationSpaceTree* parent{nullptr};
//   std::vector<double> startS;
//   std::vector<std::vector<double>> goalRegion;
//   graph* graph;   
// public:
//   ConfigurationSpaceTree() = default;
//   ConfigurationSpaceTree(const ob::SpaceInformationPtr &si) : si_(std::move(si))
//   {}
//   ~ConfigurationSpaceTree();
//   void setStartState(arr start);
//   void addGoalConfig(arr goal);
// }ConfigurationSpaceTree;

//=============================================================

void VisualizePath(arrA configs, std::string filename = ""){
	static int Trajectory = 1;
	
	// Create an instance of KOMO
  rai::Configuration C;
  C.addFile(filename.c_str());
  KOMO komo;
  komo.verbose = 0;
  komo.setModel(C, true);
  
  komo.setTiming(3., configs.N/3, 5., 2);
	komo.add_qControlObjective({}, 1, 1.);
  komo.addSwitch_stable(1., 2., "table", "gripper", "box");
  komo.addSwitch_stable(2., -1., "gripper", "table", "box", false);

  //use configs to initialize with waypoints
	komo.initWithWaypoints(configs, configs.N, false);
  komo.run_prepare(0);
	komo.plotTrajectory();
	// filename.erase(0,19);
	// filename.erase(filename.length()-2);
	// std::string SaveToPath = std::string("../visualize/videos/") + filename + "/";

	rai::ConfigurationViewer V;
	V.setPath(C, komo.x, "result", true);
	V.playVideo(true, 1./* , SaveToPath.c_str() */);
}

//===========================================================================

void testPickAndPlace(bool keyframesOnly){
  rai::Configuration C("../examples/Models/model2.g");

//  rai::ConfigurationViewer V;
//  V.setConfiguration(C, "initial model", false);

  KOMO komo;

  komo.setModel(C, false);
  if(!keyframesOnly){
    komo.setTiming(2.5, 30, 5., 2);
    komo.add_qControlObjective({}, 2);
  }else{
    komo.setTiming(3., 1, 5., 1);
    komo.add_qControlObjective({}, 1, 1e-1);
  }
  komo.addSquaredQuaternionNorms();

  //grasp
  komo.addSwitch_stable(1., 2., "table", "gripper", "box");
  komo.addObjective({1.}, FS_positionDiff, {"gripper", "box"}, OT_eq, {1e2});
  komo.addObjective({1.}, FS_scalarProductXX, {"gripper", "box"}, OT_eq, {1e2}, {0.});
  komo.addObjective({1.}, FS_vectorZ, {"gripper"}, OT_eq, {1e2}, {0., 0., 1.});

  if(!keyframesOnly){
    //slow - down - up
    komo.addObjective({1.}, FS_qItself, {}, OT_eq, {}, {}, 1);
    komo.addObjective({.9,1.1}, FS_position, {"gripper"}, OT_eq, {}, {0.,0.,.1}, 2);
  }


  //place
  komo.addSwitch_stable(2., -1., "gripper", "table", "box");
  komo.addObjective({2.}, FS_positionDiff, {"box", "table"}, OT_eq, {1e2}, {0,0,.08}); //arr({1,3},{0,0,1e2})
  komo.addObjective({2.}, FS_vectorZ, {"gripper"}, OT_eq, {1e2}, {0., 0., 1.});

  if(!keyframesOnly){
    //slow - down - up
    komo.addObjective({2.}, FS_qItself, {}, OT_eq, {}, {}, 1);
    komo.addObjective({1.9,2.1}, FS_position, {"gripper"}, OT_eq, {}, {0.,0.,.1}, 2);
  }
  
  // komo.animateOptimization = 2;
  komo.verbose = 4;
  komo.optimize();
  for (int i=0; i<75; i++){
    arr goal1 = komo.getConfiguration_q(i);
    std::cout << "state" << i << " = " << goal1 << std::endl;
  }
  // arr goal2 = komo.getConfiguration_q(60);
  // std::cout << "Goal 2 = " << goal2 << std::endl;
//  komo.checkGradients();

  komo.view(true, "optimized motion");
  for(uint i=0;i<2;i++) komo.view_play(true);
 // V.setPath(komo.getPath_frames(), "optimized motion", true);
 // for(uint i=0;i<2;i++) V.playVideo(true);
}

//===========================================================================

arrA solve(ConfigurationSpaceTree *leaf_)
{
  /** This function uses the problem definition (start and goal), 
   * the space information (space, validity checker), 
   * and the pre-computed graph to find the best path from start to goal using A*
   * The graph may be modified in this algorithm
   */
}

void sequentialOmpl()
{
  bool goalReached = false;

  // Setup KOMO

  // Init
  ConfigurationSpaceTree leaf_;
  leaf_.setStartState({0,0,0}); // my current location
  leaf_.addGoalConfig({1,1,1}); // use KOMO for this (region)
  // setcollision checking

  while (!goalReached)
  {
    // arrA solutionPath = solve(ConfigurationSpaceTree &leaf_);

    // If success, move to goal region in komo and perform mode_switch // fix rel transformation for now
    // How do I do this? Is there a nice way of doing it?

      // If final goal reached (Goal Condition sattisfied): Need a function to check this.
        // return
      // else
        // create a new Configuration Space and set it as child
        // Set Start
        // Define Goal Region
        // Move to the child Configuration Space

    // Else, we assume there is no path to goal and we need to explore other path (we need a method to come back here)
    // Move to the parent config space and coninue search with this goal state removed.
  }

}

//===========================================================================

void sequentialOmplHardCoded()
{
  std::string filename = "../examples/Models/model2.g";
  rai::Configuration C(filename.c_str());

	KOMO komo;
	komo.setModel(C, true);
	komo.setTiming(1, 1, 1, 1);
	komo.addObjective({}, FS_accumulatedCollisions, {}, OT_eq, { 1 });
	komo.run_prepare(0);

	C_Dimension = C.getJointStateDimension();

	//Construct the state space we are planning in
	auto space(std::make_shared<ob::RealVectorStateSpace>(C_Dimension));

	ob::RealVectorBounds bounds(C_Dimension);
	bounds.setLow(-PI);
	bounds.setHigh(PI);
	space->setBounds(bounds);

  // Set Goals 1 and 2
  arr goal1_ = {0.0505656, 0.673177, 0.10986, -1.96045, -0.139219, 2.62652, -1.05106, 0.0481076};
	arr goal2_ = {-0.00371728, 0.210831, -0.0231252, -2.97777, -0.0949239, 3.19421, -0.420676, 0.0491746};

	//create simple setup
	og::SimpleSetup ss1(space);

  // set state validity checking for this space
	auto nlp = std::make_shared<KOMO::Conv_KOMO_SparseNonfactored>(komo, false);
	ValidityCheckWithKOMO checker(*nlp);

	ss1.setStateValidityChecker([&checker](const ob::State *state) {
		return checker.check(state);
	});

	// create start and goal states. These states might change from example to example
  ob::ScopedState<> start(space);
	for (unsigned int i=0; i<C.getJointStateDimension(); i++){
	start[i] = komo.getConfiguration_q(0).elem(i);
	}

	ss1.setStartState(start);

  ob::ScopedState<> goal1(space);
  for (unsigned int i=0; i<goal1_.N; i++){
	goal1[i] = goal1_(i);
	}
  ss1.setGoalState(goal1);

  ss1.setPlanner(std::make_shared<og::BITstar>(ss1.getSpaceInformation()));

  ss1.setup();

  // attempt to solve the problem
  ob::PlannerStatus solved = ss1.solve(10.0);

  if (solved == ob::PlannerStatus::StatusType::APPROXIMATE_SOLUTION)
    std::cout << "Found solution: APPROXIMATE_SOLUTION" << std::endl;
  else if (solved == ob::PlannerStatus::StatusType::EXACT_SOLUTION)
    std::cout << "Found solution: EXACT_SOLUTION" << std::endl;
  else if (solved == ob::PlannerStatus::StatusType::TIMEOUT)
    std::cout << "Found solution: TIMEOUT" << std::endl;
  else{
    std::cout << "No solution found: Invalid " << std::endl;
    return;
  }

  auto path = ss1.getSolutionPath();
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
    std::cout << config;
  }

  //part 2
  std::cout<< "configuration: " << C.getJointState();
  std::cout<< "last: " << configs.last();

  komo.setConfiguration_X(0,configs.last());
  std::cout<< "configuration: " << komo.getConfiguration_q(0);
  komo.addSwitch_stable(0., 1., "table", "gripper", "box");
  komo.run_prepare(0);

  // std::cout<< "configuration: " << komo.getConfiguration_q(0);

  // //create simple setup
	// og::SimpleSetup ss2(space);

  // // set state validity checking for this space
	// auto nlp2 = std::make_shared<KOMO::Conv_KOMO_SparseNonfactored>(komo, false);
	// ValidityCheckWithKOMO checker2(*nlp2);

	// ss2.setStateValidityChecker([&checker2](const ob::State *state) {
	// 	return checker2.check(state);
	// });

	// // create start and goal states. These states might change from example to example
  // ob::ScopedState<> start2(space);
	// for (unsigned int i=0; i<C.getJointStateDimension(); i++){
	// start2[i] = komo.getConfiguration_q(0).elem(i);
	// }

	// ss2.setStartState(start);

  // ob::ScopedState<> goal2(space);
  // for (unsigned int i=0; i<goal2_.N; i++){
	// goal2[i] = goal2_(i);
	// }
  // ss2.setGoalState(goal2);

  // ss2.setPlanner(std::make_shared<og::BITstar>(ss2.getSpaceInformation()));

  // ss2.setup();

  // // attempt to solve the problem
  // ob::PlannerStatus solved2 = ss2.solve(10.0);

  // if (solved2 == ob::PlannerStatus::StatusType::APPROXIMATE_SOLUTION)
  //   std::cout << "Found solution: APPROXIMATE_SOLUTION" << std::endl;
  // else if (solved2 == ob::PlannerStatus::StatusType::EXACT_SOLUTION)
  //   std::cout << "Found solution: EXACT_SOLUTION" << std::endl;
  // else if (solved2 == ob::PlannerStatus::StatusType::TIMEOUT)
  //   std::cout << "Found solution: TIMEOUT" << std::endl;
  // else{
  //   std::cout << "No solution found: Invalid " << std::endl;
  //   return;
  // }

  // Visualize in KOMO
  VisualizePath(configs, filename);
}

int main(int argc,char** argv){
  rai::initCmdLine(argc,argv);

  // inputs for the sequential planner will be the example id, goal 1, goal 2

  // sequentialOmpl();
  // sequentialOmplHardCoded();


  // -------------------------------------
  testPickAndPlace(false);
  // testPickAndPlace(true);

  return 0;
}

