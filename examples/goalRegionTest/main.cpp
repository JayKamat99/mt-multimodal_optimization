#include <iostream>

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
#include <Kin/viewer.h>


namespace ob = ompl::base;

class MyGoalRegion : public ob::GoalRegion
{
public:

    MyGoalRegion(const ob::SpaceInformationPtr &si) : ob::GoalRegion(si)
    {
        setThreshold(1e-4);
    }

    // If I can somehow calculate the distance between endeff and target I am done
    double distanceGoal(const ob::State *state) const override
    {
        // goal region is given by states where x = y
        double R1 = state->as<ob::RealVectorStateSpace::StateType>()->operator[](0);
        double R2 = state->as<ob::RealVectorStateSpace::StateType>()->operator[](1);
        double d = (R1 - R2)*(R1 - R2);
        return d;
    }

};

bool regionSampling(const ob::SpaceInformationPtr &si, const ob::ProblemDefinitionPtr &pd, const ob::GoalLazySamples *gls, ob::State *result, std::string filename)
{
    bool cont = false;

    // Use Quim's method to get final config
    // Check for collision, if collision-free cont = true

    rai::Configuration C;
    C.addFile(filename.c_str());

    KOMO komo;
    komo.setModel(C, true);
    komo.setTiming(1., 1, 1., 1);
	komo.addObjective(
      {}, FS_qItself, {},
      OT_sos); // this regularizes with respect to q = zeros, i.e. term q.T q

    if (filename == "../examples/Models/1_kuka_shelf.g"){
        komo.addObjective({}, FS_positionDiff, {">tool0_joint", "target"}, OT_eq, {10});
	}

    // komo.add_collision(true);
	komo.addObjective(
      {}, FS_accumulatedCollisions, {}, OT_eq,
      {1.}); // add collisions between all bodies with keyword contact
	
	komo.optimize();

    // convert arr to state
    arr config = komo.getConfiguration_q(0);
    std::cout << config << std::endl;
    std::vector<double> reals;
    for (double r : config){
        reals.push_back(r);
    }
    const ob::StateSpace *space(si->getStateSpace().get());
    space->copyFromReals(result, reals);
    cont = true;

    if (cont)
    {
        std::cout << "Found goal state: " << std::endl;
        si->printState(result);
    }

    // we continue sampling while we are able to find solutions, we have found not more than 2 previous solutions and we have not yet solved the problem
    return cont && gls->maxSampleCount() < 3 && !pd->hasSolution();
}

void plan()
{
    // construct the state space we are planning in
    auto space(std::make_shared<ob::RealVectorStateSpace>(2));

    std::string filename = "../examples/Models/1_kuka_shelf.g";

    // set the bounds
    ob::RealVectorBounds bounds(2);
    bounds.setLow(-1);
    bounds.setHigh(1);

    space->setBounds(bounds);

    // define a simple setup class
    og::SimpleSetup ss(space);

    // create a random start state
    ob::ScopedState<ob::RealVectorStateSpace> start(space);
    start[0] = 0; start[1] = 0;
    ss.addStartState(start);

    // define our goal region
    MyGoalRegion region(ss.getSpaceInformation());

    // bind a sampling function that fills its argument with a sampled state
    // and returns true while it can produce new samples we don't need to
    // check if new samples are different from ones previously computed as
    // this is pefromed automatically by GoalLazySamples
    ob::GoalSamplingFn samplingFunction = [&ss, &region, filename](const ob::GoalLazySamples *gls, ob::State *result)
        {
            return regionSampling(ss.getSpaceInformation(), ss.getProblemDefinition(),
                gls, result, filename);
        };

    // create an instance of GoalLazySamples:
    auto goal(std::make_shared<ob::GoalLazySamples>(ss.getSpaceInformation(), samplingFunction));

    // we set a goal that is sampleable, but it in fact corresponds to a region that is not sampleable by default
    ss.setGoal(goal);

    //Set the planner
    ss.setPlanner(std::make_shared<ompl::geometric::BITstar>(ss.getSpaceInformation()));

    // attempt to solve the problem
    ob::PlannerStatus solved = ss.solve(3.0);

    if (solved)
    {
        std::cout << "Found solution:" << std::endl;
        // print the path to screen
        ss.simplifySolution();
        ss.getSolutionPath().print(std::cout);
    }
    else
        std::cout << "No solution found" << std::endl;

    // the region variable will now go out of scope. To make sure it is not used in the sampling function any more
    // (i.e., the sampling thread was able to terminate), we make sure sampling has terminated
    goal->as<ob::GoalLazySamples>()->stopSampling();
}

int main(int /*argc*/, char ** /*argv*/)
{
    plan(); // This function uses BIT* to plan from start to a goal region

    //Steps
    // 1: Plan in a  2d environment where goal is to reach any point in the square.
    // 2: Use an example from KOMO with a user defined hard coded goal region.
    // 3: KOMO example with Goal region also described by KOMO.
    return 0;
}