#ifndef MT_MULTIMODAL_OPTIMIZER
#define MT_MULTIMODAL_OPTIMIZER

#include<iostream>
#include<fstream>
#include<vector>

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/prm/LazyPRMstar.h>
#include <ompl/geometric/planners/rrt/LBTRRT.h>
#include <ompl/geometric/planners/rrt/LazyLBTRRT.h>
#include <ompl/geometric/planners/rrt/TRRT.h>
#include <ompl/geometric/planners/rrt/BiTRRT.h>
#include <ompl/multilevel/planners/multimodal/LocalMinimaSpanners.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/tools/benchmark/Benchmark.h>

#include <ompl/geometric/PathOptimizerKOMO.h>
#include <ompl/geometric/PathSimplifier.h>

#include <KOMO/komo.h>
#include <Kin/viewer.h>

#define PI 3.1412

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace om = ompl::multilevel;

std::string filename;
unsigned int CSpace_Dimension = 0;

class Optimizer
{
public:
	enum Planners {
        pathOptimizerKOMO,
        RRTStar,
        LBTRRT,
		KOMO_Optimizer
    };

	enum Samplers {
		RandomSampler,
		ObstacleBasedValidSampler,
		GaussianValidSampler
	};

	Optimizer() = default;
	~Optimizer() = default;
	void plan();
	void benchmark();
	void setStart(ob::ScopedState<> startPosition){this->startPosition = startPosition;}
	void setGoal(ob::ScopedState<> goalPosition){this->goalPosition = goalPosition;}
	void setPlanner(Planners planner){this->planner = planner;}
	void setSolveTime(double solveTime){this->solveTime = solveTime;}
	void setSampler(Samplers sampler){this->sampler = sampler;}
	void setConfigurationFilename(std::string filename);

private:
	double solveTime = 10.0;
	ob::ScopedState<> startPosition;
	ob::ScopedState<> goalPosition;
	Planners planner = pathOptimizerKOMO;
	Samplers sampler = RandomSampler;

	og::SimpleSetup createSimpleSetup(ValidityCheckWithKOMO checker);
	void VisualizePath(arrA configs); //make this public later
};


#endif