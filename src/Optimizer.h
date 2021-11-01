#ifndef MT_MULTIMODAL_OPTIMIZER
#define MT_MULTIMODAL_OPTIMIZER

#include<iostream>
#include<fstream>
#include<vector>

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/LBTRRT.h>
#include <ompl/multilevel/planners/multimodal/LocalMinimaSpanners.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/tools/benchmark/Benchmark.h>

#include <ompl/base/samplers/ObstacleBasedValidStateSampler.h>
#include <ompl/base/samplers/GaussianValidStateSampler.h>

#include <ompl/geometric/PathOptimizerKOMO.h>

#include <ompl/config.h>

#include <KOMO/komo.h>
#include <Kin/viewer.h>


#include <iostream>
#include <functional>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/ProblemDefinition.h>

#define PI 3.1412

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace om = ompl::multilevel;

class Optimizer
{
public:

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
			// return 1;
		}
	};

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

	Optimizer();
	~Optimizer() = default;
	void plan();
	void benchmark();
	void setStart(std::vector<double> startPosition){this->startPosition = startPosition;}
	void setGoal(std::vector<double> goalPosition){this->goalPosition = goalPosition;}
	void setPlanner(Planners planner){this->planner = planner;}
	void setSolveTime(double solveTime){this->solveTime = solveTime;}
	void setSampler(Samplers sampler){this->sampler = sampler;}
	void setConfigurationFilename(std::string filename);
	void visualize_random();

private:
	std::string filename;
	unsigned int CSpace_Dimension;
	double solveTime = 10.0;
	std::vector<double> startPosition;
	std::vector<double> goalPosition;
	Planners planner = pathOptimizerKOMO;
	Samplers sampler = RandomSampler;

	og::SimpleSetup createSimpleSetup(ValidityCheckWithKOMO checker);
	void VisualizePath(arrA configs); //make this public later
};


#endif