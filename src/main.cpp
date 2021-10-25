#include <iostream>
#include "Optimizer.h"

#define PI 3.1412

/** \brief This is the main file that runs the whole code.
 * All commands / user variables are defined in this code.
 **************************************************/

int main(int /* argc */, char** /* argv[] */)
{
	/**
	 * @brief What do I do?
	 * define optimizer
	 * set configuration
	 * set statespace
	 * 	set start and goal states
	 * 	define planner // define enum of planners
	 * 	define solution time
	 * solve
	 * visualize solution 
	 */

	Optimizer optimizer;
	optimizer.setConfigurationFilename("../Models/Configuration.txt");

	//set Start and Goal configurations
	// std::vector<double> Start = {0,0};
	// optimizer.setStart(Start); // would I ever need start? Does KOMO change start state?
	optimizer.setGoal({1.5,0.0});

	//set planner
	optimizer.setPlanner(Optimizer::Planners::pathOptimizerKOMO);

	//set maximum solution time
	optimizer.setSolveTime(10.0);

	//plan
	optimizer.plan();

	// //visualize solution
	// optimizer.visualizeSolution();
}