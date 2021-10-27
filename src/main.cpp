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
	optimizer.setConfigurationFilename("/home/jay/mt-multimodal_optimization/Models/Configuration.txt");

	//set the Goal configuration
	optimizer.setGoal({3.0,0.0});

	//set planner
	optimizer.setPlanner(Optimizer::Planners::pathOptimizerKOMO);

	//set maximum solution time
	optimizer.setSolveTime(5.0);

	//plan
	// optimizer.visualize_random();
	optimizer.plan();

	// //visualize solution
	// optimizer.visualizeSolution();
}