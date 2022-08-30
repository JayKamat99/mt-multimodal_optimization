#ifndef OMPL_GEOMETRIC_naiveSequentialPlanner_
#define OMPL_GEOMETRIC_naiveSequentialPlanner_

/**
 * @file naiveSequentialPlanner.h
 * @author Jay Kamat
 * @brief Short for sequential keyframe tree planner
 * @version 0.1
 * @date 2022-07-11
 * 
 * @copyright Copyright (c) 2022
 * 
**/

#include <cstdlib>
#include <time.h>

#include <ompl/geometric/planners/PlannerIncludes.h>
#include <limits>
#include <stack>
#include <set>
#include <KOMO/komo.h>
#include <Kin/kin.h>
#include <Kin/viewer.h>

// ompl includes
#include <ompl/config.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/goals/GoalStates.h>
#include <ompl/base/Planner.h>

#include <ompl/base/ProblemDefinition_ext.h>

// Planners
#include <ompl/geometric/planners/informedtrees/BITstar.h>

// #define VISUALIZE

namespace ob = ompl::base;
namespace og = ompl::geometric;

#define PI 3.14159
#define tol 1e-2
#define nullArrA (arrA){{}}

namespace ompl
{
    namespace geometric
    {

        OMPL_CLASS_FORWARD(naiveSequentialPlanner);

        class naiveSequentialPlanner : public base::Planner
        {
        public: 
            naiveSequentialPlanner(const base::SpaceInformationPtr &si, std::string name = "naiveSequentialPlanner");
            virtual ~naiveSequentialPlanner() override;

			base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

            enum SUBPLANNER{
                BITstar
            };

            void set_subPlanner(SUBPLANNER subPlanner) {this->subPlanner = subPlanner;}
            void set_inputs(std::vector<std::string> inputs) {this->inputs = inputs;}
            void set_maxConstraintViolationKOMO(double maxConstraintViolationKOMO) {this->maxConstraintViolationKOMO = maxConstraintViolationKOMO;}

        protected:
            // PLanner Variables
            std::vector<std::string> inputs;
            SUBPLANNER subPlanner = BITstar;
            double maxConstraintViolationKOMO;
            int C_Dimension;
            std::string filename;
			int totalPhases;

            void freeMemory();

            void clear() override;

            void setup() override;

		    arrA sampleKeyframeSequence(ompl::base::PlannerTerminationCondition ptc);

            ompl::base::PlannerPtr initPlanner(int currentPhase, arrA sequence);

            /* visualizePath can visualize only one phase at a time */
            void visualizePath(arrA &configs, int currentPhase = 0, arrA sequence = {});

            arrA copyPathToArrA(std::shared_ptr<ompl::geometric::PathGeometric> path, ompl::base::PlannerPtr planner);

            /* Progress Properties */
            /** \brief Retrieve the best exact-solution cost found as a planner-progress property. */
			ompl::base::Cost bestCost;
            std::string bestCostProgressProperty() const
            {
                return ompl::toString(bestCost.value());
            }

        };  
    } // namespace geometric
} //namespace ompl

#endif