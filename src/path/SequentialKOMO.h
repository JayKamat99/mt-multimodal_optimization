#ifndef OMPL_GEOMETRIC_SequentialKOMO_
#define OMPL_GEOMETRIC_SequentialKOMO_

#include <ompl/geometric/planners/PlannerIncludes.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <KOMO/komo.h>
#include <Kin/viewer.h>
#include <Core/graph.h>
#include <limits>

namespace ompl
{
    namespace geometric
    {
        OMPL_CLASS_FORWARD(SequentialKOMO);

        class SequentialKOMO : public base::Planner
        {
        private:
            int iteration{0};
            int C_Dimension;
            int stepsPerPhase;

        protected:
            std::vector<std::string> inputs;
            double maxConstraintViolationKOMO = 1;
            arr startConfig;

            double calcCost(arrA configs);

			double bestCost = std::numeric_limits<double>::infinity();
			std::string bestCostProperty()
            {
                return std::to_string(bestCost);
            }

            void freeMemory();

            void clear() override;

            void setup() override;

            std::shared_ptr<ompl::geometric::PathGeometric> path;

        public:
            SequentialKOMO(const base::SpaceInformationPtr &si, std::string name = "SequentialKOMO");
            virtual ~SequentialKOMO() override;

            void set_inputs(std::vector<std::string> inputs) {this->inputs = inputs;}
            void set_maxConstraintViolationKOMO(double maxConstraintViolationKOMO) {this->maxConstraintViolationKOMO = maxConstraintViolationKOMO;}
            void set_stepsPerPhase(double stepsPerPhase) {this->stepsPerPhase = stepsPerPhase;}

            std::shared_ptr<KOMO> setupKOMO();

			base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;
			// std::string filename;
        };  
    } // namespace  geometric
} //namespace ompl

#endif