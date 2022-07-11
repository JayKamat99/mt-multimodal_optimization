#ifndef OMPL_GEOMETRIC_sktp_
#define OMPL_GEOMETRIC_sktp_

/**
 * @file sktp.h
 * @author Jay Kamat
 * @brief Short for sequential keyframe tree planner
 * @version 0.1
 * @date 2022-07-11
 * 
 * @copyright Copyright (c) 2022
 * 
**/

#include <ompl/geometric/planners/PlannerIncludes.h>
#include <limits>

namespace ompl
{
    namespace geometric
    {

        OMPL_CLASS_FORWARD(sktp);

        class sktp : public base::Planner
        {
        public: 
            enum SUBPLANNER{
                BITstar
            };

        private:
            std::vector<std::string> inputs;
            SUBPLANNER subPlanner;
            uint branchingFactor;
			double bestCost = std::numeric_limits<double>::infinity();

        protected:
            void freeMemory();
			std::string bestCostProperty()
            {
                return std::to_string(bestCost);
            }
            void clear() override;

            void setup() override;

        public:
            sktp(const base::SpaceInformationPtr &si);
            virtual ~sktp() override;

			base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

            void set_subPlanner(SUBPLANNER subPlanner) {this->subPlanner = subPlanner;}
            void set_branchingFactor(uint branchingFactor) {this->branchingFactor = branchingFactor;}
            void set_inputs(std::vector<std::string> inputs) {this->inputs = inputs;}
        };  
    } // namespace  geometric
} //namespace ompl

#endif