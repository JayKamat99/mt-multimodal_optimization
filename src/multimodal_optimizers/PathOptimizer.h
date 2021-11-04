#ifndef OMPL_GEOMETRIC_PATH_OPTIMIZER_
#define OMPL_GEOMETRIC_PATH_OPTIMIZER_

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/goals/GoalSampleableRegion.h>

namespace ompl
{
    namespace geometric
    {
        OMPL_CLASS_FORWARD(PathOptimizer);

        class PathOptimizer
        {
        protected:
            double pathCost = 10000.;

            base::SpaceInformationPtr si_;

        public:
            PathOptimizer() = default;
            virtual ~PathOptimizer() = default;

            virtual bool optimize(PathGeometric &path)=0;

            void setPathCost(double pathCost){
                this->pathCost = pathCost;
            }

            double getPathCost() const{
                std::cout << "Path Cost:" << pathCost <<std::endl;
                return pathCost;
            }

            void setSI(base::SpaceInformationPtr si);

            virtual void displayPath(PathGeometric &path, const std::string &txt) const;

            bool isStepWise{false};
        };
    } // namespace geometric
} // namespace ompl


#endif