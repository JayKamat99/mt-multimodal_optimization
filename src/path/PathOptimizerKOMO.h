#ifndef OMPL_GEOMETRIC_PATH_OPTIMIZER_KOMO_
#define OMPL_GEOMETRIC_PATH_OPTIMIZER_KOMO_

#include <ompl/geometric/PathOptimizer.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/geometric/PathGeometric.h>

#include <KOMO/komo.h>
#include <Kin/viewer.h>
#include <Core/graph.h>

#include <string>

namespace ompl
{
    namespace geometric
    {
        OMPL_CLASS_FORWARD(PathOptimizerKOMO);

        class PathOptimizerKOMO : public PathOptimizer
        {
        private:
            std::shared_ptr<KOMO> komo_;
            
        public:
            // using PathOptimizer::PathOptimizer;
            PathOptimizerKOMO(base::SpaceInformationPtr si, std::shared_ptr<KOMO> komo_);
            virtual ~PathOptimizerKOMO() = default;

            bool optimize(PathGeometric &path) override;
        };  
    } // namespace  geometric
} //namespace ompl

#endif
