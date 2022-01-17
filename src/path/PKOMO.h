#ifndef OMPL_GEOMETRIC_PKOMO_
#define OMPL_GEOMETRIC_PKOMO_

#include <ompl/geometric/planners/PlannerIncludes.h>
#include <KOMO/komo.h>
#include <Kin/viewer.h>
#include <Core/graph.h>
#include <limits>
#include <random>

namespace ompl
{
    namespace geometric
    {
        OMPL_CLASS_FORWARD(PKOMO);

        class PKOMO : public base::Planner
        {
        protected:
            void freeMemory();
			double bestCost = std::numeric_limits<double>::infinity();
			std::string bestCostProperty()
            {
                return std::to_string(bestCost);
            }
            arrA OptimalPath;
            void clear() override;
            double dist(arr p1,arr p2);
            bool compare(arrA path,arrA OptimalPath,double threshold);
            int dim;
            std::default_random_engine generator;

            /**
             * @brief This is the function that is unique to our planner. 
             * The aim of the planner is to generate an optimal path from start to goal while sampling using poisson distribution
             * and generating the least cost path using A*. A valid path is then to be converted to arrA and returned.
             * 
             * @param delta defines the sparsity of the poisson sampling.
             * @return arrA 
             */
            arrA bestPoissonPath(double delta);

            /**
             * @brief This function uses the Bos-Muller transform method for generating a uniform random unit vector.             * 
             */
            void generate_randomUnitVector();

        public:
            PKOMO(const base::SpaceInformationPtr &si);
            virtual ~PKOMO() override;

			base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;
			// std::string filename;
        };  
    } // namespace  geometric
} //namespace ompl

#endif