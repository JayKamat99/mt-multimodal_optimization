#ifndef OMPL_GEOMETRIC_PKOMO_
#define OMPL_GEOMETRIC_PKOMO_

#include <ompl/geometric/planners/PlannerIncludes.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/State.h>
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

            std::string filename_;

            arrA OptimalPath;

            void clear() override;

            double dist(arr p1,arr p2);

            bool compare(arrA path,arrA OptimalPath,double threshold);

            ompl::base::RealVectorStateSpace *RN = si_->getStateSpace()->as<ompl::base::RealVectorStateSpace>();

            int dim = RN->getDimension();

            const std::vector<double> &bl = RN->getBounds().low;

            const std::vector<double> &bh = RN->getBounds().high;

            int gridLim;

            std::vector<unsigned long long> prod;

            unsigned long long cellCheck;

            double min_dist;

            unsigned long long gridCell;

            double delta;

            bool stateValid{false};
            
            /** \brief State sampler */
            base::StateSamplerPtr sampler_;

            /** \brief Objective we're optimizing */
            base::OptimizationObjectivePtr opt_;

            /**
             * @brief This is the function that is unique to our planner. 
             * The aim of the planner is to generate an optimal path from start to goal while sampling using poisson distribution
             * and generating the least cost path using A*. A valid path is then to be converted to arrA and returned.
             * 
             * @param delta defines the sparsity of the poisson sampling.
             * @return PathGeometricPtr 
             */
            PathGeometricPtr bestPoissonPath(double delta);
            PathGeometricPtr bestPoissonPath_grid(double delta);

            /**
             * @brief This function uses the Bos-Muller transform method for generating a uniform random unit vector.             * 
             */
            void generate_randomUnitVector();

            void generate_grid(double delta);

            /** \brief Representation of a motion
                This only contains pointers to parent motions as we
                only need to go backwards in the tree. */
            class Motion
            {
            public:
                Motion() = default;

                /** \brief Constructor that allocates memory for the state */
                Motion(const base::SpaceInformationPtr &si) : state(si->allocState())
                {
                }

                ~Motion() = default;

                /** \brief The state contained by the motion */
                base::State *state{nullptr};

                /** \brief The parent motion in the exploration tree */
                Motion *parent{nullptr};

                /** \brief The cost up to this motion */
                base::Cost cost;

                base::Cost costHeuristic;
            };

            /** \brief Compute distance between motions (actually distance between contained states) */
            double distanceFunction(const Motion *a, const Motion *b) const
            {
                return si_->distance(a->state, b->state);
            }

            /** @brief An ordered list of active state arranged in decending order of cost heuristic*/
            std::vector<Motion*> activeList;

            /** @brief The sole purpose of this list is to document the pointers of all motions generated
             * so that we can delete all of them later */
            std::vector<Motion*> motionList;

            void runNextNestedFor(std::vector<int> counters, int index, Motion* rmotion, std::shared_ptr<std::vector<Motion *>> gridArray);

        public:
            PKOMO(const base::SpaceInformationPtr &si, std::string filename);

            virtual ~PKOMO() override;

			base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

            void setup() override;
        };  
    } // namespace  geometric
} //namespace ompl

#endif