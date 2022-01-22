#ifndef OMPL_GEOMETRIC_PKOMO_
#define OMPL_GEOMETRIC_PKOMO_

#include <ompl/geometric/planners/PlannerIncludes.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/base/State.h>
#include <KOMO/komo.h>
#include <Kin/viewer.h>
#include <Core/graph.h>
#include <limits>
#include <random>
#include <list>
#include <iterator>
#include <queue>

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

            bool flag{false};

            bool isValid{false};

            bool stateValid{false};

            void clear() override;

            ompl::base::RealVectorStateSpace *RN = si_->getStateSpace()->as<ompl::base::RealVectorStateSpace>();

            int dim = RN->getDimension();

            const std::vector<double> &bl = RN->getBounds().low;

            const std::vector<double> &bh = RN->getBounds().high;

            int gridLim;

            std::vector<unsigned long long> prod;

            unsigned long long cellCheck;

            double min_cost;

            unsigned long long gridCell;

            double delta;

            double threshold;

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
            PathGeometricPtr bestPoissonPath_list(double delta, const base::PlannerTerminationCondition &ptc);

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

                double getCostHeuristic() const
                {
                    return this->costHeuristic.value();
                }

                double getCost() const
                {
                    return this->cost.value();
                }

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

            /** \brief A nearest-neighbors datastructure containing the tree of motions */
            std::shared_ptr<NearestNeighbors<Motion *>> nn_;

            /** @brief The sole purpose of this list is to document the pointers of all motions generated
             * so that we can delete all of them later */
            std::vector<Motion*> motionList;

            std::list<std::pair<unsigned long long, Motion*>> gridList;
            std::list<std::pair<unsigned long long, Motion*>>::iterator it;

            void runNextNestedFor_list(std::vector<int> counters, int index, Motion* rmotion);

            struct CmpMotionPtrs
            {
                // ">" operator: smallest value is top in queue
                // "<" operator: largest value is top in queue (default)
                bool operator()(const Motion *lhs, const Motion *rhs) const
                {
                    return lhs->getCostHeuristic() < rhs->getCostHeuristic();
                }
            };
            /** \brief \brief Priority queue of Objects */
            typedef std::priority_queue<Motion*, std::vector<Motion*>, CmpMotionPtrs>
                MotionPriorityQueue;

            /** @brief An ordered list of active state arranged in decending order of cost heuristic*/
            MotionPriorityQueue activeList;
            // std::vector<Motion*> activeList;
        public:
            PKOMO(const base::SpaceInformationPtr &si, std::string filename);

            virtual ~PKOMO() override;

			base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

            void setup() override;
        };  
    } // namespace  geometric
} //namespace ompl

#endif