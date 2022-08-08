#ifndef OMPL_BASE_GOALS_WEIGHTED_GOAL_STATES_
#define OMPL_BASE_GOALS_WEIGHTED_GOAL_STATES_

#include <ompl/base/goals/GoalStates.h>
#include <unordered_map>

#include <string>

namespace ompl
{
    namespace base
    {
        OMPL_CLASS_FORWARD(WeightedGoalStates);

        class WeightedGoalStates : public GoalStates
        {
        protected:
            std::shared_ptr<std::unordered_map<const State*,double>> goalWeights;
            std::vector<State*> badStates_;
            double calcPenalty(const State *st, State *badState);
            
        public:
            // using PathOptimizer::PathOptimizer;
            WeightedGoalStates(const SpaceInformationPtr &si) : GoalStates(si)
            {
                goalWeights = std::make_shared<std::unordered_map<const State*,double>>();
            }

            virtual ~WeightedGoalStates() = default;

            void add_penalty(State *badState);

            /** \brief Add a goal state */
            virtual void addState(const State *st) override;

            /** \brief Add a goal state (calls the previous definition of addState())*/
            void addState(const ScopedState<> &st);

            std::shared_ptr<std::unordered_map<const State*,double>> getMap()
            {
                return goalWeights;
            }
        };  
    } // namespace  geometric
} //namespace ompl

#endif
