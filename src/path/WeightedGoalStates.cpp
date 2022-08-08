#include "ompl/base/SpaceInformation.h"
#include "ompl/util/Exception.h"
#include "path/WeightedGoalStates.h"
#include <limits>

void ompl::base::WeightedGoalStates::addState(const State *st)
{
    states_.push_back(si_->cloneState(st));
    // add weights based on previous failure cases
    double penalty = 0;
    for (auto badState : badStates_)
    {
        penalty += calcPenalty(st, badState);
    }
    goalWeights->insert(std::make_pair(st,penalty));
}

double ompl::base::WeightedGoalStates::calcPenalty(const State *st, State *badState)
{
    double penalty = 0;
    if (st == badState)
    {
        std::cout << "I am adding a penalty" << std::endl;
        penalty = 2;
    } // No else
}

void ompl::base::WeightedGoalStates::addState(const ScopedState<> &st)
{
    addState(st.get());
}

void ompl::base::WeightedGoalStates::add_penalty(State *badState)
{
    badStates_.push_back(badState);
    for (auto state : states_)
    {
        double penalty = goalWeights->at(state);
        penalty += calcPenalty(state, badState);
        goalWeights->operator[](state) = penalty;
    }
}