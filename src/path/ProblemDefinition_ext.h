#ifndef OMPL_BASE_PROBLEM_DEFINITION_EXT_
#define OMPL_BASE_PROBLEM_DEFINITION_EXT_

#include <ompl/base/ProblemDefinition.h>

/**
 * @brief The reason for having this extension is add more functions like,
 * 1) Asking for a solution path given goal state.
 * 
 */

namespace ompl
{
    namespace base
    {
        OMPL_CLASS_FORWARD(ProblemDefinition_ext);

        class ProblemDefinition_ext : public base::ProblemDefinition
        {
        public:
            ProblemDefinition_ext(SpaceInformationPtr si) : ProblemDefinition(si) {}
            virtual ~ProblemDefinition_ext() override {}
        };  
    } // namespace base
} //namespace ompl

#endif