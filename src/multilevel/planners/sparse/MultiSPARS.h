#ifndef OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_MultiSPARS_
#define OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_MultiSPARS_

#include <multilevel/datastructures/BundleSpaceSequence.h>
#include <multilevel/planners/sparse/MultiSPARSImpl.h>

namespace ompl
{
    namespace multilevel
    {
        using MultiSPARS = BundleSpaceSequence<MultiSPARSImpl>;
    }
}
#endif
