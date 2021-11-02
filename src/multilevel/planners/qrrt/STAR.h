#ifndef OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_STAR_
#define OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_STAR_
#include <multilevel/datastructures/BundleSpaceSequence.h>
#include <multilevel/planners/qrrt/STARImpl.h>

namespace ompl
{
    namespace multilevel
    {
        /* STAR: Sparse Trees over Abstract Representations */
        using STAR = BundleSpaceSequence<STARImpl>;

    }  // namespace multilevel
}  // namespace ompl

#endif
