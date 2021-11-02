#ifndef OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_LOCALMINIMASPANNERS__
#define OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_LOCALMINIMASPANNERS__
#include <multilevel/datastructures/BundleSpaceSequence.h>
#include <multilevel/planners/multimodal/PathSpaceSparse.h>
#include <multilevel/planners/multimodal/datastructures/MultiLevelPathSpace.h>

namespace ompl
{
    namespace multilevel
    {
        using LocalMinimaSpanners = MultiLevelPathSpace<PathSpaceSparse>;
    }
}

#endif
