#ifndef OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_BITKOMO_
#define OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_BITKOMO_

#include <ompl/base/Planner.h>
#include <ompl/geometric/planners/informedtrees/BITstar.h>

namespace ompl
{
    namespace geometric
    {
        class BITKOMO : public BITstar
        {

        public:
            BITKOMO(const base::SpaceInformationPtr &si, const std::string &name = "BITKOMO");

            ~BITKOMO()override = default;

            /** \brief Set the inflation factor for the initial search. */
            void setInitialInflationFactor(double factor);

            /** \brief Set the parameter for the inflation factor update policy. */
            void setInflationScalingParameter(double parameter);

            /** \brief Set the parameter for the truncation factor update policy. */
            void setTruncationScalingParameter(double parameter);

            /** \brief Get the inflation factor for the initial search. */
            double getInitialInflationFactor() const;

            /** \brief Get the inflation scaling parameter. */
            double getInflationScalingParameter() const;

            /** \brief Get the truncation scaling parameter. */
            double getTruncationScalingParameter() const;

            /** \brief Get the inflation factor for the current search. */
            double getCurrentInflationFactor() const;

            /** \brief Get the truncation factor for the current search. */
            double getCurrentTruncationFactor() const;
        };
        
    } // namespace geometric
    
} // namespace ompl


#endif //OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_BITKOMO_