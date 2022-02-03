#include <path/BITKOMO.h>

ompl::geometric::BITKOMO::BITKOMO(const base::SpaceInformationPtr &si, const std::string &name /* = "BITKOMO" */) : ompl::geometric::BITstar(si, name)
{
    std::cout << "BITKOMO created" << std::endl;
    // Enable cascading rewirings.
    enableCascadingRewirings(true);

    // Set the default initial inflation factor to very high.
    setInitialInflationFactor(1000000.0);

    // Set the default inflation factor parameter to something reasonable.
    setInflationScalingParameter(10.0);

    // Set the default truncation factor parameter to something reasonable.
    setTruncationScalingParameter(5.0);

    // Declare the planner parameters.
    Planner::declareParam<double>("initial_inflation_factor", this, &BITKOMO::setInitialInflationFactor,
                                    &BITKOMO::getInitialInflationFactor, "1.0:0.01:1000000.0");
    Planner::declareParam<double>("inflation_scaling_parameter", this, &BITKOMO::setInflationScalingParameter,
                                    &BITKOMO::getInflationScalingParameter, "1.0:0.01:1000000.0");
    Planner::declareParam<double>("truncation_scaling_parameter", this, &BITKOMO::setTruncationScalingParameter,
                                    &BITKOMO::getTruncationScalingParameter, "1.0:0.01:1000000.0");
}

    void ompl::geometric::BITKOMO::setInitialInflationFactor(double factor)
    {
        BITstar::setInitialInflationFactor(factor);
    }

    void ompl::geometric::BITKOMO::setInflationScalingParameter(double factor)
    {
        BITstar::setInflationScalingParameter(factor);
    }

    void ompl::geometric::BITKOMO::setTruncationScalingParameter(double factor)
    {
        BITstar::setTruncationScalingParameter(factor);
    }

    double ompl::geometric::BITKOMO::getInitialInflationFactor() const
    {
        return BITstar::getInitialInflationFactor();
    }

    double ompl::geometric::BITKOMO::getInflationScalingParameter() const
    {
        return BITstar::getInflationScalingParameter();
    }

    double ompl::geometric::BITKOMO::getTruncationScalingParameter() const
    {
        return BITstar::getTruncationScalingParameter();
    }

    double ompl::geometric::BITKOMO::getCurrentInflationFactor() const
    {
        return BITstar::getCurrentInflationFactor();
    }

    double ompl::geometric::BITKOMO::getCurrentTruncationFactor() const
    {
        return BITstar::getCurrentTruncationFactor();
    }
