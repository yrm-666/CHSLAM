#pragma once
#include <gtsam/slam/OrientedPlane3Factor.h>
#include <gtsam/geometry/OrientedPlane3.h>

namespace co_lrio
{

// gtsam::OrientedPlane3Factor in GTSAM 4.2a8 is missing clone(), which is
// required by GncOptimizer. This thin subclass supplies the missing method.
class GroundPlaneFactor : public gtsam::OrientedPlane3Factor
{
public:
    using gtsam::OrientedPlane3Factor::OrientedPlane3Factor;

    ~GroundPlaneFactor() override = default;

    gtsam::NonlinearFactor::shared_ptr clone() const override
    {
        return boost::make_shared<GroundPlaneFactor>(*this);
    }
};

}  // namespace co_lrio
