#pragma once

#include <memory>

#include "drake/math/rigid_transform.h"
#include "drake/common/drake_copyable.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/query_object.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/multibody/plant/externally_applied_spatial_force.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace examples {

using Eigen::Vector3d;
using multibody::ExternallyAppliedSpatialForce;
using multibody::SpatialForce;
using multibody::BodyIndex;

class EasyForce : public systems::LeafSystem<double> {
public:
    DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(EasyForce)

    explicit EasyForce();

    const systems::OutputPort<double>& get_externally_applied_spatial_force_output_port() const;

    int AddForce(
        const BodyIndex body_index);
    int AddForce(
        const BodyIndex body_index,
        const SpatialForce<double> force);
    int AddForce(
        const BodyIndex body_index,
        const SpatialForce<double> force,
        const Vector3d position);

    void SetBodyIndex(const int spatial_force_index, const BodyIndex body_index);

    void SetForce(const int spatial_force_index, const SpatialForce<double> force);

    void SetPosition(const int spatial_force_index, const Vector3d p_BoF_B);

private:

    void SendForce(
        const systems::Context<double>&,
        std::vector<ExternallyAppliedSpatialForce<double>>* output) const;

    int spatial_force_output_port_{-1};

    std::vector<ExternallyAppliedSpatialForce<double>> spatial_force_state_;
};

}  // namespace examples
}  // namespace drake
