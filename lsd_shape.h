#pragma once

#include <memory>

#include "drake/common/find_resource.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/linear_spring_damper.h"

namespace drake {
namespace examples {

using drake::multibody::MultibodyPlant;
using drake::multibody::LinearSpringDamper;
using Eigen::Vector3d;

multibody::ModelInstanceIndex AddLSDBoxToPlant(
    std::string name, double div_radius,
    double width, double depth, double height,
    double mass, double k, double b,
    MultibodyPlant<double>* plant);

void SetLSDPose(
    systems::Context<double>* context,
    multibody::ModelInstanceIndex model_instance_index,
    std::vector<math::RigidTransformd> X_WD_vector,
    MultibodyPlant<double>* plant);

}  // namespace examples
}  // namespace drake
