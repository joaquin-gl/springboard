#pragma once

#include <memory>

#include "drake/common/drake_optional.h"
#include "drake/common/find_resource.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace examples {

using drake::multibody::MultibodyPlant;
using Eigen::Vector3d;

multibody::BodyIndex AddBoxToPlant(
    const std::string name, const multibody::ModelInstanceIndex model_instance,
    const double width, const double depth, const double height, const double mass,
    MultibodyPlant<double>* plant);

multibody::BodyIndex AddCylinderToPlant(
    const std::string name, const multibody::ModelInstanceIndex model_instance,
    const double radius, const double height, const double mass,
    MultibodyPlant<double>* plant);

multibody::BodyIndex AddSphereToPlant(
    const std::string name, const multibody::ModelInstanceIndex model_instance,
    const double radius, const double mass,
    MultibodyPlant<double>* plant);

multibody::ModelInstanceIndex AddTableToPlant(
    const double width, const double depth, const double height,
    const math::RigidTransformd X_WT,
    MultibodyPlant<double>* plant);

}  // namespace examples
}  // namespace drake
