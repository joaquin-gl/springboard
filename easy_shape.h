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
    std::string name, multibody::ModelInstanceIndex model_instance,
    double width, double depth, double height, double mass,
    MultibodyPlant<double>* plant);

multibody::BodyIndex AddCylinderToPlant(
    std::string name, multibody::ModelInstanceIndex model_instance,
    double radius, double height, double mass,
    MultibodyPlant<double>* plant);

multibody::BodyIndex AddSphereToPlant(
    std::string name, multibody::ModelInstanceIndex model_instance,
    double radius, double mass,
    MultibodyPlant<double>* plant);

multibody::ModelInstanceIndex AddTableToPlant(
    double width, double depth, double height,
    math::RigidTransformd X_WT,
    MultibodyPlant<double>* plant);

}  // namespace examples
}  // namespace drake
