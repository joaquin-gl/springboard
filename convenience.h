#pragma once

#include <memory>

#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace examples {

using drake::multibody::MultibodyPlant;
using drake::multibody::ModelInstanceIndex;

// prints some multibody plant statistics
void PrintMBPStats(MultibodyPlant<double>* plant);

// print diagram of ModelInstanceIndex and BodyIndex with their names
void PrintBodyIndices(MultibodyPlant<double>* plant);

// print diagram of ModelInstanceIndex and BodyIndex with their states
void PrintBodyStates(MultibodyPlant<double>* plant, systems::Context<double>* context);

}  // namespace examples
}  // namespace drake
