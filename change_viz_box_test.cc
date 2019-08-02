#include <memory>

#include <gflags/gflags.h>

#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/geometry/scene_graph.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/uniform_gravity_field_element.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace examples {
namespace {

using geometry::SceneGraph;
using lcm::DrakeLcm;
using Eigen::Vector3d;

// "multibody" namespace is ambiguous here without "drake::".
using drake::multibody::MultibodyPlant;
using drake::multibody::UniformGravityFieldElement;

DEFINE_double(target_realtime_rate, 1.0,
            "Desired rate relative to real time.  See documentation for "
            "Simulator::set_target_realtime_rate() for details.");

DEFINE_double(duration, 5.0,
            "Desired duration of the simulation in seconds.");

DEFINE_double(time_step, 0,
            "If greater than zero, the plant is modeled as a system with "
            "discrete updates and period equal to this time_step. "
            "If 0, the plant is modeled as a continuous system.");

DEFINE_double(box_l, 1, "Side length of Box");
DEFINE_double(box_m, 1, "Mass of Box");

DEFINE_bool(rand_rpy, false, "Randomize initial Box rotation");

int do_main() {
    systems::DiagramBuilder<double> builder;

    auto pair = AddMultibodyPlantSceneGraph(&builder, std::make_unique<MultibodyPlant<double>>(FLAGS_time_step));
    MultibodyPlant<double>& plant = pair.plant;

    SceneGraph<double>& scene_graph = pair.scene_graph;
    scene_graph.set_name("scene_graph");

    // Add gravity to the model.
    plant.AddForceElement<UniformGravityFieldElement>();

    // Add box to model.
    math::RigidTransformd X_WA(Vector3d(0, 0, 2*FLAGS_box_l));
    if (FLAGS_rand_rpy) {
        srand(time(NULL));

        math::RollPitchYawd R_WA(
            2.0*M_PI*double(rand()) / double(RAND_MAX),
            2.0*M_PI*double(rand()) / double(RAND_MAX),
            2.0*M_PI*double(rand()) / double(RAND_MAX));

        X_WA = math::RigidTransformd(R_WA, Vector3d(0, 0, 2*FLAGS_box_l));
    }

    // Add spatial inertia
    drake::multibody::SpatialInertia<double> M_Acm(
        FLAGS_box_m,
        Vector3d::Zero(),
        drake::multibody::UnitInertia<double>::SolidCube(FLAGS_box_l));

    const drake::multibody::RigidBody<double>& box = plant.AddRigidBody("Box", M_Acm);

    // Visual Geometry
    geometry::GeometryId box_visual_id = plant.RegisterVisualGeometry(
        box, X_WA,
        geometry::Box(FLAGS_box_l, FLAGS_box_l, FLAGS_box_l),
        "BoxVisualGeometry",
        Eigen::Vector4d(0.7, 0.5, 0, 1));

    // Collision Geometry
    plant.RegisterCollisionGeometry(
        box, X_WA,
        geometry::Box(FLAGS_box_l, FLAGS_box_l, FLAGS_box_l),
        "BoxCollisionGeometry",
        drake::multibody::CoulombFriction<double>(0.3, 0.3));

    // Add floor to model.
    math::RigidTransformd X_WF;

    plant.RegisterVisualGeometry(
        plant.world_body(), X_WF,
        geometry::Box(4*FLAGS_box_l, 4*FLAGS_box_l, 0.5*FLAGS_box_l),
        "InclinedPlaneVisualGeometry",
        Eigen::Vector4d(0.5, 0, 0.7, 1));

    plant.RegisterCollisionGeometry(
        plant.world_body(), X_WF,
        geometry::Box(4*FLAGS_box_l, 4*FLAGS_box_l, 0.5*FLAGS_box_l),
        "InclinedPlaneCollisionGeometry",
        drake::multibody::CoulombFriction<double>(0.3, 0.3));

    // Now the model is complete.
    plant.Finalize();

    plant.set_penetration_allowance(1.0E-5);
    plant.set_stiction_tolerance(1.0E-5);

    // Sanity check on the availability of the optional source id before using it.
    DRAKE_DEMAND(plant.geometry_source_is_registered());

    // drake::multibody::ConnectContactResultsToDrakeVisualizer(&builder, plant);
    geometry::ConnectDrakeVisualizer(&builder, scene_graph);
    auto diagram = builder.Build();

    // Create a context for this system:
    std::unique_ptr<systems::Context<double>> diagram_context =
        diagram->CreateDefaultContext();
    diagram->SetDefaultContext(diagram_context.get());
    systems::Context<double>& plant_context =
        diagram->GetMutableSubsystemContext(plant, diagram_context.get());
    plant.SetDefaultContext(&plant_context);

    systems::Simulator<double> simulator(*diagram, std::move(diagram_context));
    auto& simulator_context = simulator.get_mutable_context();

    simulator.set_publish_every_time_step(false);
    simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
    simulator.Initialize();

    systems::IntegratorBase<double>& integrator =
        simulator.get_mutable_integrator();

    // Set the integration accuracy when the plant is integrated with a variable-
    // step integrator. This value is not used if time_step > 0 (fixed-time step).
    integrator.set_target_accuracy(1.0E-6);

    // Using a while loop to step the sim as opposed to `simulator.StepTo(FLAGS_duration)` so that we
    // can periodically print the sim's context to command line for debugging.
    double current_time = 0.0;
    const double time_delta = 0.008;
    bool changed(false);

    while( current_time < FLAGS_duration ){

        if (current_time > 2.0 && !changed) {
            std::cout << "Change color for id " << box_visual_id.get_value() << "\n";
            // Change color of box using its GeometryId
            changed = true;
        }
        simulator.StepTo(current_time + time_delta);
        current_time = simulator_context.get_time();
    }

    return 0;
}

}  // namespace
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage(
      "A simple EasyForce box demo, pushing around a"
      "box to demonstrate EasyForce."
      "Launch drake-visualizer before running this example.");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::logging::HandleSpdlogGflags();
  return drake::examples::do_main();
}
