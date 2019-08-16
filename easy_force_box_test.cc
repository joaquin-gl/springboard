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
#include "drake/examples/springboard/easy_force.h"

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

DEFINE_double(duration, 15.0,
            "Desired duration of the simulation in seconds.");

DEFINE_double(time_step, 0,
            "If greater than zero, the plant is modeled as a system with "
            "discrete updates and period equal to this time_step. "
            "If 0, the plant is modeled as a continuous system.");

DEFINE_double(box_l, 1, "Side length of Box");
DEFINE_double(box_m, 1, "Mass of Box");

DEFINE_bool(rand_rpy, true, "Randomize initial Box rotation");

DEFINE_double(t0, 1, "Trigger time 0");
DEFINE_double(t1, 3, "Trigger time 1");
DEFINE_double(t2, 8, "Trigger time 2");
DEFINE_double(t, 0.4, "Torque");
DEFINE_double(f, 1, "Force");

int do_main() {
    systems::DiagramBuilder<double> builder;

    auto pair = AddMultibodyPlantSceneGraph(&builder, std::make_unique<MultibodyPlant<double>>(FLAGS_time_step));
    MultibodyPlant<double>& plant = pair.plant;

    SceneGraph<double>& scene_graph = pair.scene_graph;
    scene_graph.set_name("scene_graph");

    // Add gravity to the model.
    // plant.AddForceElement<UniformGravityFieldElement>();

    // Add red box to model.

    // Add spatial inertia
    drake::multibody::SpatialInertia<double> M_Acm(
        FLAGS_box_m,
        Vector3d::Zero(),
        drake::multibody::UnitInertia<double>::SolidCube(FLAGS_box_l));

    const drake::multibody::RigidBody<double>& red_box = plant.AddRigidBody("red_box", M_Acm);

    // Visual Geometry
    plant.RegisterVisualGeometry(
        red_box, math::RigidTransformd(),
        geometry::Box(FLAGS_box_l, FLAGS_box_l, FLAGS_box_l),
        "red_box_visual",
        Eigen::Vector4d(1, 0.5, 0.5, 1));

    // Collision Geometry
    plant.RegisterCollisionGeometry(
        red_box, math::RigidTransformd(),
        geometry::Box(FLAGS_box_l, FLAGS_box_l, FLAGS_box_l),
        "red_box_collision",
        drake::multibody::CoulombFriction<double>(0.3, 0.3));


    // Add green box to model.

    const drake::multibody::RigidBody<double>& green_box = plant.AddRigidBody("green_box", M_Acm);

    // Visual Geometry
    plant.RegisterVisualGeometry(
        green_box, math::RigidTransformd(),
        geometry::Box(FLAGS_box_l, FLAGS_box_l, FLAGS_box_l),
        "green_box_visual",
        Eigen::Vector4d(0.5, 1, 0.5, 1));

    // Collision Geometry
    plant.RegisterCollisionGeometry(
        green_box, math::RigidTransformd(),
        geometry::Box(FLAGS_box_l, FLAGS_box_l, FLAGS_box_l),
        "green_box_collision",
        drake::multibody::CoulombFriction<double>(0.3, 0.3));


    // Now the model is complete.
    plant.Finalize();

    // Sanity check on the availability of the optional source id before using it.
    DRAKE_DEMAND(plant.geometry_source_is_registered());

    // Add EasyForce
    auto easy_force = builder.AddSystem(std::make_unique<EasyForce>());

    // connect EasyForce output port to plant input port
    builder.Connect(
        easy_force->get_externally_applied_spatial_force_output_port(),
        plant.get_applied_spatial_force_input_port());

    // CALL ALL CONNECTIONS BEFORE THESE TWO LINES:
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

    math::RigidTransform<double> X_WA; // Identity
    math::RigidTransform<double> X_WB; // Identity

    if (FLAGS_rand_rpy) {
        srand(time(NULL));

        math::RollPitchYaw<double> R_WA(
            2.0*M_PI*double(rand()) / double(RAND_MAX),
            2.0*M_PI*double(rand()) / double(RAND_MAX),
            2.0*M_PI*double(rand()) / double(RAND_MAX));

        X_WA = math::RigidTransformd(R_WA, Vector3d(-2, 0, 0));
        srand(time(NULL));

        srand(time(NULL));
        math::RollPitchYaw<double> R_WB(
            2.0*M_PI*double(rand()) / double(RAND_MAX),
            2.0*M_PI*double(rand()) / double(RAND_MAX),
            2.0*M_PI*double(rand()) / double(RAND_MAX));

        X_WB = math::RigidTransformd(R_WB, Vector3d(2, 0, 0));
    }

    plant.SetFreeBodyPose(&plant_context, plant.GetBodyByName("red_box"), X_WA);
    plant.SetFreeBodyPose(&plant_context, plant.GetBodyByName("green_box"), X_WB);

    simulator.set_publish_every_time_step(false);
    simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
    simulator.Initialize();

    // Using a while loop to step the sim as opposed to `simulator.StepTo(FLAGS_duration)` so that we
    // can periodically print the sim's context to command line for debugging.
    double current_time = 0.0;
    const double time_delta = 0.008;

    while(current_time < FLAGS_duration) {

        // Set BodyIndex to first box for target of force
        // Second box falls :,(
        // Set Force to balance gravity and small spin torque
        if (current_time >= FLAGS_t0 && current_time < FLAGS_t0 + time_delta) {
            std::cout << "Sending First Force...\n";
            easy_force->SetBodyIndex(0, plant.GetBodyByName("red_box").index());
            easy_force->SetForce(0, drake::multibody::SpatialForce<double>(
                Vector3d(0, 0, FLAGS_t),        // torque
                Vector3d(0, 0, 0)));     // force
            easy_force->AddForce(plant.GetBodyByName("green_box").index(),
            drake::multibody::SpatialForce<double>(
                Vector3d(0, 0, 0),
                Vector3d(0, FLAGS_f, 0)));
        }

        // Set Force higher in z direction, reverse spin torque
        if (current_time >= FLAGS_t1 && current_time < FLAGS_t1 + time_delta) {
            std::cout << "Increasing force...\n";
            easy_force->SetForce(0, drake::multibody::SpatialForce<double>(
                Vector3d(0, 0, -FLAGS_t),
                Vector3d(0, 0, 0)));
            easy_force->SetForce(1, drake::multibody::SpatialForce<double>(
                Vector3d(0, 0, 0),
                Vector3d(0, -FLAGS_f, 0)));
        }

        // Change BodyIndex to the second box, first box now falls
        if (current_time >= FLAGS_t2 && current_time < FLAGS_t2 + time_delta) {
            std::cout << "Changing Body...\n";
            easy_force->SetBodyIndex(0, plant.GetBodyByName("green_box").index());
            easy_force->SetBodyIndex(1, plant.GetBodyByName("red_box").index());
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
