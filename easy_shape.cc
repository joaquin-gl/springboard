#include "drake/examples/springboard/easy_shape.h"

namespace drake {
namespace examples {

multibody::BodyIndex AddBoxToPlant(
    std::string name, multibody::ModelInstanceIndex model_instance,
    double width, double depth, double height, double mass,
    MultibodyPlant<double>* plant) {

    DRAKE_THROW_UNLESS(plant != nullptr);

    // Add spatial inertia
    drake::multibody::SpatialInertia<double> M_Bcm(
        mass,
        Vector3d::Zero(),
        drake::multibody::UnitInertia<double>::SolidBox(width, depth, height));

    // Add a rigid body
    const drake::multibody::RigidBody<double>& body = plant->AddRigidBody(name, model_instance, M_Bcm);

    plant->RegisterVisualGeometry(
        body, math::RigidTransformd(),
        geometry::Box(width, depth, height),
        "box_visual",
        Eigen::Vector4d(0.5, 0.5, 0.5, 0.5));

    // Add friction model
    plant->RegisterCollisionGeometry(
        body, math::RigidTransformd(),
        geometry::Box(width, depth, height),
        "box_collision",
        drake::multibody::CoulombFriction<double>(0.3, 0.3));

    return body.index();
    }

multibody::BodyIndex AddCylinderToPlant(
    std::string name, multibody::ModelInstanceIndex model_instance,
    double radius, double height, double mass,
    MultibodyPlant<double>* plant) {

    DRAKE_THROW_UNLESS(plant != nullptr);

    // Add spatial inertia
    drake::multibody::SpatialInertia<double> M_Ccm(
        mass,
        Vector3d::Zero(),
        drake::multibody::UnitInertia<double>::SolidCylinder(radius, height, Vector3d::UnitZ()));

    // Add a rigid body
    const drake::multibody::RigidBody<double>& body = plant->AddRigidBody(name, model_instance, M_Ccm);

    plant->RegisterVisualGeometry(
        body, math::RigidTransformd(),
        geometry::Cylinder(radius, height),
        "cylinder_visual",
        Eigen::Vector4d(0.5, 0.5, 0.5, 0.5));

    plant->RegisterCollisionGeometry(
        body, math::RigidTransformd(),
        geometry::Cylinder(radius, height),
        "cylinder_collision",
        drake::multibody::CoulombFriction<double>(0.3, 0.3));

    return body.index();
    }

multibody::BodyIndex AddSphereToPlant(
    std::string name, multibody::ModelInstanceIndex model_instance,
    double radius, double mass,
    MultibodyPlant<double>* plant) {

    DRAKE_THROW_UNLESS(plant != nullptr);

    // Add spatial inertia
    drake::multibody::SpatialInertia<double> M_Scm(
        mass,
        Vector3d::Zero(),
        drake::multibody::UnitInertia<double>::SolidSphere(radius));

    // Add a rigid body
    const drake::multibody::RigidBody<double>& body = plant->AddRigidBody(name, model_instance, M_Scm);

    plant->RegisterVisualGeometry(
        body, math::RigidTransformd(),
        geometry::Sphere(radius),
        "sphere_visual",
        Eigen::Vector4d(0.5, 0.5, 0.5, 0.5));

    plant->RegisterCollisionGeometry(
        body, math::RigidTransformd(),
        geometry::Sphere(radius),
        "sphere_collision",
        drake::multibody::CoulombFriction<double>(0.3, 0.3));

    return body.index();
    }

multibody::ModelInstanceIndex AddTableToPlant(
    double width, double depth, double height,
    math::RigidTransformd X_WT,
    MultibodyPlant<double>* plant) {

    DRAKE_THROW_UNLESS(plant != nullptr);

    plant->RegisterVisualGeometry(
        plant->world_body(), X_WT,
        geometry::Box(width, depth, height),
        "table_visual",
        Eigen::Vector4d(0.5, 0.5, 0.5, 1));

    plant->RegisterCollisionGeometry(
        plant->world_body(), X_WT,
        geometry::Box(width, depth, height),
        "table_collision",
        drake::multibody::CoulombFriction<double>(0.3, 0.3));

    return multibody::ModelInstanceIndex{0};
    }

}  // namespace examples
}  // namespace drake
