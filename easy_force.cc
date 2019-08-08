#include "drake/examples/springboard/easy_force.h"

namespace drake {
namespace examples {

EasyForce::EasyForce() {
    spatial_force_output_port_ = this->DeclareAbstractOutputPort(
        "easy_force_output",
        &EasyForce::SendForce).get_index();

        ExternallyAppliedSpatialForce<double> default_;
        default_.body_index = BodyIndex(0);
        default_.p_BoBq_B = Vector3d::Zero();
        default_.F_Bq_W = SpatialForce<double>(Vector3d::Zero(), Vector3d::Zero());

        spatial_force_state_.push_back(default_); // add default blank force to world body
}

const systems::OutputPort<double> &EasyForce::get_externally_applied_spatial_force_output_port() const {
    return systems::System<double>::get_output_port(spatial_force_output_port_); // for script to connect to input port
}

void EasyForce::SendForce(
    const systems::Context<double>&,
    std::vector<ExternallyAppliedSpatialForce<double>>* output) const {

    output->resize(spatial_force_state_.size()); // make sure output vector is right size
    (*output) = spatial_force_state_; // populate output ExternallyAppliedSpatialForce vector
}

int EasyForce::AddForce(const BodyIndex body_index) {
    // const int spatial_force_index = spatial_force_state_.size();

    ExternallyAppliedSpatialForce<double> spatial_force_;
    spatial_force_.body_index = body_index;
    spatial_force_.p_BoBq_B = Vector3d::Zero();
    spatial_force_.F_Bq_W = SpatialForce<double>(Vector3d::Zero(), Vector3d::Zero());

    spatial_force_state_.push_back(spatial_force_); // add blank force to body
    return spatial_force_state_.size()-1;
}

int EasyForce::AddForce(const BodyIndex body_index, const SpatialForce<double> force) {
    // const int spatial_force_index = spatial_force_state_.size();

    ExternallyAppliedSpatialForce<double> spatial_force_;
    spatial_force_.body_index = body_index;
    spatial_force_.p_BoBq_B = Vector3d::Zero();
    spatial_force_.F_Bq_W = force;

    spatial_force_state_.push_back(spatial_force_); // add force to body
    return spatial_force_state_.size()-1;
}

int EasyForce::AddForce(const BodyIndex body_index, const SpatialForce<double> force, const Vector3d position) {
    // const int spatial_force_index = spatial_force_state_.size();

    ExternallyAppliedSpatialForce<double> spatial_force_;
    spatial_force_.body_index = body_index;
    spatial_force_.p_BoBq_B = position;
    spatial_force_.F_Bq_W = force;

    spatial_force_state_.push_back(spatial_force_); // add force to body in position
    return spatial_force_state_.size()-1;
}

// Following three functions just set different parts of the ExternallyAppliedSpatialForce

void EasyForce::SetBodyIndex(const int spatial_force_index, const BodyIndex body_index) {
    // body_index_ = body_index;
    spatial_force_state_[spatial_force_index].body_index = body_index;
}

void EasyForce::SetForce(const int spatial_force_index, const SpatialForce<double> force) {
    // force_ = force;
    spatial_force_state_[spatial_force_index].F_Bq_W = force;
}

void EasyForce::SetPosition(const int spatial_force_index, const Vector3d position) {
    // position_ = position;
    spatial_force_state_[spatial_force_index].p_BoBq_B = position;
}

}  // namespace examples
}  // namespace drake
