#include "drake/examples/springboard/convenience.h"

namespace drake {
namespace examples {

void PrintMBPStats(MultibodyPlant<double>* plant) {
    std::cout << "\n";
    std::cout << plant->num_frames() << " frames\n";
    std::cout << plant->num_bodies() << " bodies\n";
    std::cout << plant->num_joints() << " joints\n";
    std::cout << plant->num_actuators() << " actuators\n";
    std::cout << plant->num_force_elements() << " force_elements\n";
    std::cout << plant->num_model_instances() << " model_instances\n";
    std::cout << "\n";
}

void PrintBodyIndices(MultibodyPlant<double>* plant) {
    std::cout << "\n";
    for(int mii=0; mii<plant->num_model_instances(); mii++) {
        ModelInstanceIndex model_instance_index(mii);
        std::cout << model_instance_index << " : " << plant->GetModelInstanceName(model_instance_index) << "\n";
        for(auto body_index : plant->GetBodyIndices(model_instance_index)) {
            const std::string name = plant->get_body(body_index).name();
            std::cout << "  " << body_index << " : " << name << "\n";
        };
        std::cout << "\n";
    };
    std::cout << "\n";
}

void PrintBodyStates(MultibodyPlant<double>* plant, systems::Context<double>* context) {
    std::cout << "\n";
    for(int mii=0; mii<plant->num_model_instances(); mii++) {
        ModelInstanceIndex model_instance_index(mii);
        std::cout << model_instance_index << " : " << plant->GetModelInstanceName(model_instance_index) << "\n";
        for(auto body_index : plant->GetBodyIndices(model_instance_index)) {
            const std::string name = plant->get_body(body_index).name();
            const Eigen::Vector3d position = plant->get_body(body_index).EvalPoseInWorld(*context).translation();
            std::cout << "  " << body_index << " : " << name << "\n";
            std::cout << "      position:\n";
            std::cout << position << "\n";
        };
        std::cout << "\n";
    };
    std::cout << "\n";
}

}  // namespace examples
}  // namespace drake
