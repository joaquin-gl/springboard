#include "drake/examples/springboard/csv_logger.h"

namespace drake {
namespace examples {

CSVLogger::CSVLogger(
    const std::string output_filename,
    const std::vector<std::string> header_names) {

    n_columns_ = header_names.size();

    // open file to write
    const std::string file = FindResourceOrThrow(output_filename);
    output_file_.open(file);
    DRAKE_DEMAND(output_file_.is_open());

    for (std::string name : header_names) {
        output_file_ << name;
        if (name != header_names.back()) output_file_ << ",";
    }
    output_file_ << std::endl;

}

void CSVLogger::Log(const std::vector<double> output_vector) {
    for (double output : output_vector) {
        output_file_ << std::to_string(output);
        if (output != output_vector.back()) output_file_ << ",";
    }
    output_file_ << std::endl;
}

}  // namespace examples
}  // namespace drake
