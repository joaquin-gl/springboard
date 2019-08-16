#pragma once

#include <fstream>
#include <iostream>
#include <vector>
#include <string>

#include "drake/common/find_resource.h"

namespace drake {
namespace examples {

class CSVLogger {

public:

    explicit CSVLogger(
        const std::string output_filename,
        const std::vector<std::string> header_names);

    void Log(const std::vector<double> output_vector);

private:

    unsigned int n_columns_{0};
    std::ofstream output_file_;
};

}  // namespace examples
}  // namespace drake
