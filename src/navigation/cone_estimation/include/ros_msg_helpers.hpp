#include "driverless_msgs/msg/cone.hpp"
#include "driverless_msgs/msg/cone_detection_stamped.hpp"

using Matrix = std::vector<std::vector<double>>;
using RowMajor = std::vector<double>; 


Matrix row_major_to_array(RowMajor& row_major);


RowMajor array_to_row_major(Matrix& full_array);