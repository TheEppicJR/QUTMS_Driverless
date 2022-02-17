#include "driverless_msgs/msg/cone.hpp"
#include "driverless_msgs/msg/cone_detection_stamped.hpp"

struct arrCov {
    double arr[3][3];
};

struct arrCovRowMaj {
    double arr[9];
};

struct arrCov row_major_to_array(struct arrCovRowMaj row_major);


struct arrCovRowMaj array_to_row_major(struct arrCov full_array);