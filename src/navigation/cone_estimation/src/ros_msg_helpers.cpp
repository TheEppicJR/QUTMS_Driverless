#include "driverless_msgs/msg/cone.hpp"
#include "driverless_msgs/msg/cone_detection_stamped.hpp"
#include "ros_msg_helpers.hpp"

struct arrCov row_major_to_array(struct arrCovRowMaj row_major)
{
    struct arrCov full_array;
    full_array.arr[0][0] = row_major.arr[0];
    full_array.arr[0][1] = row_major.arr[1];
    full_array.arr[0][2] = row_major.arr[2];
    full_array.arr[1][0] = row_major.arr[3];
    full_array.arr[1][1] = row_major.arr[4];
    full_array.arr[1][2] = row_major.arr[5];
    full_array.arr[2][0] = row_major.arr[6];
    full_array.arr[2][1] = row_major.arr[7];
    full_array.arr[2][2] = row_major.arr[8];
    return full_array;
}

struct arrCovRowMaj array_to_row_major(struct arrCov full_array)
{
    struct arrCovRowMaj row_major;
    row_major.arr[0] = full_array.arr[0][0];
    row_major.arr[1] = full_array.arr[0][1];
    row_major.arr[2] = full_array.arr[0][2];
    row_major.arr[3] = full_array.arr[1][0];
    row_major.arr[4] = full_array.arr[1][1];
    row_major.arr[5] = full_array.arr[1][2];
    row_major.arr[6] = full_array.arr[2][0];
    row_major.arr[7] = full_array.arr[2][1];
    row_major.arr[8] = full_array.arr[2][2];
    return row_major;
}