#include "driverless_msgs/msg/cone.hpp"
#include "driverless_msgs/msg/cone_detection_stamped.hpp"
#include "ros_msg_helpers.hpp"

Matrix row_major_to_array(RowMajor& row_major)
{
    Matrix full_array(3, RowMajor(3, 0));
    full_array[0][0] = row_major[0];
    full_array[0][1] = row_major[1];
    full_array[0][2] = row_major[2];
    full_array[1][0] = row_major[3];
    full_array[1][1] = row_major[4];
    full_array[1][2] = row_major[5];
    full_array[2][0] = row_major[6];
    full_array[2][1] = row_major[7];
    full_array[2][2] = row_major[8];
    return full_array;
}

RowMajor array_to_row_major(Matrix& full_array)
{
    RowMajor row_major(3, 0);
    row_major[0] = full_array[0][0];
    row_major[1] = full_array[0][1];
    row_major[2] = full_array[0][2];
    row_major[3] = full_array[1][0];
    row_major[4] = full_array[1][1];
    row_major[5] = full_array[1][2];
    row_major[6] = full_array[2][0];
    row_major[7] = full_array[2][1];
    row_major[8] = full_array[2][2];
    return row_major;
}