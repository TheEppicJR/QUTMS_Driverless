#include "ros_msg_helpers.hpp"//"cone_estimation/include/ros_msg_helpers.hpp"
#include <math.h>       /* cos */

#define PI 3.14159265

// Matrix rotate_cov_about_phi(Matrix& covMatrix, double& phi)
// {
//     // this equation comes from here https://en.wikipedia.org/wiki/Rotation_matrix#:~:text=Basic%20rotations%5Bedit%5D-,A%20basic%20rotation,-(also%20called%20elemental

//     Matrix rotMatrix(3, std::vector<double>(3, 0));
//     rotMatrix[0][0] = cos ( phi * PI / 180.0 );
//     rotMatrix[1][1] = cos ( phi * PI / 180.0 );
//     rotMatrix[0][1] = sin ( phi * PI / 180.0 ) * -1.0;
//     rotMatrix[1][0] = sin ( phi * PI / 180.0 );
//     rotMatrix[2][2] = 1.0;

//     Matrix rslt(3, std::vector<double>(3, 0));

// }
