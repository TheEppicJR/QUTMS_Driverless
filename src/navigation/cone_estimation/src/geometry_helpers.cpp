#include "ros_msg_helpers.hpp"
#include <math.h>       /* cos */

#define PI 3.14159265

Matrix rotate_cov_about_phi(Matrix& covMatrix, double& phi)
{
    // this equation comes from here https://en.wikipedia.org/wiki/Rotation_matrix#:~:text=Basic%20rotations%5Bedit%5D-,A%20basic%20rotation,-(also%20called%20elemental

    Matrix rotMatrix(3, std::vector<double>(3, 0));
    rotMatrix[0][0] = cos ( phi * PI / 180.0 );
    rotMatrix[1][1] = cos ( phi * PI / 180.0 );
    rotMatrix[0][1] = sin ( phi * PI / 180.0 ) * -1.0;
    rotMatrix[1][0] = sin ( phi * PI / 180.0 );
    rotMatrix[2][2] = 1.0;

    Matrix rslt(3, std::vector<double>(3, 0));

    rslt = mulMat(mulMat(rotMatrix, covMatrix), );
}

template <size_t rows, size_t cols>
Matrix mulMat(Matrix& mat1, Matrix& mat2)
{
    Matrix rslt(3, std::vector<double>(3, 0));
 
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            rslt[i][j] = 0;
 
            for (int k = 0; k < 3; k++) {
                rslt[i][j] += mat1[i][k] * mat2[k][j];
            }
        }
    }
    
}