/**
 * Author: Bill(cocobill1987ATgmail.com)
 * Description: Demo
 * License: see the LICENSE.txt file
 */

#include <netinet/in.h> /* For htonl and ntohl */
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include <chrono>
#include <functional>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <thread>
#include <vector>

#include "find_3d_affine.h"
#include "tic_toc.h"

using namespace std;

// A function to test Find3DAffineTransform()
void TestFind3DAffineTransform(){

  // Create datasets with known transform
  int data_size = 10000;
  Eigen::Matrix3Xd in(3, data_size), out(3, data_size);
  Eigen::Quaternion<double> Q(1, 3, 5, 2);
  Q.normalize();
  Eigen::Matrix3d R = Q.toRotationMatrix();
  double scale = 3.9;
  for (int row = 0; row < in.rows(); row++) {
    for (int col = 0; col < in.cols(); col++) {
      in(row, col) = log(2*row + 10.0)/sqrt(1.0*col + 4.0) + sqrt(col*1.0)/(row + 1.0);
    }
  }
  Eigen::Vector3d S;
  S << -5, 6, -27;
  for (int col = 0; col < in.cols(); col++)
    out.col(col) = scale*R*in.col(col) + S;

  std::cout << "Designed rotation:\n" << R << std::endl
            << "and translation:\n" << S << std::endl
            << "and scale: " << scale << std::endl;

  TicToc cal_rot;
  Eigen::Affine3d A = umeyama(in, out);
  std::cout << "Find3DAffineTransform cost: " << cal_rot.toc() << std::endl;

  double est_scale = (A.linear()*Eigen::Vector3d::Identity()).norm();
  std::cout << "Estimated rotation:\n" << A.linear()/est_scale << std::endl
            << "and translation:\n" << A.translation() << std::endl
            << "and scale: " << est_scale << std::endl;
  // See if we got the transform we expected
  if ( (scale*R-A.linear()).cwiseAbs().maxCoeff() > 1e-20 ||
       (S-A.translation()).cwiseAbs().maxCoeff() > 1e-20)
    std::cout << "Test failed!" << std::endl;
    std::cout << "Rotation error: " << (scale*R-A.linear()).cwiseAbs().maxCoeff() << std::endl
              << "Translation error: " << (S-A.translation()).cwiseAbs().maxCoeff() << std::endl;
}

int main(int argc, char **argv) {

  TestFind3DAffineTransform ();
  return 0;
}
