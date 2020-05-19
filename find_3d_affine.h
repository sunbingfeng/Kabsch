// This code is released in public domain

#include <iostream>
#include <Eigen/Geometry>


// Given two sets of 3D points, find the rotation + translation + scale
// which best maps the first set to the second.
// Source: http://en.wikipedia.org/wiki/Kabsch_algorithm

// The input 3D points are stored as columns.
Eigen::Affine3d kabsch(Eigen::Matrix3Xd in, Eigen::Matrix3Xd out) {

  // Default output
  Eigen::Affine3d A;
  A.linear() = Eigen::Matrix3d::Identity(3, 3);
  A.translation() = Eigen::Vector3d::Zero();

  if (in.cols() != out.cols())
    throw "Find3DAffineTransform(): input data mis-match";

  // First find the scale, by finding the ratio of sums of some distances,
  // then bring the datasets to the same scale.
  double dist_in = 0, dist_out = 0;
  for (int col = 0; col < in.cols()-1; col++) {
    dist_in  += (in.col(col+1) - in.col(col)).norm();
    dist_out += (out.col(col+1) - out.col(col)).norm();
  }
  if (dist_in <= 0 || dist_out <= 0)
    return A;
  double scale = dist_out/dist_in;
  out /= scale;

  // Find the centroids then shift to the origin
  Eigen::Vector3d in_ctr = Eigen::Vector3d::Zero();
  Eigen::Vector3d out_ctr = Eigen::Vector3d::Zero();
  for (int col = 0; col < in.cols(); col++) {
    in_ctr  += in.col(col);
    out_ctr += out.col(col);
  }
  in_ctr /= in.cols();
  out_ctr /= out.cols();
  for (int col = 0; col < in.cols(); col++) {
    in.col(col)  -= in_ctr;
    out.col(col) -= out_ctr;
  }

  // SVD
  Eigen::MatrixXd Cov = in * out.transpose();
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(Cov, Eigen::ComputeThinU | Eigen::ComputeThinV);

  // Find the rotation
  double d = (svd.matrixV() * svd.matrixU().transpose()).determinant();
  if (d > 0)
    d = 1.0;
  else
    d = -1.0;
  Eigen::Matrix3d I = Eigen::Matrix3d::Identity(3, 3);
  I(2, 2) = d;
  Eigen::Matrix3d R = svd.matrixV() * I * svd.matrixU().transpose();

  // The final transform
  A.linear() = scale * R;
  A.translation() = scale*(out_ctr - R*in_ctr);

  return A;
}

// The input 3D points are stored as columns.
Eigen::Affine3d umeyama(const Eigen::Matrix3Xd &in_, const Eigen::Matrix3Xd &out_) {

  Eigen::Matrix3Xd in = in_;
  Eigen::Matrix3Xd out = out_;

  // Default output
  Eigen::Affine3d A;
  A.linear() = Eigen::Matrix3d::Identity(3, 3);
  A.translation() = Eigen::Vector3d::Zero();

  if (in.cols() != out.cols())
    throw "Find3DAffineTransform(): input data mis-match";

  int n = in.cols();

  // Find the centroids then shift to the origin
  Eigen::Vector3d u_in = Eigen::Vector3d::Zero();
  Eigen::Vector3d u_out = Eigen::Vector3d::Zero();
  for (int col = 0; col < in.cols(); col++) {
    u_in  += in.col(col);
    u_out += out.col(col);
  }
  u_in /= in.cols();
  u_out /= out.cols();
  for (int col = 0; col < in.cols(); col++) {
    in.col(col)  -= u_in;
    out.col(col) -= u_out;
  }

  // SVD
  Eigen::MatrixXd Cov = 1.0 / n * out * in.transpose();
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(Cov, Eigen::ComputeThinU | Eigen::ComputeThinV);

  // Find the rotation
  double d = (svd.matrixU() * svd.matrixV().transpose()).determinant();
  if (d > 0)
    d = 1.0;
  else
    d = -1.0;
  Eigen::Matrix3d I = Eigen::Matrix3d::Identity(3, 3);
  I(2, 2) = d;
  Eigen::Matrix3d R = svd.matrixU() * I * svd.matrixV().transpose();

  // Find the scale and translation
  double cov_in = 1.0 / n * (in.transpose() * in).trace();

  int singular_size = svd.singularValues().size();
  double tr_DS = 0;
  for(int i = 0; i < singular_size; i++)  
  {
    tr_DS += I(i, i) * svd.singularValues()[i];
  }
  double scale = 1.0 / cov_in * tr_DS;
  Eigen::Vector3d trans = u_out - scale * R * u_in; 

  // The final transform
  A.linear() = scale * R;
  // A.translation() = scale*(out_ctr - R*in_ctr);
  A.translation() = trans;

  return A;
}


