#include <CentroidalPlanner/Utils/Utils.h>

Eigen::Affine3d cpl::utils::GetAffineFromNormal(const Eigen::Vector3d& p, const Eigen::Vector3d& n)
{
    Eigen::Matrix3d R;
    R.setZero();

    double eps = 1e-8;
    auto n_vec = -n;
    auto n_norm = n.head(2).norm() + eps;

    R.coeffRef(0, 0) =  n_vec.y() / n_norm;
    R.coeffRef(0, 1) = -n_vec.x() / n_norm;

    R.coeffRef(1, 0) = (n_vec.x() * n_vec.z()) / n_norm;
    R.coeffRef(1, 1) = (n_vec.y() * n_vec.z()) / n_norm;
    R.coeffRef(1, 2) = - n_norm;

    R.coeffRef(2, 0) = n_vec.x();
    R.coeffRef(2, 1) = n_vec.y();
    R.coeffRef(2, 2) = n_vec.z();

    Eigen::Affine3d pose;
    pose.translation() = p;
    pose.linear() =  R.transpose();

    return pose;
}
