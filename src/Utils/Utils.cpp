#include <CentroidalPlanner/Utils/Utils.h>

Eigen::Affine3d cpl::utils::GetAffineFromNormal(const Eigen::Vector3d& n)
{
    Eigen::Matrix3d R;
    R.setZero();

    R.coeffRef(0, 0) =  n.y() / ((n.head(2)).norm());
    R.coeffRef(0, 1) = -n.x() / ((n.head(2)).norm());

    R.coeffRef(1, 0) = (n.x() * n.z()) / ((n.head(2)).norm());
    R.coeffRef(1, 1) = (n.y() * n.z()) / ((n.head(2)).norm());
    R.coeffRef(1, 2) = - (n.head(2)).norm();

    R.coeffRef(2, 0) = n.x();
    R.coeffRef(2, 1) = n.y();
    R.coeffRef(2, 2) = n.z();

    Eigen::Affine3d pose;
    pose.translation() = n;
    pose.linear() =  R.transpose();

    return pose;
}
