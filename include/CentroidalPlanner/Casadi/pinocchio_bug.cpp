#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/contact-dynamics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/contact-dynamics.hpp>

#include <urdf_parser/urdf_parser.h>

#include <casadi/casadi.hpp>

typedef casadi::SX Scalar;

void generate_forward_kin(std::string urdf_string, std::string body_name)
{

    auto urdf = urdf::parseURDF(urdf_string);

    pinocchio::Model model_dbl;
    pinocchio::urdf::buildModel(urdf, model_dbl, true);
    pinocchio::Data data_dbl(model_dbl);

    auto model = model_dbl.cast<Scalar>();
    pinocchio::DataTpl<Scalar> data(model);

    Eigen::Matrix<Scalar, -1, 1> q;
    Eigen::Matrix<Scalar, 6, -1> J;
    pinocchio::computeJointJacobians(model, data, q);
    pinocchio::framesForwardKinematics(model, data, q);
    pinocchio::frameJacobian(model, data, q, pinocchio::FrameIndex(), J);
}
