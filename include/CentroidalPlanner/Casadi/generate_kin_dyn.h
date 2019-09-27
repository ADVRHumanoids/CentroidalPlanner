#include <casadi/casadi.hpp>


#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/contact-dynamics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/contact-dynamics.hpp>

#include <urdf_parser/urdf_parser.h>

typedef casadi::SX Scalar;
typedef Eigen::Matrix<Scalar, -1, 1>  VectorXs;
typedef Eigen::Matrix<Scalar, -1, -1> MatrixXs;

VectorXs cas_to_eig(const casadi::SX& cas)
{
    VectorXs eig(cas.size1());
    for(int i = 0; i < eig.size(); i++)
    {
        eig(i) = cas(i);
    }
    return eig;
}

casadi::SX eig_to_cas(const VectorXs& eig)
{
    auto sx = casadi::SX(casadi::Sparsity::dense(eig.size()));
    for(int i = 0; i < eig.size(); i++)
    {
        sx(i) = eig(i);
    }
    return sx;

}

casadi::SX eigmat_to_cas(const MatrixXs& eig)
{
    auto sx = casadi::SX(casadi::Sparsity::dense(eig.rows(), eig.cols()));
    for(int i = 0; i < eig.rows(); i++)
    {
        for(int j = 0; j < eig.cols(); j++)
        {
            sx(i,j) = eig(i,j);
        }
    }
    return sx;

}



std::string generate_inv_dyn(std::string urdf_string)
{
    
    auto urdf = urdf::parseURDF(urdf_string);
    
    pinocchio::Model model_dbl;
    pinocchio::urdf::buildModel(urdf, model_dbl, true);
    pinocchio::Data data_dbl(model_dbl);

    auto model = model_dbl.cast<Scalar>();
    pinocchio::DataTpl<Scalar> data(model);
    int nq = model.nq;

//    for(int i = 0; i < nq; i++)
//    {
//        std::cout << model.getJointName(i) << std::endl;
//    }

    // casadi variabes
    casadi::SX u = casadi::SX::sym("u", nq); // control (qddot)
    casadi::SX q = casadi::SX::sym("q", nq), qdot = casadi::SX::sym("qdot", nq); // states

    // Compute expression for inverse dynamics with Pinocchio
    pinocchio::rnea(model, data, cas_to_eig(q), cas_to_eig(qdot), cas_to_eig(u));
    auto tau = eig_to_cas(data.tau);
//    auto tau_u = eig_to_cas(data.tau.head(6));
//    auto tau_a = eig_to_cas(data.tau.tail(nq-6-1));
    casadi::Function ID("inverse_dynamics", {q, qdot, u}, {tau}, {"q", "qdot", "qddot"}, {"tau"});
//    casadi::Function ID("inverse_dynamics", {q, qdot, u}, {tau_u,tau_a}, {"q", "qdot", "qddot"}, {"tau_u","tau_a"});

    std::stringstream ss;
    ss << ID.serialize();

    return ss.str();

}

std::string generate_forward_kin(std::string urdf_string, std::string body_name)
{

    auto urdf = urdf::parseURDF(urdf_string);

    pinocchio::Model model_dbl;
    pinocchio::urdf::buildModel(urdf, model_dbl, true);
    pinocchio::Data data_dbl(model_dbl);

    auto model = model_dbl.cast<Scalar>();
    pinocchio::DataTpl<Scalar> data(model);
    int nq = model.nq;

    // casadi variabes
    casadi::SX q = casadi::SX::sym("q", nq);

    auto frame_idx = model.getFrameId(body_name);

    // Compute expression for forward kinematics with Pinocchio
    pinocchio::framesForwardKinematics(model, data, cas_to_eig(q));
    auto eig_fk_pos = data.oMf.at(frame_idx).translation();
    auto eig_fk_rot = data.oMf.at(frame_idx).rotation();
    auto ee_position = eig_to_cas(eig_fk_pos);
    auto ee_rot = eigmat_to_cas(eig_fk_rot);
    casadi::Function FK("forward_kinematics", {q}, {ee_position, ee_rot}, {"q"}, {"ee_pos", "ee_rot"});

    std::stringstream ss;
    ss << FK.serialize();

    return ss.str();
}