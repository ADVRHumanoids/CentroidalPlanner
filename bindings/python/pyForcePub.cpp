#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pybind11/operators.h>
#include <forza_giusta/ForcePublisher.h>

namespace py = pybind11;
using namespace force_publisher;

PYBIND11_MODULE(pyforcepub, m) {

    py::class_<ForcePublisher>(m, "ForcePublisher")
        .def(py::init<std::vector<std::string>>())
//        .def("sendForce", &ForcePublisher::send_force)
        .def("sendForce", (void (ForcePublisher::*)(const Eigen::VectorXd&))                           &ForcePublisher::send_force)
        .def("sendForce", (void (ForcePublisher::*)(std::vector<std::string>, const Eigen::VectorXd&)) &ForcePublisher::send_force)
        .def("sendNormal",(void (ForcePublisher::*)(const Eigen::VectorXd&))                           &ForcePublisher::send_normal)
        .def("sendNormal",(void (ForcePublisher::*)(std::vector<std::string>, const Eigen::VectorXd&)) &ForcePublisher::send_normal)
        .def("setLiftedContacts", &ForcePublisher::setLiftedContacts)
        .def("setContacts",  &ForcePublisher::setContacts)
        .def("setPointContacts",  &ForcePublisher::setPointContacts)
        .def("switchController", &ForcePublisher::switch_controller)
//        .def("sendWrenchManip", &ForcePublisher::send_wrench_manip)
//        .def("sendForceArms", &ForcePublisher::send_force_arms)
        ;
}


//        .def("sendForce", (void (ForcePublisher::*)(std::vector<std::string>, Eigen::VectorXd)), &ForcePublisher::send_force)
