#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pybind11/operators.h>
#include <CentroidalPlanner/CentroidalPlanner.h>
#include <CentroidalPlanner/CoMPlanner.h>

namespace py = pybind11;
using namespace cpl;

PYBIND11_MODULE(pycpl, m) {
    
    auto print_solution = [](const solver::Solution& sol)
    {
        std::stringstream ss;
        ss << sol;
        return ss.str();
    };
    
    py::class_<env::EnvironmentClass, std::shared_ptr<env::EnvironmentClass>>(m, "EnvironmentClass")
        .def("GetMu", &env::EnvironmentClass::GetMu)
        .def("SetMu", &env::EnvironmentClass::SetMu);
    
    py::class_<env::Ground, env::EnvironmentClass, std::shared_ptr<env::Ground>>(m, "Ground")
        .def(py::init())
        .def("SetGroundZ", &env::Ground::SetGroundZ);
        
    py::class_<solver::ContactValues>(m, "ContactValues")
        .def_readonly("force", &solver::ContactValues::force_value)
        .def_readonly("position", &solver::ContactValues::position_value)
        .def_readonly("normal", &solver::ContactValues::normal_value);
        
    py::class_<solver::Solution>(m, "Solution")
        .def("__repr__", print_solution)
        .def_readonly("com", &solver::Solution::com_sol)
        .def_readonly("contact_values_map", &solver::Solution::contact_values_map);

    py::class_<CentroidalPlanner>(m, "CentroidalPlanner")
    .def(py::init<std::vector<std::string>,
                  double,
                  env::EnvironmentClass::Ptr>())
    .def("Solve", &CentroidalPlanner::Solve)
    .def("SetCoMWeight", &CentroidalPlanner::SetCoMWeight)
    .def("SetPosWeight", &CentroidalPlanner::SetPosWeight)
    .def("SetForceWeight", &CentroidalPlanner::SetForceWeight);
    
    py::class_<CoMPlanner>(m, "CoMPlanner")
    .def(py::init<std::vector<std::string>,
                  double>())
    .def("Solve", &CoMPlanner::Solve)
    .def("SetLiftingContact", &CoMPlanner::SetLiftingContact)
    .def("SetContactPosition", &CoMPlanner::SetContactPosition)
    .def("SetContactNormal", &CoMPlanner::SetContactNormal)
    .def("SetCoMRef", &CoMPlanner::SetCoMRef)
    .def("ResetForceBounds", &CoMPlanner::ResetForceBounds)
    .def("SetMu", &CoMPlanner::SetMu)
    .def("SetForceThreshold", &CoMPlanner::SetForceThreshold)
    .def("GetContactPosition", &CoMPlanner::GetContactPosition)
    .def("SetCoMWeight", &CoMPlanner::SetCoMWeight)
    .def("SetPosWeight", &CoMPlanner::SetPosWeight)
    .def("SetForceWeight", &CoMPlanner::SetForceWeight)
    ;
}

