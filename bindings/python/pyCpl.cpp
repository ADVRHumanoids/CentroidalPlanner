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

    py::class_<env::Superquadric, env::EnvironmentClass, std::shared_ptr<env::Superquadric>>(m, "Superquadric")
        .def(py::init())
        .def("SetParameters", &env::Superquadric::SetParameters)
        .def("GetParameters", &env::Superquadric::GetParameters);
        
    py::class_<solver::ContactValues>(m, "ContactValues")
        .def_readonly("force", &solver::ContactValues::force_value)
        .def_readonly("position", &solver::ContactValues::position_value)
        .def_readonly("normal", &solver::ContactValues::normal_value);
        
    py::class_<solver::Solution>(m, "Solution")
        .def("__repr__", print_solution)
        .def_readonly("com", &solver::Solution::com_sol)
        .def_readonly("contact_values_map", &solver::Solution::contact_values_map);

    auto base = py::class_<CentroidalPlanner>(m, "CentroidalPlanner")
    .def(py::init<std::vector<std::string>,
                  double,
                  env::EnvironmentClass::Ptr>())
    .def("Solve", &CentroidalPlanner::Solve)
    .def("SetCoMWeight", &CentroidalPlanner::SetCoMWeight)
    .def("SetPosWeight", &CentroidalPlanner::SetPosWeight)
    .def("SetForceWeight", &CentroidalPlanner::SetForceWeight)
    .def("SetCoMRef", &CentroidalPlanner::SetCoMRef)
    .def("SetPosRef", &CentroidalPlanner::SetPosRef)
    .def("SetForceThreshold", &CentroidalPlanner::SetForceThreshold)
    .def("SetPosBounds", &CentroidalPlanner::SetPosBounds)
    .def("SetForceBounds", &CentroidalPlanner::SetForceBounds)
    .def("SetManipulationWrench", &CentroidalPlanner::SetManipulationWrench)
    .def("GetForceThreshold", &CentroidalPlanner::GetForceThreshold);
    
    
    py::class_<CoMPlanner>(m, "CoMPlanner", base)
    .def(py::init<std::vector<std::string>,
                  double>())
    .def("SetLiftingContact", &CoMPlanner::SetLiftingContact)
    .def("ResetLiftingContact", &CoMPlanner::ResetLiftingContact)
    .def("SetContactPosition", &CoMPlanner::SetContactPosition)
    .def("GetContactPosition", &CoMPlanner::GetContactPosition)
    .def("SetContactNormal", &CoMPlanner::SetContactNormal)
    .def("SetMu", &CoMPlanner::SetMu);

}
