#include <pybind11/pybind11.h>
namespace py = pybind11;

#include <UnitreeCameraSDK.hpp>
#include <unistd.h>

PYBIND11_MODULE(unitree_camera_sdk, m){
    py::class_<UnitreeCamera>(m, "UnitreeCamera")
        .def(py::init<std::string>())
        .def("is_opened", [](UnitreeCamera &self){
            return self.isOpened();
        });
}