#include <mppi_py_interface/mppi_py_interface.h>
namespace py = pybind11;

int add(int i, int j)
{
    return i + j;
}

PYBIND11_MODULE(mppi_py_interface, m)
{
    m.doc() = "MPPI library python interface";

    //define the Python function name, the corresponding C++ function name, and a description
    m.def("add", &add, "A function which adds two numbers");
}