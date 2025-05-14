
# 查找 pybind11
find_package(pybind11 REQUIRED)

pybind11_add_module(livox_sdk livox_lidar_sdk_wrapper.cpp)

target_link_libraries(livox_sdk PUBLIC livox_lidar_sdk_shared pybind11::module)
    