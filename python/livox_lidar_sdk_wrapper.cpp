/*****************************************************************************
*
* @file       livox_lidar_sdk_wrapper.cpp
* @author     Wei Wang -> shaxikai@outlook.com
* @date       2025.5.13
* @version    V1.0.0"
* @brief      livox python sdk
*
*****************************************************************************/


#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <iostream>
#include <mutex>
#include "livox_lidar_api.h"

namespace py = pybind11;
std::mutex callback_mutex;
py::object py_pointcloud_callback;
py::object py_imu_callback;

// init
const std::map<std::string, LivoxLidarDeviceType> dev_type_map = {
  {"HAP",     kLivoxLidarTypeIndustrialHAP},
  {"MID360",  kLivoxLidarTypeMid360}
};

class PyLivoxLidarNetCfg {
public:
    std::string lidar_name;
    std::string lidar_ipaddr;
    std::string lidar_subnet_mask;
    std::string lidar_gateway;
    std::string host_ip;
    std::string multicast_ip;

    uint16_t lidar_cmd_data_port;
    uint16_t lidar_push_msg_port;
    uint16_t lidar_point_data_port;
    uint16_t lidar_imu_data_port;
    uint16_t lidar_log_data_port;

    uint16_t host_cmd_data_port;
    uint16_t host_push_msg_port;
    uint16_t host_point_data_port;
    uint16_t host_imu_data_port;
    uint16_t host_log_data_port;
};

bool LivoxLidarSdkInitFromCfgWrapper(const PyLivoxLidarNetCfg& py_cfg) {
    LivoxLidarNetCfg cfg;

    strncpy(cfg.lidar_name, py_cfg.lidar_name.c_str(), sizeof(cfg.lidar_name));
    strncpy(cfg.lidar_ipaddr, py_cfg.lidar_ipaddr.c_str(), sizeof(cfg.lidar_ipaddr));
    strncpy(cfg.lidar_subnet_mask, py_cfg.lidar_subnet_mask.c_str(), sizeof(cfg.lidar_subnet_mask));
    strncpy(cfg.lidar_gateway, py_cfg.lidar_gateway.c_str(), sizeof(cfg.lidar_gateway));
    strncpy(cfg.host_ip, py_cfg.host_ip.c_str(), sizeof(cfg.host_ip));
    strncpy(cfg.multicast_ip, py_cfg.multicast_ip.c_str(), sizeof(cfg.multicast_ip));

    cfg.lidar_name[sizeof(cfg.lidar_name) - 1] = '\0';
    cfg.lidar_ipaddr[sizeof(cfg.lidar_ipaddr) - 1] = '\0';
    cfg.lidar_subnet_mask[sizeof(cfg.lidar_subnet_mask) - 1] = '\0';
    cfg.lidar_gateway[sizeof(cfg.lidar_gateway) - 1] = '\0';
    cfg.host_ip[sizeof(cfg.host_ip) - 1] = '\0';
    cfg.multicast_ip[sizeof(cfg.multicast_ip) - 1] = '\0';

    cfg.lidar_cmd_data_port = py_cfg.lidar_cmd_data_port;
    cfg.lidar_push_msg_port = py_cfg.lidar_push_msg_port;
    cfg.lidar_point_data_port = py_cfg.lidar_point_data_port;
    cfg.lidar_imu_data_port = py_cfg.lidar_imu_data_port;
    cfg.lidar_log_data_port = py_cfg.lidar_log_data_port;

    cfg.host_cmd_data_port = py_cfg.host_cmd_data_port;
    cfg.host_push_msg_port = py_cfg.host_push_msg_port;
    cfg.host_point_data_port = py_cfg.host_point_data_port;
    cfg.host_imu_data_port = py_cfg.host_imu_data_port;
    cfg.host_log_data_port = py_cfg.host_log_data_port;

    DisableLivoxSdkConsoleLogger();
    SaveLivoxLidarSdkLoggerFile();
    return LivoxLidarSdkInitFromCfg(cfg);
}

bool LivoxLidarSdkInitWrapper(const std::string path) {
    return LivoxLidarSdkInit(path.c_str(), "", nullptr);
}

// point cloud 
class PyLivoxLidarEthernetPacket {
public:
    uint16_t time_interval;
    uint16_t dot_num;
    uint64_t ts;
    uint8_t* data;

    PyLivoxLidarEthernetPacket(LivoxLidarEthernetPacket* packet)
        : time_interval(packet->time_interval),
          dot_num(packet->dot_num),
          data(packet->data) {
        std::memcpy(&ts, packet->timestamp, sizeof(ts));
    }

    py::bytes get_pts_data() const {
        size_t valid_size = dot_num * sizeof(LivoxLidarCartesianHighRawPoint);
        return py::bytes(reinterpret_cast<const char*>(data), valid_size);
    }
};

void PointCloudCallback(uint32_t handle, const uint8_t dev_type,
                        LivoxLidarEthernetPacket* data, void* client_data) {

    if (!data) {
        throw std::runtime_error("Null LivoxLidarEthernetPacket received");
    }

    py::gil_scoped_acquire acquire;
    std::lock_guard<std::mutex> lock(callback_mutex);
    if (py_pointcloud_callback && !py_pointcloud_callback.is_none()) {
        PyLivoxLidarEthernetPacket py_data(data);
        py_pointcloud_callback(py_data);
    }
}

void SetLivoxLidarPointCloudCallBackWrapper(py::function cb) {
    py_pointcloud_callback = std::move(cb); 
    SetLivoxLidarPointCloudCallBack(PointCloudCallback, nullptr);
}

// IMU
class PyLivoxLidarImuPacket {
public:
    uint64_t ts;
    std::array<float, 3> gyr;
    std::array<float, 3> acc;

    PyLivoxLidarImuPacket(LivoxLidarEthernetPacket* packet) {
        std::memcpy(&ts, packet->timestamp, sizeof(ts));
        LivoxLidarImuRawPoint* p_imu = reinterpret_cast<LivoxLidarImuRawPoint *>(packet->data);
        
        gyr = {p_imu->gyro_x, p_imu->gyro_y, p_imu->gyro_z};
        acc = {p_imu->acc_x, p_imu->acc_y, p_imu->acc_z};
    }
};


void ImuDataCallback(uint32_t handle, const uint8_t dev_type,
                     LivoxLidarEthernetPacket* data, void* client_data) {
                                                    
    if (!data) {
        throw std::runtime_error("Null LivoxLidarEthernetPacket received");
    }
    
    py::gil_scoped_acquire acquire;
    std::lock_guard<std::mutex> lock(callback_mutex);
    if (py_imu_callback && !py_imu_callback.is_none()) {
        PyLivoxLidarImuPacket py_data(data);
        py_imu_callback(py_data);
    }
}

void SetLivoxLidarImuDataCallbackWrapper(py::function cb) {
    py_imu_callback = std::move(cb); 
    SetLivoxLidarImuDataCallback(ImuDataCallback, nullptr);
}

void LivoxLidarSdkUninitWrapper() { 
    py::gil_scoped_acquire acquire;
    {
        std::lock_guard<std::mutex> lock(callback_mutex);
        py_pointcloud_callback = py::none();
        py_imu_callback = py::none();
    }
    SetLivoxLidarPointCloudCallBack(nullptr, nullptr);
    SetLivoxLidarImuDataCallback(nullptr, nullptr);
    LivoxLidarSdkUninit();
}

PYBIND11_MODULE(livox_sdk, m) {

    py::class_<PyLivoxLidarNetCfg>(m, "PyLivoxLidarCfg")
        .def(py::init<>())
        .def_readwrite("lidar_name", &PyLivoxLidarNetCfg::lidar_name)
        .def_readwrite("lidar_ipaddr", &PyLivoxLidarNetCfg::lidar_ipaddr)
        .def_readwrite("lidar_subnet_mask", &PyLivoxLidarNetCfg::lidar_subnet_mask)
        .def_readwrite("lidar_gateway", &PyLivoxLidarNetCfg::lidar_gateway)
        .def_readwrite("host_ip", &PyLivoxLidarNetCfg::host_ip)
        .def_readwrite("multicast_ip", &PyLivoxLidarNetCfg::multicast_ip)
        .def_readwrite("lidar_cmd_data_port", &PyLivoxLidarNetCfg::lidar_cmd_data_port)
        .def_readwrite("lidar_push_msg_port", &PyLivoxLidarNetCfg::lidar_push_msg_port)
        .def_readwrite("lidar_point_data_port", &PyLivoxLidarNetCfg::lidar_point_data_port)
        .def_readwrite("lidar_imu_data_port", &PyLivoxLidarNetCfg::lidar_imu_data_port)
        .def_readwrite("lidar_log_data_port", &PyLivoxLidarNetCfg::lidar_log_data_port)
        .def_readwrite("host_cmd_data_port", &PyLivoxLidarNetCfg::host_cmd_data_port)
        .def_readwrite("host_push_msg_port", &PyLivoxLidarNetCfg::host_push_msg_port)
        .def_readwrite("host_point_data_port", &PyLivoxLidarNetCfg::host_point_data_port)
        .def_readwrite("host_imu_data_port", &PyLivoxLidarNetCfg::host_imu_data_port)
        .def_readwrite("host_log_data_port", &PyLivoxLidarNetCfg::host_log_data_port);

    py::class_<PyLivoxLidarEthernetPacket>(m, "PyLivoxLidarEthernetPacket")
        .def(py::init<LivoxLidarEthernetPacket*>())
        .def_readwrite("time_interval", &PyLivoxLidarEthernetPacket::time_interval)
        .def_readwrite("dot_num", &PyLivoxLidarEthernetPacket::dot_num)
        .def_readwrite("ts", &PyLivoxLidarEthernetPacket::ts)
        .def("get_pts_data", &PyLivoxLidarEthernetPacket::get_pts_data);

    py::class_<PyLivoxLidarImuPacket>(m, "PyLivoxLidarImuPacket")
        .def(py::init<LivoxLidarEthernetPacket*>())
        .def_readwrite("ts", &PyLivoxLidarImuPacket::ts)
        .def_readwrite("gyr", &PyLivoxLidarImuPacket::gyr)
        .def_readwrite("acc", &PyLivoxLidarImuPacket::acc);

    m.def("LivoxLidarSdkInit", &LivoxLidarSdkInitWrapper);
    m.def("LivoxLidarSdkInitFromCfg", &LivoxLidarSdkInitFromCfgWrapper);
    m.def("LivoxLidarSdkUninit", &LivoxLidarSdkUninitWrapper);
    m.def("SetLivoxLidarPointCloudCallBack", &SetLivoxLidarPointCloudCallBackWrapper);
    m.def("SetLivoxLidarImuDataCallback", &SetLivoxLidarImuDataCallbackWrapper);
    m.def("SetLivoxLidarInfoCallback", &SetLivoxLidarInfoCallback);
    m.def("SetLivoxLidarInfoChangeCallback", &SetLivoxLidarInfoChangeCallback);
}
