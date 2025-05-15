'''
**************************************************************************

* @file         start.py
* @author       Wei Wang -> shaxikai@outlook.com
* @date         2025.4.23
* @version      V1.0.0"
* @brief        livox sdk python demo

"*************************************************************************
'''



import sys
import time
import numpy as np
import open3d as o3d

sys.path.append("./build_vs/python/")
import livox_sdk


class LivoxLidarWrapper:
    def __init__(self):
        self.pcd_buffer = []
        self.start_ts = None
        self.frame_id = 0

        # 配置 Livox Lidar 参数
        self.cfg = livox_sdk.PyLivoxLidarCfg()
        self.cfg.lidar_name = "MID360"
        self.cfg.lidar_ipaddr = "192.168.1.201"
        self.cfg.lidar_subnet_mask = "255.255.255.0"
        self.cfg.lidar_gateway = "192.168.1.1"

        self.cfg.host_ip = "192.168.1.50"
        self.cfg.multicast_ip = "224.1.1.5"

        self.cfg.lidar_cmd_data_port = 56100
        self.cfg.lidar_push_msg_port = 56200
        self.cfg.lidar_point_data_port = 56300
        self.cfg.lidar_imu_data_port = 56400
        self.cfg.lidar_log_data_port = 56500

        self.cfg.host_cmd_data_port = 56101
        self.cfg.host_push_msg_port = 56201
        self.cfg.host_point_data_port = 56301
        self.cfg.host_imu_data_port = 56401
        self.cfg.host_log_data_port = 56501

    def init_sdk(self):
        success = livox_sdk.LivoxLidarSdkInitFromCfg(self.cfg)
        if success:
            print("✅ Livox SDK 初始化成功")
        else:
            print("❌ Livox SDK 初始化失败")
        return success

    def pointcloud_callback(self, packet):
        raw = packet.get_pts_data()
        dtype = np.dtype([
            ('x', 'i4'),
            ('y', 'i4'),
            ('z', 'i4'),
            ('reflectivity', 'u1'),
            ('tag', 'u1'),
        ])
        points = np.frombuffer(raw, dtype=dtype)
        if len(points) == 0:
            return

        xyz_m = np.stack([points['x'], points['y'], points['z']], axis=-1) / 1000.0
        ts = packet.ts

        if self.start_ts is None:
            self.start_ts = ts

        self.pcd_buffer.append(xyz_m)

        if ts - self.start_ts >= 99_000_000:
            all_points = np.concatenate(self.pcd_buffer, axis=0)
            # pcd = o3d.geometry.PointCloud()
            # pcd.points = o3d.utility.Vector3dVector(all_points)
            # filename = f"data/frame_{self.frame_id}.pcd"
            # o3d.io.write_point_cloud(filename, pcd)
            # print(f"[保存] 写入 {filename}，包含 {len(all_points)} 个点")

            self.frame_id += 1
            self.start_ts = None
            self.pcd_buffer = []

    def imu_callback(self, imu_packet):
        # print("Timestamp:", imu_packet.ts)
        # print("Gyroscope:", imu_packet.gyr)
        # print("Accelerometer:", imu_packet.acc)
        return

    def run(self, duration=3):
        if not self.init_sdk():
            return

        # 注册回调函数
        livox_sdk.SetLivoxLidarPointCloudCallBack(self.pointcloud_callback)
        livox_sdk.SetLivoxLidarImuDataCallback(self.imu_callback)

        # 等待数据
        try:
            print("⏳ 接收中...")
            time.sleep(duration)
        finally:
            print("🛑 反初始化 Livox SDK")
            livox_sdk.LivoxLidarSdkUninit()


if __name__ == "__main__":
    lidar = LivoxLidarWrapper()
    lidar.run(duration=5)
