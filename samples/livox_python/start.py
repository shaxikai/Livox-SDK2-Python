import sys
import time
import numpy as np

sys.path.append("./python/")
import livox_sdk

# # 初始化 SDK
# config_path = "/home/nhy/ww/robot/livox/Livox-SDK2/samples/livox_lidar_quick_start/mid360_config.json"
# success = livox_sdk.LivoxLidarSdkInit(config_path)


# 设置字段（参考你的配置 JSON 和 Livox 设备网络配置）
cfg = livox_sdk.PyLivoxLidarCfg()
cfg.lidar_name = "MID360"
cfg.lidar_ipaddr = "192.168.1.201"
cfg.lidar_subnet_mask = "255.255.255.0"
cfg.lidar_gateway = "192.168.1.1"

cfg.host_ip = "192.168.1.50"
cfg.multicast_ip = "224.1.1.5"

cfg.lidar_cmd_data_port = 56100
cfg.lidar_push_msg_port = 56200
cfg.lidar_point_data_port = 56300
cfg.lidar_imu_data_port = 56400
cfg.lidar_log_data_port = 56500

cfg.host_cmd_data_port = 56101
cfg.host_push_msg_port = 56201
cfg.host_point_data_port = 56301
cfg.host_imu_data_port = 56401
cfg.host_log_data_port = 56501

# 初始化 SDK
success = livox_sdk.LivoxLidarSdkInitFromCfg(cfg)

if success:
    print("Livox SDK 初始化成功")
else:
    print("Livox SDK 初始化失败")

# 缓存点云和起始时间
pcd_buffer = []
start_ts = None
frame_id = 0

def pointcloud_callback(packet):
    global pcd_buffer, start_ts, frame_id

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
    if start_ts is None:
        start_ts = ts

    pcd_buffer.append(xyz_m)

    if ts - start_ts >= 99_000_000:
        all_points = np.concatenate(pcd_buffer, axis=0)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(all_points)
        filename = f"data/frame.pcd"
        o3d.io.write_point_cloud(filename, pcd)
        print(f"[保存] 写入 {filename}，包含 {len(all_points)} 个点")

        frame_id += 1
        start_ts = None
        pcd_buffer = []


def imu_callback(imu_packet):
    print("Timestamp:", imu_packet.ts)
    print("Gyroscope:", imu_packet.gyr)
    print("Accelerometer:", imu_packet.acc)
    return 

# 注册回调
livox_sdk.SetLivoxLidarPointCloudCallBack(pointcloud_callback)
livox_sdk.SetLivoxLidarImuDataCallback(imu_callback)

# 等待数据
time.sleep(3)

# 反初始化
livox_sdk.LivoxLidarSdkUninit()
