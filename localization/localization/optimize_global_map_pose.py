import numpy as np
import utm
import os
import open3d as o3d
from scipy.spatial.transform import Rotation as R


class MapBuilder:
    def __init__(self, map_folder: str) -> None:
        self.map_folder = map_folder
        self.odom_poses_file_path = os.path.join(
            map_folder, "odometry_positions.txt")
        self.gps_imu_data_file_path = os.path.join(
            map_folder, "gps_imu_poses.txt")
        self.max_num_poses_to_optimize = 50

        self.map_pcd = o3d.geometry.PointCloud()
        self.map_T_global = np.eye(4)
        self.max_translation_pose_transform = 0.50  # [m]

    def load_odom_positions(self) -> list[list, int]:
        odom_positions = np.loadtxt(fname=self.odom_poses_file_path, skiprows=1)
        # Count the poses with less than the threshold for translation
        count = 0
        for i in range(len(odom_positions)):
            t = np.linalg.norm(odom_positions[i])
            if t < self.max_translation_pose_transform:
                count += 1
            else:
                break

        return odom_positions, count

    def load_gps_imu_poses(self) -> np.ndarray:
        gps_imu_poses = np.loadtxt(
            fname=self.gps_imu_data_file_path, skiprows=1)
        # Convert the poses to 4x4 transformation matrices
        # The incoming code is lat lon alt r p y
        global_T_map_list = list()
        for pose in gps_imu_poses:
            # Convert quaternion to rotation matrix
            rot = R.from_euler("xyz", pose[3:7]).as_matrix()
            utm_x, utm_y, _, _ = utm.from_latlon(pose[0], pose[1])
            # Create the 4x4 transformation matrix
            T = np.eye(4)
            T[:3, :3] = rot
            T[:3, 3] = np.asarray([utm_x, utm_y, pose[2]])
            global_T_map_list.append(T)

        return global_T_map_list

    def load_gps_imu_poses_inverse(self) -> np.ndarray:
        gps_imu_poses = np.loadtxt(
            fname=self.gps_imu_data_file_path, skiprows=1)
        # Convert the poses to 4x4 transformation matrices
        # The incoming code is lat lon alt r p y
        map_T_global_list = list()
        for pose in gps_imu_poses:
            # Convert quaternion to rotation matrix
            rot = R.from_euler("xyz", pose[3:7]).as_matrix()
            utm_x, utm_y, _, _ = utm.from_latlon(pose[0], pose[1])
            # Create the 4x4 transformation matrix
            T = np.eye(4)
            T[:3, :3] = rot.T
            T[:3, 3] = -np.asarray([utm_x, utm_y, pose[2]])
            map_T_global_list.append(T)

        return map_T_global_list

    def create_save_map(self, map_pcd_name: str) -> bool:
        # Search for all pcd files in the map folder
        pcd_files = [f for f in os.listdir(
            self.map_folder) if f.endswith(".pcd")]
        if len(pcd_files) == 0:
            print("No pcd files found in the map folder!")
            return False

        # Aggregate all of them into a single point cloud
        for pcd_file in pcd_files:
            pcd = o3d.io.read_point_cloud(
                os.path.join(self.map_folder, pcd_file))
            self.map_pcd += pcd
        # Save as a single binary pcd file
        map_pcd_path = os.path.join(self.map_folder, map_pcd_name)
        o3d.io.write_point_cloud(
            filename=map_pcd_path, pointcloud=self.map_pcd, format='pcd', write_ascii=False)

        return True

    def optimize_map_T_global(self) -> np.ndarray:
        # Load the odometry poses
        _, n_valid_poses = self.load_odom_positions()
        # Load the GPS/IMU poses
        map_T_global_list = self.load_gps_imu_poses_inverse()
        # Get the relative transformations from the global to map frames
        n_poses = min(n_valid_poses, len(map_T_global_list),
                      self.max_num_poses_to_optimize)
        map_T_global_list = map_T_global_list[:n_poses]
        # Get the mean translation
        map_t_global = np.mean([T[:3, 3] for T in map_T_global_list], axis=0)
        # Get the RPY angles for each relative transformation
        rpy_angles = [R.from_matrix(T[:3, :3]).as_euler('xyz')
                      for T in map_T_global_list]
        # Get the mean RPY angles
        mean_rpy = np.mean(rpy_angles, axis=0)
        map_R_global = R.from_euler('xyz', mean_rpy).as_matrix()
        # Create the best transformation matrix
        self.map_T_global = np.eye(4)
        self.map_T_global[:3, :3] = map_R_global
        self.map_T_global[:3, 3] = map_t_global

        return self.map_T_global

    def get_map(self) -> o3d.geometry.PointCloud:
        return self.map_pcd

    def get_map_T_global(self) -> np.ndarray:
        return self.map_T_global


def make_map_data(map_folder: str, map_name: str) -> list[o3d.geometry.PointCloud, np.ndarray]:
    map_builder = MapBuilder(
        map_folder=map_folder,
    )
    # Create and save the map as a single pcd file
    map_pcd_name = os.path.join(map_folder, map_name)
    if not map_builder.create_save_map(map_pcd_name=map_pcd_name):
        print("Failed to save map!")
        return o3d.geometry.PointCloud(), np.eye(4)

    # Compute the global transformation matrix from global to map frame
    map_T_global = map_builder.optimize_map_T_global()
    transformation_file_name = os.path.join(map_folder, "map_T_global.npy")
    np.save(transformation_file_name, map_T_global)

    return map_builder.get_map(), map_builder.get_map_T_global()


if __name__ == "__main__":
    map_folder = os.path.join(os.getenv("HOME"), "Desktop/map_data")
    map_name = "map.pcd"
    _, _ = make_map_data(map_folder=map_folder, map_name=map_name)
