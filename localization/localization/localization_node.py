import rclpy
import open3d as o3d
import numpy as np
from scipy.spatial.transform import Rotation as R
from time import time
import os
import utm

from rclpy.node import Node
import sensor_msgs_py.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, NavSatFix, Imu
from std_msgs.msg import Header, Float64
from nav_msgs.msg import Odometry
from message_filters import ApproximateTimeSynchronizer, Subscriber

from .optimize_global_map_pose import make_map_data


class LocalizationNode(Node):
    #####################################################################
    # region Initialization
    #####################################################################
    def __init__(self):
        super().__init__('localization_node')

        # Frame transforms
        self.map_T_global = np.eye(4)

        # Load the Map data to RAM memory, loading it if we have calculated before
        self.map_loaded = False
        self.map_folder = os.path.join(os.getenv("HOME"), "Desktop/map_data")
        self.map_name = "map.pcd"
        self.map_T_global_name = "map_T_global.npy"
        if (os.path.exists(os.path.join(self.map_folder, self.map_name))):
            self.get_logger().info('Loading map data from disk ...')
            self.map_original = o3d.io.read_point_cloud(
                os.path.join(self.map_folder, self.map_name))
            self.map_T_global = np.load(os.path.join(
                self.map_folder, self.map_T_global_name))
            self.get_logger().info('Map data loaded successfully!')
        else:
            self.get_logger().info('Making map data ...')
            self.map_original, self.map_T_global = make_map_data(
                self.map_folder, self.map_name)
            self.get_logger().info('Map data made successfully!')
        self.map_loaded = True
        self.map_t_global = self.map_T_global[:3, 3]
        self.map_R_global = self.map_T_global[:3, :3]

        # Parameters
        self.icp_conversion_threshold = 0.5  # [m]
        bbox_side = 11.0  # [m]
        self.min_boundaries = [-bbox_side/2, -bbox_side/2, -bbox_side/2]
        self.max_boundaries = [bbox_side/2, bbox_side/2, bbox_side/2]
        self.extent = np.array([bbox_side, bbox_side, bbox_side])
        self.current_compass = None

        # Create subscribers
        # Point cloud will be in current sensor frame, and odometry from current sensor to loc frames
        # Odometry will be used for model prediction between sequential sensor frames
        self.pointcloud_sub = Subscriber(
            self, PointCloud2, '/cloud_registered_body')
        self.odometry_sub = Subscriber(self, Odometry, '/Odometry')
        self.gps_sub = Subscriber(
            self, NavSatFix, '/mavros/global_position/global')
        self.sync = ApproximateTimeSynchronizer(
            [self.pointcloud_sub, self.odometry_sub, self.gps_sub],
            queue_size=1,
            slop=0.1
        )
        self.sync.registerCallback(self.syncCallback)
        self.compass_sub_ = self.create_subscription(
            Float64, '/mavros/global_position/compass_hdg', self.compassCallback, 1)

        # Publishers
        self.map_T_sensor_pub = self.create_publisher(
            Odometry, '/localization/map_T_sensor', 10)
        self.map_T_sensor_coarse_pub = self.create_publisher(
            Odometry, '/localization/map_T_sensor_gps', 10)
        self.map_pub = self.create_publisher(
            PointCloud2, '/localization/map', 10)
        self.cropped_scan_pub = self.create_publisher(
            PointCloud2, '/localization/cropped_scan_map_frame', 10)

        # Create a timer callback to publish the map at each 5 seconds
        self.create_timer(5.0, self.publishMapCallback)

        self.get_logger().info('Localization node started!')

    #####################################################################
    # endregion Initialization
    #####################################################################
    # region Conversions
    #####################################################################
    def readFilterPtcRegionPoints(self, ptc_msg: PointCloud2) -> np.ndarray:
        points = pc2.read_points(
            ptc_msg, field_names=("x", "y", "z"), skip_nans=True)
        x_min, x_max = self.min_boundaries[0], self.max_boundaries[0]
        y_min, y_max = self.min_boundaries[1], self.max_boundaries[1]
        z_min, z_max = self.min_boundaries[2], self.max_boundaries[2]
        points = np.array([[p[0], p[1], p[2]] for p in points])

        return points[(points[:, 0] >= x_min) & (points[:, 0] <= x_max) &
                      (points[:, 1] >= y_min) & (points[:, 1] <= y_max) &
                      (points[:, 2] >= z_min) & (points[:, 2] <= z_max)]

    def convertQuaternionTranslationToMatrix(self, q: np.ndarray, t: np.ndarray) -> np.ndarray:
        # Convert quaternion and translation to a 4x4 transformation matrix
        T = np.eye(4)
        r = R.from_quat(q).as_matrix()
        T[0:3, 0:3] = r
        T[:3, 3] = t

        return T

    def convertGpsCompassToMatrices(self, gps_msg: NavSatFix) -> list[np.ndarray, np.ndarray]:
        # Convert the compass yaw angle into rotation matrix
        compass_R = R.from_euler(
            'xyz', [0, 0, self.current_compass]).as_matrix()
        # Convert the GPS latitude, longitude and altitude to UTM coordinates
        utm_e, utm_n, _, _ = utm.from_latlon(
            gps_msg.latitude, gps_msg.longitude)
        t = np.array([utm_e, utm_n, gps_msg.altitude])

        return compass_R, t

    def buildNavOdomMsg(self, T: np.ndarray, frame_id: str, child_frame_id: str, stamp: float) -> Odometry:
        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = frame_id
        odom_msg.child_frame_id = child_frame_id
        odom_msg.pose.pose.position.x = T[0, 3]
        odom_msg.pose.pose.position.y = T[1, 3]
        odom_msg.pose.pose.position.z = T[2, 3]
        q = R.from_matrix(T[:3, :3]).as_quat()
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]

        return odom_msg

    def computeGpsCoarsePoseInMapFrame(self, gps_msg: NavSatFix) -> np.ndarray:
        # Calculate the coarse pose in global frame from GPS and compass class member from async callback
        global_R_sensor, global_t_sensor = self.convertGpsCompassToMatrices(
            gps_msg)
        # Calculate the coarse pose in map frame from the global pose
        # This is not the right methodology for working with 4x4 matrices, but it is fine for now
        map_T_sensor = np.eye(4)
        map_T_sensor[:3, :3] = self.map_R_global @ global_R_sensor
        map_T_sensor[:3, 3] = self.map_R_global @ (
            self.map_t_global + global_t_sensor)

        return map_T_sensor

    #####################################################################
    # endregion Conversions
    #####################################################################
    # region Callbacks
    #####################################################################
    def publishMapCallback(self):
        if not self.map_loaded:
            self.get_logger().warn('Map not loaded yet, not publishing ...')
            return
        # Publish the map point cloud
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'map'
        self.map_pub.publish(pc2.create_cloud_xyz32(
            header=header, points=np.asarray(self.map_original.points)))

    def compassCallback(self, compass_msg: Float64) -> None:
        self.current_compass = np.radians(90 - compass_msg.data)
        # Wrap to -PI to PI
        if self.current_compass > np.pi:
            self.current_compass -= 2*np.pi
        elif self.current_compass < -np.pi:
            self.current_compass += 2*np.pi

    def syncCallback(self, pointcloud_msg: PointCloud2, odometry_msg: Odometry, gps_msg: NavSatFix) -> None:
        if not self.map_loaded:
            self.get_logger().warn('Map not loaded yet, not localizing ...')
            return
        if not self.current_compass:
            self.get_logger().warn('Compass not received yet, not localizing ...')
            return
        self.get_logger().info('Localization callback called!')

        # Obtain the coarse pose from GPS and compass in the map frame, based on the global frame information
        map_T_sensor = self.computeGpsCoarsePoseInMapFrame(gps_msg)

        # Convert the PointCloud2 message to Open3D point cloud
        start = time()
        cropped_scan = o3d.geometry.PointCloud()
        cropped_scan.points = o3d.utility.Vector3dVector(
            self.readFilterPtcRegionPoints(ptc_msg=pointcloud_msg))
        # Transform the cropped point cloud region to the map frame
        cropped_scan.transform(map_T_sensor)

        # Crop the map region around the robot with a bounding box in the map frame
        map_bbox = o3d.geometry.OrientedBoundingBox(
            center=map_T_sensor[:3, 3], R=map_T_sensor[:3, :3],
            extent=self.extent)
        cropped_map = self.map_original.crop(map_bbox)
        if (len(cropped_map.points) == 0):
            self.get_logger().warn('Cropped map has no points, not localizing ...')
            return

        # Apply ICP to align the point clouds
        lidar_pose_adjustment = o3d.pipelines.registration.registration_icp(
            cropped_scan, cropped_map, self.icp_conversion_threshold, np.eye(
                4),
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=10))

        # Check if we had convergence in the lidar_pose_adjustment, and then apply fine tune to the map_T_sensor matrix
        # TODO: Implement the check to this lidar_pose_adjustment
        self.map_T_sensor = lidar_pose_adjustment.transformation @ map_T_sensor

        # Publish the fine class member map_T_sensor transformation as Odom message
        self.map_T_sensor_pub.publish(self.buildNavOdomMsg(
            self.map_T_sensor, 'map', 'sensor', odometry_msg.header.stamp))
        end = time()
        self.get_logger().info('Callback time: {}'.format(end-start))

        # Publish the coarse class memeber map_T_sensor transformation as Odom message
        self.map_T_sensor_coarse_pub.publish(self.buildNavOdomMsg(
            map_T_sensor, 'map', 'sensor', odometry_msg.header.stamp))
        # Publish the cropped scan in the map frame
        cropped_scan.transform(lidar_pose_adjustment.transformation)
        header = Header()
        header.stamp = odometry_msg.header.stamp
        header.frame_id = 'map'
        self.cropped_scan_pub.publish(pc2.create_cloud_xyz32(
            header=header, points=np.asarray(cropped_scan.points)))

    #####################################################################
    # endregion Callbacks
    #####################################################################


def main(args=None):
    rclpy.init(args=args)

    localization_node = LocalizationNode()

    rclpy.spin(localization_node)
    localization_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
