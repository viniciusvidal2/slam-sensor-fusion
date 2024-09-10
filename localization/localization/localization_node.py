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

    def __init__(self):
        super().__init__('localization_node')

        # Frame transforms
        self.map_T_global = np.eye(4)

        # Load the Map data to RAM memory, loading it if we have calculated before
        self.map_loaded = False
        self.map_folder = "/home/vini/Desktop/map_data"
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
        print("map_T_global")
        print(self.map_T_global)

        # Parameters
        self.icp_conversion_threshold = 0.05  # [m]
        self.bbox_side = 20.0  # [m]
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
            queue_size=10,
            slop=0.1
        )
        self.sync.registerCallback(self.syncCallback)
        self.compass_sub_ = self.create_subscription(
            Float64, '/mavros/global_position/compass_hdg', self.compassCallback, 10)

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

    def convertQuaternionTranslationToMatrix(self, q: np.ndarray, t: np.ndarray) -> np.ndarray:
        # Convert quaternion and translation to a 4x4 transformation matrix
        T = np.eye(4)
        r = R.from_quat(q).as_matrix()
        T[0:3, 0:3] = r
        T[:3, 3] = t

        return T

    def convertGpsCompassToMatrix(self, gps_msg: NavSatFix) -> np.ndarray:
        # Convert the GPS and compass quaternion to a 4x4 transformation matrix
        q = R.from_euler('z', self.current_compass).as_quat()
        utm_x, utm_y, _, _ = utm.from_latlon(
            gps_msg.latitude, gps_msg.longitude)
        t = np.array([utm_x, utm_y, gps_msg.altitude])
        print("t")
        print(t)
        T = self.convertQuaternionTranslationToMatrix(q, t)

        return T

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

    def compassCallback(self, compass_msg: Float64):
        self.current_compass = np.radians(compass_msg.data - 90.0)
        # Wrap to -PI to PI
        if self.current_compass > np.pi:
            self.current_compass -= 2*np.pi

    def syncCallback(self, pointcloud_msg: PointCloud2, odometry_msg: Odometry, gps_msg: NavSatFix):
        if not self.map_loaded:
            self.get_logger().warn('Map not loaded yet, not localizing ...')
            return
        if not self.current_compass:
            self.get_logger().warn('Compass not received yet, not localizing ...')
            return
        self.get_logger().info('Localization callback called!')

        # Convert the PointCloud2 message to Open3D point cloud
        start = time()
        original_points = pc2.read_points(
            pointcloud_msg, field_names=("x", "y", "z", "intensity"), skip_nans=True)
        points = np.array([[p[0], p[1], p[2]] for p in original_points])
        input_scan = o3d.geometry.PointCloud()
        input_scan.points = o3d.utility.Vector3dVector(points)
        # Crop the incoming point cloud around the robot with a bounding box
        bbox_scan = o3d.geometry.AxisAlignedBoundingBox(
            min_bound=(-self.bbox_side, -self.bbox_side, -self.bbox_side),
            max_bound=(self.bbox_side, self.bbox_side, self.bbox_side))
        cropped_scan = input_scan.crop(bbox_scan)

        # Calculate the coarse pose in global frame from GPS and compass class member from async callback
        global_T_sensor = self.convertGpsCompassToMatrix(gps_msg)

        # Calculate the coarse pose in map frame from the global pose
        map_T_sensor = self.map_T_global @ global_T_sensor
        print("-------------------")
        print("map_T_sensor")
        print(map_T_sensor)
        print("map_T_global")
        print(self.map_T_global)
        print("global_T_sensor")
        print(global_T_sensor)
        map_R_sensor = map_T_sensor[:3, :3]
        map_t_sensor = map_T_sensor[:3, 3]
        print("map_t_sensor")
        print(map_t_sensor)

        # Transform the cropped point cloud region to the sensor frame
        cropped_scan.transform(map_T_sensor)

        # Crop the map around the robot with a bounding box in the loc frame
        map_bbox = o3d.geometry.OrientedBoundingBox(
            center=map_t_sensor, R=map_R_sensor,
            extent=np.array([self.bbox_side, self.bbox_side, self.bbox_side]))
        cropped_map = self.map_original.crop(map_bbox)
        if (len(cropped_map.points) == 0):
            self.get_logger().warn('Cropped map has no points, not localizing ...')
            return

        # Apply ICP to align the point clouds
        fine_registration_adjustment = o3d.pipelines.registration.registration_icp(
            cropped_scan, cropped_map, self.icp_conversion_threshold, np.eye(
                4),
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=30))

        # Check if we had convergence in the fine_registration_adjustment, and then apply fine tune to the map_T_sensor matrix
        # TODO: Implement the check to this fine_registration_adjustment
        self.map_T_sensor = fine_registration_adjustment.transformation @ map_T_sensor

        # Publish the coarse class memeber map_T_sensor transformation as Odom message
        self.map_T_sensor_coarse_pub.publish(self.buildNavOdomMsg(
            map_T_sensor, 'map', 'sensor', odometry_msg.header.stamp))
        # Publish the fine class memeber map_T_sensor transformation as Odom message
        self.map_T_sensor_pub.publish(self.buildNavOdomMsg(
            self.map_T_sensor, 'map', 'sensor', odometry_msg.header.stamp))
        
        # Publish the cropped scan in the map frame
        # cropped_scan.transform(fine_registration_adjustment.transformation)
        header = Header()
        header.stamp = odometry_msg.header.stamp
        header.frame_id = 'map'
        self.cropped_scan_pub.publish(pc2.create_cloud_xyz32(
            header=header, points=np.asarray(cropped_scan.points)))

        end = time()
        self.get_logger().info('Callback time: {}'.format(end-start))


def main(args=None):
    rclpy.init(args=args)

    localization_node = LocalizationNode()

    rclpy.spin(localization_node)
    localization_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
