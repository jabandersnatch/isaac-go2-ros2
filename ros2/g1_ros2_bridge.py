import subprocess
import time

import cv2
import numpy as np
import omni
import omni.graph.core as og
import omni.replicator.core as rep
import omni.syntheticdata._syntheticdata as sd
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from tf2_ros import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

import g1.g1_ctrl as g1_ctrl

ext_manager = omni.kit.app.get_app().get_extension_manager()
ext_manager.set_extension_enabled_immediate("omni.isaac.ros2_bridge", True)
from isaacsim.ros2.bridge import read_camera_info


class RobotDataManager(Node):
    def __init__(self, env, lidar_annotators, cameras, cfg):
        super().__init__("robot_data_manager")
        self.cfg = cfg
        self.env = env
        self.num_envs = env.unwrapped.scene.num_envs
        self.lidar_annotators = lidar_annotators
        self.cameras = cameras
        
        # Initialize ROS2 time
        self._setup_ros_time()
        
        # Create publishers and subscribers
        self._create_publishers_subscribers()
        
        # Setup timing
        self.odom_pose_freq = 50.0
        self.joint_state_freq = 10.0
        self.lidar_freq = 15.0
        self._reset_timers()
        
        # Create static transforms and camera publishers
        self._create_static_transforms()
        self._create_camera_publishers()

    def _setup_ros_time(self):
        """Setup ROS simulation time."""
        self._create_ros_time_graph()
        sim_time_set = False
        while rclpy.ok() and not sim_time_set:
            sim_time_set = self._use_sim_time()

    def _create_ros_time_graph(self):
        """Create omnigraph for ROS time synchronization."""
        og.Controller.edit(
            {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                    ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("PublishClock.inputs:topicName", "/clock"),
                ],
            },
        )

    def _use_sim_time(self):
        """Set ROS parameter to use simulation time."""
        command = ["ros2", "param", "set", "/robot_data_manager", "use_sim_time", "true"]
        subprocess.Popen(command)
        return True

    def _get_topic_name(self, base_topic, env_idx=0):
        """Generate topic name based on number of environments."""
        if self.num_envs == 1:
            return f"unitree_g1/{base_topic}"
        else:
            return f"unitree_g1_{env_idx}/{base_topic}"

    def _get_frame_id(self, frame_name, env_idx=0):
        """Generate frame ID based on number of environments."""
        if self.num_envs == 1:
            return f"unitree_g1/{frame_name}"
        else:
            return f"unitree_g1_{env_idx}/{frame_name}"

    def _create_publishers_subscribers(self):
        """Create all ROS publishers and subscribers."""
        self.broadcaster = TransformBroadcaster(self)
        
        # Initialize lists
        self.odom_pub = []
        self.pose_pub = []
        self.joint_state_pub = []
        self.lidar_pub = []
        self.semantic_seg_img_vis_pub = []
        self.cmd_vel_sub = []
        self.semantic_seg_img_sub = []

        for i in range(self.num_envs):
            # Publishers
            self.odom_pub.append(
                self.create_publisher(Odometry, self._get_topic_name("odom", i), 10)
            )
            self.pose_pub.append(
                self.create_publisher(PoseStamped, self._get_topic_name("pose", i), 10)
            )
            self.joint_state_pub.append(
                self.create_publisher(JointState, self._get_topic_name("joint_states", i), 10)
            )
            self.lidar_pub.append(
                self.create_publisher(PointCloud2, self._get_topic_name("lidar/point_cloud", i), 10)
            )
            self.semantic_seg_img_vis_pub.append(
                self.create_publisher(
                    Image, self._get_topic_name("front_cam/semantic_segmentation_image_vis", i), 10
                )
            )

            # Subscribers
            self.cmd_vel_sub.append(
                self.create_subscription(
                    Twist,
                    self._get_topic_name("cmd_vel", i),
                    lambda msg, env_idx=i: self._cmd_vel_callback(msg, env_idx),
                    10,
                )
            )
            self.semantic_seg_img_sub.append(
                self.create_subscription(
                    Image,
                    "/" + self._get_topic_name("front_cam/semantic_segmentation_image", i),
                    lambda msg, env_idx=i: self._semantic_segmentation_callback(msg, env_idx),
                    10,
                )
            )

    def _reset_timers(self):
        """Reset all timing variables."""
        current_time = time.time()
        self.odom_pose_pub_time = current_time
        self.joint_state_pub_time = current_time
        self.lidar_pub_time = current_time

    def _create_static_transforms(self):
        """Create static TF transforms for sensors."""
        for i in range(self.num_envs):
            # LiDAR transform
            self._create_lidar_transform(i)
            # Camera transform
            self._create_camera_transform(i)

    def _create_lidar_transform(self, env_idx):
        """Create static transform for LiDAR sensor."""
        broadcaster = StaticTransformBroadcaster(self)
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self._get_frame_id("base_link", env_idx)
        transform.child_frame_id = self._get_frame_id("lidar_frame", env_idx)

        # LiDAR mounted on torso
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.3
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0

        broadcaster.sendTransform(transform)

    def _create_camera_transform(self, env_idx):
        """Create static transform for camera sensor."""
        broadcaster = StaticTransformBroadcaster(self)
        transform = TransformStamped()
        transform.header.frame_id = self._get_frame_id("base_link", env_idx)
        transform.child_frame_id = self._get_frame_id("front_cam", env_idx)

        # Camera mounted on head/torso
        transform.transform.translation.x = 0.2
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.4
        transform.transform.rotation.x = -0.5
        transform.transform.rotation.y = 0.5
        transform.transform.rotation.z = -0.5
        transform.transform.rotation.w = 0.5

        broadcaster.sendTransform(transform)

    def _create_camera_publishers(self):
        """Create camera data publishers if enabled."""
        if not self.cfg.sensor.enable_camera:
            return
            
        if self.cfg.sensor.color_image:
            self._setup_color_image_publisher()
        if self.cfg.sensor.depth_image:
            self._setup_depth_image_publisher()
        if self.cfg.sensor.semantic_segmentation:
            self._setup_semantic_image_publisher()
        self._setup_camera_info_publisher()

    def _setup_image_publisher(self, sensor_type, topic_suffix):
        """Generic method to setup image publishers."""
        for i in range(self.num_envs):
            render_product = self.cameras[i]._render_product_path
            topic_name = self._get_topic_name(f"front_cam/{topic_suffix}", i)
            frame_id = self._get_frame_id("front_cam", i)

            rv = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(sensor_type)
            writer = rep.writers.get(rv + "ROS2PublishImage")
            writer.initialize(
                frameId=frame_id,
                nodeNamespace="",
                queueSize=1,
                topicName=topic_name,
            )
            writer.attach([render_product])

            gate_path = omni.syntheticdata.SyntheticData._get_node_path(
                rv + "IsaacSimulationGate", render_product
            )
            og.Controller.attribute(gate_path + ".inputs:step").set(1)

    def _setup_color_image_publisher(self):
        """Setup RGB image publisher."""
        self._setup_image_publisher(sd.SensorType.Rgb.name, "color_image")

    def _setup_depth_image_publisher(self):
        """Setup depth image publisher."""
        self._setup_image_publisher(sd.SensorType.DistanceToImagePlane.name, "depth_image")

    def _setup_semantic_image_publisher(self):
        """Setup semantic segmentation image publisher."""
        for i in range(self.num_envs):
            render_product = self.cameras[i]._render_product_path
            topic_name = self._get_topic_name("front_cam/semantic_segmentation_image", i)
            frame_id = self._get_frame_id("front_cam", i)

            rv = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(
                sd.SensorType.SemanticSegmentation.name
            )
            writer = rep.writers.get("ROS2PublishSemanticSegmentation")
            writer.initialize(
                frameId=frame_id,
                nodeNamespace="",
                queueSize=1,
                topicName=topic_name,
            )
            writer.attach([render_product])

            gate_path = omni.syntheticdata.SyntheticData._get_node_path(
                rv + "IsaacSimulationGate", render_product
            )
            og.Controller.attribute(gate_path + ".inputs:step").set(1)

    def _setup_camera_info_publisher(self):
        """Setup camera info publisher."""
        for i in range(self.num_envs):
            render_product = self.cameras[i]._render_product_path
            topic_name = self._get_topic_name("front_cam/info", i)
            frame_id = self.cameras[i].prim_path.split("/")[-1]

            camera_info = read_camera_info(render_product_path=render_product)
            writer = rep.writers.get("ROS2PublishCameraInfo")
            writer.initialize(
                frameId=frame_id,
                nodeNamespace="",
                queueSize=1,
                topicName=topic_name,
                width=camera_info["width"],
                height=camera_info["height"],
                projectionType=camera_info["projectionType"],
                k=camera_info["k"].reshape([1, 9]),
                r=camera_info["r"].reshape([1, 9]),
                p=camera_info["p"].reshape([1, 12]),
                physicalDistortionModel=camera_info["physicalDistortionModel"],
                physicalDistortionCoefficients=camera_info["physicalDistortionCoefficients"],
            )
            writer.attach([render_product])

            gate_path = omni.syntheticdata.SyntheticData._get_node_path(
                "PostProcessDispatch" + "IsaacSimulationGate", render_product
            )
            og.Controller.attribute(gate_path + ".inputs:step").set(1)

    def _publish_odometry(self, base_pos, base_rot, base_lin_vel_b, base_ang_vel_b, env_idx):
        """Publish odometry message."""
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "map"
        odom_msg.child_frame_id = self._get_frame_id("base_link", env_idx)
        
        # Position and orientation
        odom_msg.pose.pose.position.x = base_pos[0].item()
        odom_msg.pose.pose.position.y = base_pos[1].item()
        odom_msg.pose.pose.position.z = base_pos[2].item()
        odom_msg.pose.pose.orientation.x = base_rot[1].item()
        odom_msg.pose.pose.orientation.y = base_rot[2].item()
        odom_msg.pose.pose.orientation.z = base_rot[3].item()
        odom_msg.pose.pose.orientation.w = base_rot[0].item()
        
        # Velocity
        odom_msg.twist.twist.linear.x = base_lin_vel_b[0].item()
        odom_msg.twist.twist.linear.y = base_lin_vel_b[1].item()
        odom_msg.twist.twist.linear.z = base_lin_vel_b[2].item()
        odom_msg.twist.twist.angular.x = base_ang_vel_b[0].item()
        odom_msg.twist.twist.angular.y = base_ang_vel_b[1].item()
        odom_msg.twist.twist.angular.z = base_ang_vel_b[2].item()
        
        self.odom_pub[env_idx].publish(odom_msg)

        # Publish transform
        self._publish_base_transform(base_pos, base_rot, env_idx)

    def _publish_base_transform(self, base_pos, base_rot, env_idx):
        """Publish base link transform."""
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "map"
        transform.child_frame_id = self._get_frame_id("base_link", env_idx)
        
        transform.transform.translation.x = base_pos[0].item()
        transform.transform.translation.y = base_pos[1].item()
        transform.transform.translation.z = base_pos[2].item()
        transform.transform.rotation.x = base_rot[1].item()
        transform.transform.rotation.y = base_rot[2].item()
        transform.transform.rotation.z = base_rot[3].item()
        transform.transform.rotation.w = base_rot[0].item()
        
        self.broadcaster.sendTransform(transform)

    def _publish_pose(self, base_pos, base_rot, env_idx):
        """Publish pose message."""
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"
        
        pose_msg.pose.position.x = base_pos[0].item()
        pose_msg.pose.position.y = base_pos[1].item()
        pose_msg.pose.position.z = base_pos[2].item()
        pose_msg.pose.orientation.x = base_rot[1].item()
        pose_msg.pose.orientation.y = base_rot[2].item()
        pose_msg.pose.orientation.z = base_rot[3].item()
        pose_msg.pose.orientation.w = base_rot[0].item()
        
        self.pose_pub[env_idx].publish(pose_msg)

    def _publish_joint_states(self, joint_pos, joint_vel, env_idx):
        """Publish joint states for humanoid robot."""
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()

        # Get joint names from robot
        joint_names = self.env.unwrapped.scene["unitree_g1"].data.joint_names
        joint_state_msg.name = joint_names
        joint_state_msg.position = joint_pos.cpu().numpy().tolist()
        joint_state_msg.velocity = joint_vel.cpu().numpy().tolist()
        joint_state_msg.effort = [0.0] * len(joint_names)  # Placeholder

        self.joint_state_pub[env_idx].publish(joint_state_msg)

    def _publish_lidar_data(self, points, env_idx):
        """Publish LiDAR point cloud data."""
        point_cloud = PointCloud2()
        point_cloud.header.frame_id = self._get_frame_id("lidar_frame", env_idx)
        point_cloud.header.stamp = self.get_clock().now().to_msg()
        
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        point_cloud = point_cloud2.create_cloud(point_cloud.header, fields, points)
        self.lidar_pub[env_idx].publish(point_cloud)

    def pub_ros2_data(self):
        """Main method to publish ROS2 data at appropriate frequencies."""
        current_time = time.time()
        
        # Check timing for different data types
        pub_odom_pose = (current_time - self.odom_pose_pub_time) >= (1.0 / self.odom_pose_freq)
        pub_joint_state = (current_time - self.joint_state_pub_time) >= (1.0 / self.joint_state_freq)
        pub_lidar = (current_time - self.lidar_pub_time) >= (1.0 / self.lidar_freq)

        robot_data = self.env.unwrapped.scene["unitree_g1"].data

        # Publish odometry and pose data
        if pub_odom_pose:
            self.odom_pose_pub_time = current_time
            for i in range(self.num_envs):
                self._publish_odometry(
                    robot_data.root_state_w[i, :3],
                    robot_data.root_state_w[i, 3:7],
                    robot_data.root_lin_vel_b[i],
                    robot_data.root_ang_vel_b[i],
                    i,
                )
                self._publish_pose(
                    robot_data.root_state_w[i, :3],
                    robot_data.root_state_w[i, 3:7],
                    i,
                )

        # Publish joint states
        if pub_joint_state:
            self.joint_state_pub_time = current_time
            for i in range(self.num_envs):
                self._publish_joint_states(robot_data.joint_pos[i], robot_data.joint_vel[i], i)

        # Publish LiDAR data
        if self.cfg.sensor.enable_lidar and len(self.lidar_annotators) > 0 and pub_lidar:
            self.lidar_pub_time = current_time
            for i in range(self.num_envs):
                self._publish_lidar_data(
                    self.lidar_annotators[i].get_data()["data"].reshape(-1, 3), i
                )

    def _cmd_vel_callback(self, msg, env_idx):
        """Handle velocity command messages."""
        g1_ctrl.base_vel_cmd_input[env_idx][0] = msg.linear.x
        g1_ctrl.base_vel_cmd_input[env_idx][1] = msg.linear.y
        g1_ctrl.base_vel_cmd_input[env_idx][2] = msg.angular.z

    def _semantic_segmentation_callback(self, img, env_idx):
        """Handle semantic segmentation visualization."""
        bridge = CvBridge()
        semantic_image = bridge.imgmsg_to_cv2(img, desired_encoding="passthrough")
        
        # Handle division by zero case
        max_val = semantic_image.max()
        if max_val > 0:
            semantic_image_normalized = (semantic_image / max_val * 255).astype(np.uint8)
        else:
            semantic_image_normalized = np.zeros_like(semantic_image, dtype=np.uint8)

        # Apply colormap for visualization
        color_mapped_image = cv2.applyColorMap(semantic_image_normalized, cv2.COLORMAP_JET)
        image_msg = bridge.cv2_to_imgmsg(color_mapped_image, encoding="rgb8")
        self.semantic_seg_img_vis_pub[env_idx].publish(image_msg)