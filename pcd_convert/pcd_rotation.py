# rclpy (ROS 2のpythonクライアント)の機能を使えるようにします。
import rclpy
# rclpy (ROS 2のpythonクライアント)の機能のうちNodeを簡単に使えるようにします。こう書いていない場合、Nodeではなくrclpy.node.Nodeと書く必要があります。
from rclpy.node import Node
import std_msgs.msg as std_msgs
import sensor_msgs.msg as sensor_msgs
import numpy as np
import math
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
import time


# C++と同じく、Node型を継承します。
class PcdRotation(Node):
    # コンストラクタです、PcdRotationクラスのインスタンスを作成する際に呼び出されます。
    def __init__(self):
        # 継承元のクラスを初期化します。
        super().__init__('pcd_rotation_node')
        
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth = 10
        )
        
        qos_profile_sub = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth = 10
        )
        
        # Subscriptionを作成。
        self.subscription = self.create_subscription(sensor_msgs.PointCloud2, '/converted_pointcloud2', self.pcd_rotation, qos_profile) #set subscribe pcd topic name
        self.subscription  # 警告を回避するために設置されているだけです。削除しても挙動はかわりません。
        
        # Publisherを作成
        self.pcd_rotation_publisher = self.create_publisher(sensor_msgs.PointCloud2, 'pcd_rotation', qos_profile) #set publish pcd topic name
        
        #パラメータ
        # Set LiDAR position
        self.MID360_HIGHT = 980/1000; #hight position[m]
            
        #上下反転  LiDAR init
        self.THETA_INIT_X = 181 #[deg]
        self.THETA_INIT_Y = 2.5 #[deg]
        self.THETA_INIT_Z = 0 #[deg]
        
    def pointcloud2_to_array(self, cloud_msg):
        # Extract point cloud data
        points = np.frombuffer(cloud_msg.data, dtype=np.uint8).reshape(-1, cloud_msg.point_step)
        x = np.frombuffer(points[:, 0:4].tobytes(), dtype=np.float32)
        y = np.frombuffer(points[:, 4:8].tobytes(), dtype=np.float32)
        z = np.frombuffer(points[:, 8:12].tobytes(), dtype=np.float32)
        intensity = np.frombuffer(points[:, 12:16].tobytes(), dtype=np.float32)

        # Combine into a 4xN matrix
        point_cloud_matrix = np.vstack((x, y, z, intensity))
        #print(point_cloud_matrix)
        #print(f"point_cloud_matrix ={point_cloud_matrix.shape}")
        #print(f"x ={x.dtype, x.shape}")
        
        return point_cloud_matrix
        
    def pcd_rotation(self, msg):
        
        # Print stamp message
        t_stamp = msg.header.stamp
        #print(f"t_stamp ={t_stamp}")
        
        # Get pcd data
        points = self.pointcloud2_to_array(msg)
        #print(f"points ={points.shape}")
        # For pcd rotation
        xyz_point = np.vstack([points[0,:],points[1,:],points[2,:]])
        #print(f"xyz_point ={xyz_point.shape}")
        pointcloud, rot_matrix = rotation_xyz(xyz_point, self.THETA_INIT_X, self.THETA_INIT_Y, self.THETA_INIT_Z)
        # Add intensity
        pointcloud_intensity = np.insert(pointcloud, 3, points[3,:], axis=0)
        # Add mid height position
        pointcloud_intensity[2,:] += self.MID360_HIGHT
        #print(f"pointcloud_intensity ={pointcloud_intensity.shape}")
        
        # Publish for rviz2
        self.pcd_rotation = point_cloud_intensity_msg(pointcloud_intensity.T, t_stamp, 'map')
        self.pcd_rotation_publisher.publish(self.pcd_rotation )
        self.get_logger().info(f"Publish pcd_rotation : {t_stamp}")
        
def rotation_xyz(pointcloud, theta_x, theta_y, theta_z):
    rad_x = math.radians(theta_x)
    rad_y = math.radians(theta_y)
    rad_z = math.radians(theta_z)
    rot_x = np.array([[ 1,               0,                0],
                      [ 0, math.cos(rad_x), -math.sin(rad_x)],
                      [ 0, math.sin(rad_x),  math.cos(rad_x)]])
    
    rot_y = np.array([[ math.cos(rad_y), 0,  math.sin(rad_y)],
                      [               0, 1,                0],
                      [-math.sin(rad_y), 0,  math.cos(rad_y)]])
    
    rot_z = np.array([[ math.cos(rad_z), -math.sin(rad_z), 0],
                      [ math.sin(rad_z),  math.cos(rad_z), 0],
                      [               0,                0, 1]])
    rot_matrix = rot_z.dot(rot_y.dot(rot_x))
    #print(f"rot_matrix ={rot_matrix}")
    #print(f"pointcloud ={pointcloud.shape}")
    rot_pointcloud = rot_matrix.dot(pointcloud)
    return rot_pointcloud, rot_matrix

def point_cloud_intensity_msg(points, t_stamp, parent_frame):
    # In a PointCloud2 message, the point cloud is stored as an byte 
    # array. In order to unpack it, we also include some parameters 
    # which desribes the size of each individual point.
    ros_dtype = sensor_msgs.PointField.FLOAT32
    dtype = np.float32
    itemsize = np.dtype(dtype).itemsize # A 32-bit float takes 4 bytes.
    data = points.astype(dtype).tobytes() 

    # The fields specify what the bytes represents. The first 4 bytes 
    # represents the x-coordinate, the next 4 the y-coordinate, etc.
    fields = [
            sensor_msgs.PointField(name='x', offset=0, datatype=ros_dtype, count=1),
            sensor_msgs.PointField(name='y', offset=4, datatype=ros_dtype, count=1),
            sensor_msgs.PointField(name='z', offset=8, datatype=ros_dtype, count=1),
            sensor_msgs.PointField(name='intensity', offset=12, datatype=ros_dtype, count=1),
        ]

    # The PointCloud2 message also has a header which specifies which 
    # coordinate frame it is represented in. 
    header = std_msgs.Header(frame_id=parent_frame, stamp=t_stamp)
    

    return sensor_msgs.PointCloud2(
        header=header,
        height=1, 
        width=points.shape[0],
        is_dense=True,
        is_bigendian=False,
        fields=fields,
        point_step=(itemsize * 4), # Every point consists of three float32s.
        row_step=(itemsize * 4 * points.shape[0]), 
        data=data
    )

# mainという名前の関数です。C++のmain関数とは異なり、これは処理の開始地点ではありません。
def main(args=None):
    # rclpyの初期化処理です。ノードを立ち上げる前に実装する必要があります。
    rclpy.init(args=args)
    # クラスのインスタンスを作成
    pcd_rotation = PcdRotation()
    # spin処理を実行、spinをしていないとROS 2のノードはデータを入出力することが出来ません。
    rclpy.spin(pcd_rotation)
    # 明示的にノードの終了処理を行います。
    pcd_rotation.destroy_node()
    # rclpyの終了処理、これがないと適切にノードが破棄されないため様々な不具合が起こります。
    rclpy.shutdown()

# 本スクリプト(publish.py)の処理の開始地点です。
if __name__ == '__main__':
    # 関数`main`を実行する。
    main()
