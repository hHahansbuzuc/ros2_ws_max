import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3
import numpy as np
from cv_bridge import CvBridge
import cv2




NODE_NAME:str = "obstacle_detector"



class Obstacle_detector(Node): 

    def __init__( self ):

        super().__init__(NODE_NAME)
        self.bridge = CvBridge()
        self.image_raw_subscription = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.image_callback,
            10
        )
        
        self.occupancy_state_publisher = self.create_publisher(
            Vector3,
            '/occupancy_state',
            10
        )
    
    def image_callback(self, msg: Image):
        # 1) convertir a arreglo (height, width)
        depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        depth_array = np.array(depth, copy=False).astype(np.float32)

        # 2) dimensiones y “tercios”
        h, w    = depth_array.shape
        third   = w // 3
        left    = depth_array[:,       :third]
        center  = depth_array[:, third:2*third]
        right   = depth_array[:, 2*third:]

        # 3) maskear ceros => NaN
        for region in (left, center, right):
            region[region == 0] = np.nan

        # 4) distancia mínima en cada región
        dist_left   = float(np.nanmin(left))
        dist_center = float(np.nanmin(center))
        dist_right  = float(np.nanmin(right))

        # 5) log y publicación
        self.get_logger().info(
            f'Distancias → Izq: {dist_left:.2f} m | '
            f'Cent: {dist_center:.2f} m | Der: {dist_right:.2f} m'
        )

        occ = Vector3()
        occ.x = dist_left
        occ.y = dist_center
        occ.z = dist_right
        self.occupancy_state_publisher.publish(occ)
        

if __name__ == '__main__':
    rclpy.init()
  
    detector_node = Obstacle_detector()
    rclpy.spin( detector_node )
    
    detector_node.destroy_node()
    
    rclpy.shutdown()

