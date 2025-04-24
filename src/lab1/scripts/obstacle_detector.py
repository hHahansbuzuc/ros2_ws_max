#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3
from cv_bridge import CvBridge
import numpy as np

class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')
        self.bridge = CvBridge()
        self.declare_parameter('distance_threshold', 0.5)
        self.threshold = self.get_parameter('distance_threshold').value
        self.create_subscription(Image, '/camera/depth/image_raw',
                                 self.cb_image, 10)
        self.pub = self.create_publisher(Vector3, '/occupancy_state', 10)

    def cb_image(self, msg):
        depth = self.bridge.imgmsg_to_cv2(msg, '32FC1')
        thirds = np.hsplit(depth, 3)
        state = [0,0,0]
        for i, region in enumerate(thirds):
            mask = np.isfinite(region)
            if np.any((region[mask]>0)&(region[mask]<=self.threshold)):
                state[i]=1
        vec = Vector3(x=state[0], y=state[1], z=state[2])
        self.pub.publish(vec)

def main():
    rclpy.init()
    node = ObstacleDetector()
    try: rclpy.spin(node)
    finally:
        node.destroy_node(); rclpy.shutdown()

if __name__=='__main__':
    main()
