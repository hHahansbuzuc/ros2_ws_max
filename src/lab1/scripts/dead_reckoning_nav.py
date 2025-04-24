#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseArray, Pose
from nav_msgs.msg import Odometry
import math, time
import csv
import os

class DeadReckoningNav(Node):
    def __init__(self):
        super().__init__('dead_reckoning_nav')
        self.pose = Pose()
        self.pub = self.create_publisher(Twist, '/cmd_vel_mux/cmd_vel', 10)
        self.create_subscription(PoseArray, '/goal_list', self.accion_mover_cb, 10)
        self.v_lin = 0.2
        self.v_ang = 1.0

        # listas para trayectoria
        self.real_data = []
        self.odom_data = []
        time.sleep(3)

    def cb_real(self, msg: Pose):
        # Guardamos última pose real
        self._last_real = (msg.position.x, msg.position.y)

    def cb_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        self._last_odom = (p.x, p.y)

    def aplicar_velocidad(self, cmds):
        for v, w, t in cmds:
            twist = Twist()
            twist.linear.x  = float(v)
            twist.angular.z = float(w)
            # usa time.time() puro
            start = time.time()
            end   = start + t
            while time.time() < end:
                self.pub.publish(twist)
                rclpy.spin_once(self, timeout_sec=0)
                time.sleep(0.02)
            self.pub.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0)
        # al terminar todos los cmds, detén el robot
        self.pub.publish(Twist())
        
    def normalize_angle(self, a): #normaliza el angulo entre -pi y pi para no rotar infinitamente
        return (a + math.pi) % (2*math.pi) - math.pi

    def mover_robot_a_destino(self, goal_pose):
        # El movimiento va a consistir en 3 pasos, primero apuntar, luego dirigirme y finalmente girarme
        x_obj, y_obj , theta_obj = goal_pose.position.x, goal_pose.position.y, goal_pose.orientation.z
        x, y, theta = self.pose.position.x, self.pose.position.y, self.pose.orientation.z

        # Mediante calculo geometrico obtengo el tiempo que me tarda llegar al x,y objetivo.
        dx = x_obj - x
        dy = y_obj - y
        d = math.sqrt(dx**2 + dy**2)
        # Ya que la trayectoria es recta y en cuadrados yo se que me debo girar siempre 90 grados a la izquierda al final de cada trayectoría
        
        # Calculo el tiempo que me tarda llegar al objetivo
        t_mover = d / self.v_lin
        # Me muevo ese tiempo para llegar al objetivo
        cmds = [(self.v_lin, 0, t_mover)]
        self.aplicar_velocidad(cmds)
        
        # Finalmente calculo el timepo para quedar en el angulo objetivo
        t_girar = self.normalize_angle(theta_obj-theta) / self.v_ang
        t_girar = t_girar*1.115 # 1.1 para compensar el tiempo que me tarda girar
        # Me muevo ese tiempo para quedar en el angulo objetivo
        cmds = [(0, self.v_ang, t_girar)]
        self.aplicar_velocidad(cmds)
        
        # Actualizo la pose del robot
        self.pose.position.x = x_obj
        self.pose.position.y = y_obj
        self.pose.orientation.z = theta_obj

    def accion_mover_cb(self, msg: PoseArray):
        for goal_pose in msg.poses:
            self.get_logger().info(f"Goal → x={goal_pose.position.x}, y={goal_pose.position.y}")
            self.mover_robot_a_destino(goal_pose)

def main():
    rclpy.init()
    node = DeadReckoningNav()
    try: rclpy.spin(node)
    finally:
        node.destroy_node(); rclpy.shutdown()

if __name__=='__main__':
    main()
