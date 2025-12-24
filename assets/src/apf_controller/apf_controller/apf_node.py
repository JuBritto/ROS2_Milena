import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import numpy as np
import math

class APFLidar(Node):
    def __init__(self):
        super().__init__("apf_lidar_node")

        # Parâmetros do objetivo
        self.declare_parameter("goal_x", 10.0)
        self.declare_parameter("goal_y", 0.0)

        self.goal = np.array([
            self.get_parameter("goal_x").value,
            self.get_parameter("goal_y").value
        ])

        self.get_logger().info(f"Objetivo em {self.goal}")

        # Posição inicial do robô
        self.robot_pos = np.array([-3.0, 0.0])
        self.robot_yaw = 0.0

        # Subscrições e publicações ROS2
        self.create_subscription(LaserScan, "/scan", self.scan_callback, 10)
        self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # Ganhos do Campo Potencial Artificial (APF)
        self.k_att = 1.1         # Ganho de atração
        self.k_rep_front = 2.8   # Ganho repulsivo frontal
        self.k_rep_side = 4.8    # Ganho repulsivo lateral
        self.k_rep_back = 1.8    # Ganho repulsivo traseiro (menor para evitar travamento)
        self.rep_range = 2.3     # Alcance do campo repulsivo

    # Callback para odometria
    def odom_callback(self, msg):
        # Atualiza a posição do robô
        self.robot_pos = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        ])

        # Calcula o yaw (orientação) a partir do quaternion
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny, cosy)

    # Callback para dados do LiDAR e cálculo do APF
    def scan_callback(self, msg: LaserScan):
        # Converte os dados do scan em arrays
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))

        # Calcula a força atrativa
        dir_goal = self.goal - self.robot_pos
        dist_goal = np.linalg.norm(dir_goal)

        # Verifica se o objetivo foi alcançado
        if dist_goal < 0.3:
            self.get_logger().info("Objetivo alcançado!")
            self.cmd_pub.publish(Twist())
            return

        # Normaliza a direção para o objetivo
        dir_goal_norm = dir_goal / dist_goal
        F_att = self.k_att * dir_goal_norm

        # Calcula a força repulsiva dividida em setores
        F_rep = np.array([0.0, 0.0])

        # Definição dos setores angulares (em radianos)
        FRONT_MIN = -math.radians(50)
        FRONT_MAX = math.radians(50)
        LEFT_MIN = math.radians(50)
        LEFT_MAX = math.radians(110)
        RIGHT_MIN = -math.radians(110)
        RIGHT_MAX = -math.radians(50)

        # Itera sobre cada leitura do LiDAR
        for r, ang in zip(ranges, angles):
            # Ignora leituras infinitas ou fora do alcance repulsivo
            if np.isinf(r) or r > self.rep_range:
                continue

            # Calcula a magnitude repulsiva
            rep_mag = (1.0 / r - 1.0 / self.rep_range) ** 2

            # Determina o ganho baseado no setor
            if FRONT_MIN <= ang <= FRONT_MAX:
                rep_gain = self.k_rep_front
            elif LEFT_MIN <= ang <= LEFT_MAX:
                rep_gain = self.k_rep_side
            elif RIGHT_MIN <= ang <= RIGHT_MAX:
                rep_gain = self.k_rep_side
            else:
                continue  # Ignora setores não definidos

            # Calcula a direção repulsiva ajustada ao yaw do robô
            dx = -np.cos(self.robot_yaw + ang)
            dy = -np.sin(self.robot_yaw + ang)

            # Soma à força repulsiva total
            F_rep += rep_gain * rep_mag * np.array([dx, dy])

        # Calcula a força total (atrativa + repulsiva)
        F_total = F_att + F_rep
        norm = np.linalg.norm(F_total)
        if norm > 0:
            F_total /= norm  # Normaliza a força total

        # Converte a força total em componentes forward e lateral
        Fx, Fy = F_total
        F_forward = Fx * math.cos(self.robot_yaw) + Fy * math.sin(self.robot_yaw)
        F_lateral = -Fx * math.sin(self.robot_yaw) + Fy * math.cos(self.robot_yaw)

        # Cria e configura o comando de velocidade
        cmd = Twist()
        cmd.linear.x = 0.80 * F_forward
        cmd.angular.z = 1.35 * F_lateral

        # Limita as velocidades para segurança
        cmd.linear.x = float(np.clip(cmd.linear.x, -0.2, 0.6))
        cmd.angular.z = float(np.clip(cmd.angular.z, -2.0, 2.0))

        # Publica o comando
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = APFLidar()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()