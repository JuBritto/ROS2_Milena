#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class CameraProcessor(Node):
    def __init__(self):
        super().__init__('camera_processor')
        self.bridge = CvBridge()

        # Assine o tópico que o SDF declara
        topic_name = '/camera/image_raw'
        self.subscription = self.create_subscription(
            Image,
            topic_name,
            self.image_callback,
            10
        )

        self.publisher = self.create_publisher(Image, '/camera/image_edges', 10)

        # Cria janelas apenas se houver display disponível
        try:
            cv2.namedWindow('Imagem Original', cv2.WINDOW_NORMAL)
            cv2.namedWindow('Máscara Vermelha', cv2.WINDOW_NORMAL)
            cv2.namedWindow('Contornos em Azul', cv2.WINDOW_NORMAL)
            self.has_gui = True
        except Exception:
            self.has_gui = False
            self.get_logger().warn('Sem ambiente gráfico: desabilitei janelas OpenCV.')

        self.get_logger().info(f'Nó iniciado. Inscrito em {topic_name}')

    def image_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'Erro ao converter imagem: {e}')
            return

        # Se encoding for rgb/rgba -> converte para BGR (OpenCV)
        enc = msg.encoding.lower()
        if enc == 'rgb8':
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
        elif enc == 'rgba8':
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGBA2BGR)
        elif enc == 'bgra8':
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGRA2BGR)
        # se já for bgr8, passthrough já retorna ok

        # -- teste rápido: salva 1 frame para ver se a imagem chega
        cv2.imwrite('/tmp/camera_frame_debug.png', cv_image)

        # processamento: máscara vermelho
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        result = cv_image.copy()
        cv2.drawContours(result, contours, -1, (255, 0, 0), 2)

        if self.has_gui:
            cv2.imshow('Imagem Original', cv_image)
            cv2.imshow('Máscara Vermelha', mask)
            cv2.imshow('Contornos em Azul', result)
            cv2.waitKey(1)

        try:
            ros_image = self.bridge.cv2_to_imgmsg(result, encoding='bgr8')
            self.publisher.publish(ros_image)
        except Exception as e:
            self.get_logger().error(f'Erro ao publicar imagem: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = CameraProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if cv2.getWindowProperty('Imagem Original', 0) >= 0:
        cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
