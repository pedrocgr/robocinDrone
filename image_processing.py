#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
import numpy as np
from cv_bridge import CvBridge

class ImageProcessingNode(Node):
    def __init__(self):
        super().__init__('image_processing_node')
        self.bridge = CvBridge()
        #fazendo subscriber para o topico da camera do imageProcessingNode
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.color_pub = self.create_publisher(String, '/detected_color', 10)

    def image_callback(self, msg):

        # Converte a imagem do ROS para o formato da openCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # Processa a imagem para reconher cores
        detected_color = self.process_image(cv_image)
        color_msg = String()

        if detected_color is not None:
            color_msg.data = detected_color
 
        
        print(f"Color before assignment: {detected_color}, Type: {type(detected_color)}")

    def process_image(self, cv_image):

        blurred_image = cv2.GaussianBlur(cv_image, (5, 5), 0)
        hsv_image = cv2.cvtColor(blurred_image, cv2.COLOR_BGR2HSV)

        color_ranges = {
        "red1": ([0, 100, 100], [10, 255, 255]), #precisamos de duas ranges para red pois está nos cantos do range de HSV
        "red2": ([160, 100, 100], [180, 255, 255]),
        "green": ([40, 50, 50], [70, 255, 255]),
        "blue": ([100, 150, 0], [140, 255, 255]),
        "pink": ([141, 50, 50], [159, 255, 255])
    }
        detected_color = '' 
        max_area = 0

        for color, (lower, upper) in color_ranges.items():
            # Mascara binário para a cor atual(red->green->blue->yellow)
            lower_bound = np.array(lower, dtype=np.uint8)
            upper_bound = np.array(upper, dtype=np.uint8)
            color_mask = cv2.inRange(hsv_image, lower_bound, upper_bound)

            #mask = cv2.inRange(hsv_image, lower, upper)

            kernel = np.ones((5, 5), np.uint8)
            color_mask = cv2.erode(color_mask, kernel, iterations=1)
            color_mask = cv2.dilate(color_mask, kernel, iterations=1)

            if color == "red1":
                red_mask1 = color_mask
                continue
            elif color == "red2":
                red_mask2 = color_mask
                color_mask = cv2.bitwise_or(red_mask1, red_mask2)

            #cv2.imshow(f"{color}_mask", color_mask) #printa 

            # Achando os contornos na mascara binaria
            contours, _ = cv2.findContours(color_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # Processando os contornos detectados
            for contour in contours:
                                              
                area = cv2.contourArea(contour) # Filtra os contornos baseado na sua area
                if area < 500:  # ajustar esse numero
                    continue

                if area > max_area:
                    max_area = area
                    detected_color = color

            print(f"[COR ATUAL]: {detected_color}")
            #print(f"{color}_max_area:", max_area)

        # Publish the detected color
        color_msg = String()
        color_msg.data = detected_color
        self.color_pub.publish(color_msg)

        cv2.waitKey(1)

        # Return detected color as a string
        return detected_color

def main(args=None):
    rclpy.init(args=args)
    node = ImageProcessingNode()
    print("Criou o novo processing node dilate")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()