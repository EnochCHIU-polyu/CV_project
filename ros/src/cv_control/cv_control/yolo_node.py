import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO

class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo_node')
        # NOTE: Model path might need to be absolute or configured properly
        self.declare_parameter('model', 'yolo26n.pt') 
        
        # Initialize YOLO
        model_name = self.get_parameter('model').get_parameter_value().string_value
        self.get_logger().info(f'Loading YOLO model: {model_name}')
        self.model = YOLO(model_name)
        
        # ROS Communication
        self.subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(Image, 'yolo/detections', 10)
        self.bridge = CvBridge()
        self.get_logger().info('Yolo Node has been started.')

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Inference
            results = self.model(cv_image)
            
            # Visualize (simple)
            # Plot returns a BGR numpy array
            annotated_frame = results[0].plot()
            
            # Publish result
            out_msg = self.bridge.cv2_to_imgmsg(annotated_frame, "bgr8")
            self.publisher_.publish(out_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing frame: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = YoloNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()