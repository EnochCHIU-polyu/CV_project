import sys
import os

# Try to import ROS 2 libraries, fallback to specific mock if missing
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
except ImportError:
    # Add the ../../../ directory to path so we can import mock_ros
    # Normalized path: cvProject/ros
    full_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../.."))
    sys.path.append(full_path)
    import mock_ros as rclpy
    from mock_ros import Node
    from mock_cv_bridge import CvBridge
    # Dummy Image class for type hinting
    class Image: pass

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
            
            # Store detections for the logic controller
            self.latest_detections = []
            
            for r in results:
                boxes = r.boxes
                for box in boxes:
                    # Get box coordinates
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    
                    # Calculate center (for navigation)
                    cx = int((x1 + x2) / 2)
                    cy = int((y1 + y2) / 2)
                    
                    # Get Class Name
                    cls_id = int(box.cls[0])
                    class_name = self.model.names[cls_id]
                    
                    self.latest_detections.append({
                        'class_name': class_name,
                        'box': [x1, y1, x2, y2],
                        'center': (cx, cy)
                    })
                    
                    # Draw Center Point for Visualization
                    cv2.circle(cv_image, (cx, cy), 5, (0, 0, 255), -1)
                    cv2.putText(cv_image, f"{class_name} ({cx},{cy})", (cx + 10, cy), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

            # Visualize (simple)
            # Plot returns a BGR numpy array
            annotated_frame = cv_image # results[0].plot() # Use our custom drawing or theirs
            
            # For demonstration without Rviz, show the window here
            cv2.imshow("YOLO Detection", annotated_frame)
            cv2.waitKey(1)
            
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