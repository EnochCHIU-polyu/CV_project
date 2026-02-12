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
    full_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../.."))
    sys.path.append(full_path)
    import mock_ros as rclpy
    from mock_ros import Node
    from mock_cv_bridge import CvBridge
    class Image: pass

import cv2
import numpy as np
import torch
import os

# MMPose imports
from mmpose.apis import inference_topdown, init_model
from mmpose.utils import register_all_modules

class HandNode(Node):
    def __init__(self):
        super().__init__('hand_node')
        
        # NOTE: You must update these paths to absolute paths or use ROS parameters to fetch them dynamically
        # Current placeholders point to your handDetection setup
        self.declare_parameter('config_file', '/Users/yeechiu/cvProject/handDetection/configs/hand_2d_keypoint/rtmpose/hand5/rtmpose-m_8xb256-210e_hand5-256x256.py')
        self.declare_parameter('checkpoint_file', 'https://download.openmmlab.com/mmpose/v1/projects/rtmposev1/rtmpose-m_simcc-hand5_pt-aic-coco_210e-256x256-74fb594_20230320.pth')
        
        config_file = self.get_parameter('config_file').get_parameter_value().string_value
        checkpoint_file = self.get_parameter('checkpoint_file').get_parameter_value().string_value
        
        self.get_logger().info('Initializing MMPose model...')
        register_all_modules()
        
        # Check if config exists
        if not os.path.exists(config_file):
             self.get_logger().warn(f'Config file not found at {config_file}. Model Init might fail.')
        
        self.model = init_model(config_file, checkpoint_file, device='cpu') # Use 'cuda:0' if available
        
        self.subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.listener_callback,
            10)
        self.bridge = CvBridge()
        self.get_logger().info('Hand Node initialized.')

    def process_with_boxes(self, cv_image, boxes):
        """
        Custom method to run inference using external boxes (e.g. from YOLO).
        boxes: list of [x1, y1, x2, y2]
        """
        if len(boxes) == 0:
            cv2.imshow("Hand Node Input", cv_image)
            cv2.waitKey(1)
            return

        try:
            # inference_topdown(model, img, bboxes=...)
            results = inference_topdown(self.model, cv_image, bboxes=np.array(boxes))
            
            detected_hands = []

            # Manual visualization
            for result in results:
                if hasattr(result, 'pred_instances'):
                    keypoints = result.pred_instances.keypoints[0]
                    scores = result.pred_instances.keypoint_scores[0]
                    
                    # Calculate approximate hand center (average of valid keypoints)
                    valid_kpts = []
                    for (x, y), score in zip(keypoints, scores):
                        if score > 0.3:
                            cv2.circle(cv_image, (int(x), int(y)), 3, (0, 255, 0), -1)
                            valid_kpts.append((x, y))
                    
                    if valid_kpts:
                        # Simple average for hand center
                        avg_x = np.mean([k[0] for k in valid_kpts])
                        avg_y = np.mean([k[1] for k in valid_kpts])
                        detected_hands.append((avg_x, avg_y))

            cv2.imshow("Hand Node Input", cv_image)
            cv2.waitKey(1)
            
            return detected_hands
            
        except Exception as e:
            self.get_logger().error(f'Error in hand process: {str(e)}')
            return []

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Simple inference requires detected bounding boxes from a separate detector (like YoloNode)
            # For this demo, we can assume the whole image is the bounding box or skip detection
            # results = inference_topdown(self.model, cv_image)
            
            # Integration Step:
            # 1. Receive Image
            # 2. (Optional) Receive Detection Boxes from YoloNode (via separate topic)
            # 3. Run Pose Estimation on boxes
            
            # Simple visualization
            cv2.imshow("Hand Node Input", cv_image)
            cv2.waitKey(1)
            
            pass 
            
        except Exception as e:
            self.get_logger().error(f'Error in hand detection: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = HandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()