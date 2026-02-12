import cv2
import sys
import os
import time

# Ensure we can import the nodes
sys.path.append(os.path.abspath("src/cv_control"))

# Import the nodes from the package
from cv_control.yolo_node import YoloNode
from cv_control.hand_node import HandNode
import mock_ros as rclpy
from mock_cv_bridge import CvBridge

def run():
    print("--- Starting Project Demo (No ROS Mode) ---")
    rclpy.init()

    # Instantiate Nodes
    yolo_node = YoloNode()
    hand_node = HandNode()
    
    # Setup Camera
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Could not open webcam.")
        return

    bridge = CvBridge()
    
    print("Press 'q' to quit.")
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
            
        # Mirror the frame (horizontal flip) for intuitive interaction
        frame = cv2.flip(frame, 1)

        # Simulate ROS Message
        ros_image_msg = bridge.cv2_to_imgmsg(frame)
        
        # 1. Pipeline: Image -> YOLO
        # We manually call the callback
        # In a real ROS system, the node would receive a message. 
        # Here we invoke the processing logic directly or mock the callback behavior.
        
        # YOLO Processing
        try:
            # Run YOLO (visualizes and stores detections)
            yolo_node.listener_callback(ros_image_msg)
            
            # Extract detections for High-Level Logic
            detections = getattr(yolo_node, 'latest_detections', [])
            
            # Logic Controller: "Space Navigation"
            # Example: Find "cup" and navigate to it.
            target_class = "cup"
            target_found = False
            
            person_boxes = [] # For hand detection
            
            target_center = None
            
            for det in detections:
                if det['class_name'] == target_class:
                    cx, cy = det['center']
                    target_center = (cx, cy)
                    cv2.putText(frame, f"TARGET: {target_class}", (cx, cy-20), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    print(f"[NAV] Target '{target_class}' found at ({cx}, {cy}).")
                    target_found = True
                
                if det['class_name'] == 'person':
                    person_boxes.append(det['box'])

            if not target_found:
                 print(f"[NAV] Searching for '{target_class}'...")

            # Hand Processing (on Person boxes)
            hand_centers = []
            if len(person_boxes) > 0:
                hand_centers = hand_node.process_with_boxes(frame, person_boxes) 
            else:
                 hand_node.process_with_boxes(frame, [])

            # --- GUIDANCE LOGIC ---
            # If we see a hand and a target, guide the hand to the target
            if target_center and hand_centers:
                # Pick the first hand found
                hx, hy = hand_centers[0]
                tx, ty = target_center
                
                # Draw line from hand to target
                cv2.line(frame, (int(hx), int(hy)), (int(tx), int(ty)), (255, 0, 0), 3)
                
                # Calculate instructions
                dx = tx - hx
                dy = ty - hy
                
                msg = ""
                if abs(dx) > 20:
                    msg += "Right " if dx > 0 else "Left "
                if abs(dy) > 20:
                    msg += "Down " if dy > 0 else "Up "
                
                if msg == "":
                    msg = "GRAB!"
                    color = (0, 255, 0) # Green for grab
                else:
                    msg += "..."
                    color = (0, 255, 255) # Yellow for moving
                
                cv2.putText(frame, f"GUIDANCE: {msg}", (50, 450), 
                            cv2.FONT_HERSHEY_SIMPLEX, 1.5, color, 3)
                print(f"[GUIDE] {msg}")

        except Exception as e:
            print(f"Error in processing: {e}")

        # Visualization for the Controller Loop
        cv2.putText(frame, "System Running... Check logs.", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.imshow("Main Feed", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == "__main__":
    run()