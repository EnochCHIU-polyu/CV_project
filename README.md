# CV Project: Hand & Object Navigation System

This project integrates **MMPose** (for hand keypoint detection), **YOLOv8** (for object detection), and a **Mock ROS** architecture to demonstrate a computer vision control system.

## Project Structure

- **`handDetection/`**: Contains the source code for MMPose (OpenMMLab Pose Estimation).
- **`yolo/`**: Contains resources for YOLO object detection.
- **`ros/`**: Contains the project integration code, nodes, and the main demo script.
  - `src/cv_control`: Custom ROS-style nodes for Hand and YOLO logic.
  - `mock_ros.py`: A simulation layer to run ROS-style nodes without a full ROS installation.
  - `run_project_demo.py`: The main entry point.

## Environment Setup

### 1. Prerequisites

- Python 3.8 or higher.
- Anaconda or Miniconda (recommended).

### 2. Create and Activate Environment

```bash
conda create -n cv_project python=3.8 -y
conda activate cv_project
```

### 3. Install PyTorch

Install PyTorch compatible with your system (MacOS/CPU):

```bash
pip install torch torchvision
```

### 4. Install MMPose (Hand Detection)

Install the requirements and the library from the local `handDetection` folder:

```bash
cd handDetection
pip install -r requirements.txt
pip install -e .
cd ..
```

_Note: You may also need `mmengine` and `mmcv`. If prompted, install them via `pip install -U openmim && mim install mmengine mmcv`._

### 5. Install Ultralytics (YOLO)

```bash
pip install ultralytics
```

### 6. Install Other Dependencies

```bash
pip install opencv-python numpy
```

## How to Run

### Command Line

Navigate to the `ros` directory and run the demo script:

```bash
cd ros
python run_project_demo.py
```

## Developer Guide

### 1. How to Modify the Target Object

The system is currently configured to look for a **"cup"**. To change this:

1. Open [`ros/run_project_demo.py`](ros/run_project_demo.py).
2. Look for the variable `target_class` (around line 58).
   ```python
   target_class = "cup"  # Change "cup" to "bottle", "cell phone", etc.
   ```
   _Note: The object name must match a class label from the COCO dataset (used by YOLO)._

### 2. How to Access Guidance Output (Left/Right)

The system calculates the relative position of the hand to the object to provide guidance (e.g., "Left", "Right").
You can find and modify this logic in [`ros/run_project_demo.py`](ros/run_project_demo.py):

- **Location**: Inside the main loop, under `# --- GUIDANCE LOGIC ---`.
- **Key Variables**:
  - `dx`, `dy`: Distances between the hand and the target.
  - `msg`: The guidance string (e.g., "Left", "Up").
- **Customization**:
  To send this output to another system or print it to the terminal, you can add code here:
  ```python
  if msg:
      print(f"Instruction: {msg}")  # Or publish to a ROS topic
  ```

### 3. Where to Find the Code

- **Main Controller**: [`ros/run_project_demo.py`](ros/run_project_demo.py)
  - Handles the webcam, synchronization, and high-level logic.
- **Object Detection**: [`ros/src/cv_control/cv_control/yolo_node.py`](ros/src/cv_control/cv_control/yolo_node.py)
  - Wraps the YOLO model and outputs bounding boxes.
- **Hand Pose**: [`ros/src/cv_control/cv_control/hand_node.py`](ros/src/cv_control/cv_control/hand_node.py)
  - Wraps MMPose and returns keypoints given an image and a bounding box.
