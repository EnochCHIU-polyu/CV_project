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

### Features

- **Mirror Effect**: The webcam feed is mirrored for more natural interaction.
- **Hand Guidance**: The system detects your hand and a target object (e.g., a cup) and calculates a guidance vector.
- **Mock ROS**: Uses a lightweight shim to simulate ROS 2 nodes, publishers, and subscribers.
