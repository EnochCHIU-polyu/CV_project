# YOLO26 Setup for CV Project

This folder contains the Object Detection module using **YOLO26** (via `ultralytics`).

## Environment

Ensure you are using the `openmmlab` Conda environment where `ultralytics` has been installed.

```bash
conda activate openmmlab
```

## Quick Start

Run the webcam demo to verify installation:

```bash
python demo_yolo.py
```

This will automatically download the `yolo26n.pt` (Nano) model and start inference.

## Training a Custom Model

To train YOLO26 on your own dataset (e.g., specific objects for your project):

1. Prepare a `dataset.yaml` file (see `example_dataset.yaml`).
2. Run the training script:
   ```bash
   yolo detect train data=dataset.yaml model=yolo26n.pt epochs=100 imgsz=640
   ```
   Or use the Python script:
   ```bash
   python train.py
   ```

## Integration with ROS

In your ROS node (in `../ros/src/cv_control`), you can simply import the model:

```python
from ultralytics import YOLO
model = YOLO('path/to/best.pt')
results = model(image)
```
