from ultralytics import YOLO

def train():
    # Load a model
    model = YOLO("yolo26n.pt")  # load a pretrained model (recommended for training)

    # Train the model
    # data="coco8.yaml" is a toy dataset built-in to ultralytics for testing.
    # Replace it with your custom dataset path (e.g., 'datasets/my_data.yaml')
    results = model.train(data="coco8.yaml", epochs=100, imgsz=640)
    
    # Evaluate performance
    metrics = model.val()
    
    # Export the model
    success = model.export(format="onnx")

if __name__ == '__main__':
    train()