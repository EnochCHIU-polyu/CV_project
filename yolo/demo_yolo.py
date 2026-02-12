from ultralytics import YOLO

# Load the YOLO26 model (Assuming the model weight naming convention exists for 26)
# If not, it will auto-download or we can switch to standard names.
# Based on the fetched context, "yolo26n.pt" is valid.

def detect():
    model = YOLO("yolo26n.pt")  # Load YOLO26 nano model
    
    # Run inference on a test image or webcam
    # We can use the webcam as input 0
    results = model.predict(source="0", show=True, stream=True)
    
    for r in results:
        pass # The show=True handles the display

if __name__ == "__main__":
    detect()
