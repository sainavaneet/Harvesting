import cv2
from ultralytics import YOLO

def load_model():
    
    model = YOLO("/home/dexweaver/Github/cucumber-harvesting/object_detection/runs/detect/train/weights/best.pt")  
    return model

def process_frame(model, frame):
    
    results = model(frame)
    
    
    for result in results:
        for box in result.boxes:  
            x1, y1, x2, y2 = map(int, box.xyxy[0])  
            conf = box.conf[0]  
            cls = int(box.cls[0])  
            label = model.names[cls]  

            
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, f"{label} {conf:.2f}", (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
    return frame

def main():
    
    model = load_model()

    
    cap = cv2.VideoCapture(0)  

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame = process_frame(model, frame)
        cv2.imshow("YOLOv8 Real-Time Detection", frame)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
