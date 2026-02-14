import argparse
from pathlib import Path
from ultralytics import YOLO
workspace_root = Path(__file__).resolve().parents[3]

def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--weights", default=str(workspace_root / "runs/detect/taco_yolov8/weights/best.pt"))
    p.add_argument("--source",  required=True)
    p.add_argument("--conf",    type=float, default=0.35)
    p.add_argument("--iou",     type=float, default=0.45)
    p.add_argument("--imgsz",   type=int, default=640)
    p.add_argument("--show",    action="store_true")
    p.add_argument("--save-txt", action="store_true")
    p.add_argument("--device",  default="")
    return p.parse_args()

def main():
    args = parse_args()
    model = YOLO(args.weights)
    source = int(args.source) if args.source.isdigit() else args.source
    is_stream = isinstance(source, int) or Path(str(source)).suffix in ('.mp4','.avi','.mov','.mkv')

    results = model.predict(
        source=source, conf=args.conf, iou=args.iou, imgsz=args.imgsz,
        show=args.show, save=True, save_txt=args.save_txt,
        device=args.device or None, stream=is_stream,
    )
    for r in results:
        if r.boxes is not None and len(r.boxes):
            for box in r.boxes:
                name = model.names[int(box.cls[0])]
                print(f"{name}  conf={float(box.conf[0]):.2f}  bbox={box.xyxy[0].tolist()}")

if __name__ == "__main__":
    main()