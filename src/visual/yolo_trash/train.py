import argparse
from pathlib import Path
from ultralytics import YOLO
from ultralytics.data.utils import check_det_dataset
workspace_root = Path(__file__).resolve().parents[3]

def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--model",  default="yolov8n.pt") # YOLO-nano (3.2M), for Raspberry Pi
    p.add_argument("--data",   default=str(workspace_root / "datasets/taco_yolo/data.yaml"))
    p.add_argument("--epochs", type=int, default=60)
    p.add_argument("--batch",  type=int, default=8)
    p.add_argument("--imgsz",  type=int, default=640)
    p.add_argument("--device", default="")
    p.add_argument("--project", default=str(workspace_root / "runs/detect"))
    p.add_argument("--name",   default="taco_yolov8")
    return p.parse_args()

def main():
    args = parse_args()
    model = YOLO(args.model)
    check_det_dataset(args.data)

    model.train(
        data=args.data, epochs=args.epochs, batch=args.batch,
        imgsz=args.imgsz, device=args.device or None, name=args.name,
        patience=15, save=True, pretrained=True, augment=True,
        cls=1.0,
        hsv_h=0.015, hsv_s=0.7, hsv_v=0.4,
        degrees=5.0, translate=0.1, scale=0.3, fliplr=0.5,
        copy_paste=0.1, mosaic=1.0, mixup=0.0,
    )

    metrics = model.val()
    print(f"mAP50: {metrics.box.map50:.4f}  mAP50-95: {metrics.box.map:.4f}")
    model.export(format="ncnn") # for Raspberry Pi

if __name__ == "__main__":
    main()