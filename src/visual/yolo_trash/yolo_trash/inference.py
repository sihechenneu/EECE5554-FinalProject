import argparse
from pathlib import Path
import sys

_pkg_root = Path(__file__).resolve().parents[1]
if _pkg_root not in sys.path:
    sys.path.insert(0, str(_pkg_root))
from yolo_trash.detector import DEFAULT_WEIGHTS


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--source", required=True, help="Image path, video path, or 0 for webcam")
    p.add_argument("--weights", default=str(DEFAULT_WEIGHTS))
    p.add_argument("--conf", type=float, default=0.35)
    p.add_argument("--iou", type=float, default=0.45)
    p.add_argument("--imgsz", type=int, default=640)
    p.add_argument("--show", action="store_true")
    p.add_argument("--device", default="")
    args = p.parse_args()

    from ultralytics import YOLO
    model = YOLO(args.weights)
    source = int(args.source) if args.source.isdigit() else args.source
    stream = isinstance(source, int) or Path(str(source)).suffix.lower() in (".mp4", ".avi", ".mov", ".mkv")
    for r in model.predict(
        source=source,
        conf=args.conf,
        iou=args.iou,
        imgsz=args.imgsz,
        show=args.show,
        save=True,
        device=args.device or None,
        stream=stream,
    ):
        if r.boxes is not None and len(r.boxes):
            for box in r.boxes:
                print(model.names[int(box.cls[0])], float(box.conf[0]), box.xyxy[0].tolist())


if __name__ == "__main__":
    main()
