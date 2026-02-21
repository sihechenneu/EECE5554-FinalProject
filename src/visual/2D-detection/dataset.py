import json, os, shutil, random, subprocess, sys
from pathlib import Path
from tqdm import tqdm

_WORKSPACE_ROOT = Path(__file__).resolve().parents[3]
DATASET_ROOT = _WORKSPACE_ROOT / "datasets" / "taco_yolo"
TACO_REPO    = _WORKSPACE_ROOT / "TACO"
VAL_RATIO    = 0.2
SEED         = 42

SUPERCATEGORY_TO_CLASS = {
    "Bottle": "Bottle", "Can": "Can", "Plastic bag & wrapper": "Plastic bag",
    "Carton": "Carton", "Cup": "Cup", "Lid": "Lid",
    "Cigarette": "Cigarette", "Paper": "Paper", "Straw": "Straw",
}
DEFAULT_CLASS = "Other"


def clone_and_download():
    if not TACO_REPO.exists():
        subprocess.run(["git", "clone", "https://github.com/pedropro/TACO.git"], check=True)
    subprocess.run([sys.executable, "download.py"], cwd=str(TACO_REPO), check=True)


def build_class_map(categories):
    class_names = list(dict.fromkeys(list(SUPERCATEGORY_TO_CLASS.values()) + [DEFAULT_CLASS]))
    name_to_idx = {n: i for i, n in enumerate(class_names)}
    cat_map = {}
    for cat in categories:
        cls = SUPERCATEGORY_TO_CLASS.get(cat.get("supercategory", ""), DEFAULT_CLASS)
        cat_map[cat["id"]] = name_to_idx[cls]
    return cat_map, class_names


def coco_to_yolo_labels(ann_file, cat_map):
    with open(ann_file) as f:
        coco = json.load(f)
    id_to_img = {img["id"]: img for img in coco["images"]}
    labels = {}
    for ann in coco["annotations"]:
        img = id_to_img.get(ann["image_id"])
        if img is None or img["width"] == 0 or img["height"] == 0:
            continue
        if ann["category_id"] not in cat_map:
            continue
        fname = img["file_name"]
        iw, ih = img["width"], img["height"]
        x, y, w, h = ann["bbox"]
        # COCO bbox [x,y,w,h] -> YOLO normalised [cx, cy, w, h]
        cx, cy, nw, nh = [(x+w/2)/iw, (y+h/2)/ih, w/iw, h/ih]
        cx, cy, nw, nh = [max(0.0, min(1.0, v)) for v in [cx, cy, nw, nh]]
        labels.setdefault(fname, []).append(f"{cat_map[ann['category_id']]} {cx:.6f} {cy:.6f} {nw:.6f} {nh:.6f}")
    return labels, coco["images"]


def create_yolo_dataset(labels, images_meta, class_names):
    for split in ("train", "val"):
        (DATASET_ROOT / "images" / split).mkdir(parents=True, exist_ok=True)
        (DATASET_ROOT / "labels" / split).mkdir(parents=True, exist_ok=True)

    fnames = sorted(labels.keys())
    random.seed(SEED)
    random.shuffle(fnames)
    val_set = set(fnames[:int(len(fnames) * VAL_RATIO)])

    for fname in tqdm(fnames, desc="Building dataset"):
        split = "val" if fname in val_set else "train"
        src = TACO_REPO / "data" / fname
        if not src.exists():
            src = TACO_REPO / fname
        if not src.exists():
            continue
        stem, suffix = Path(fname).stem, Path(fname).suffix
        shutil.copy2(src, DATASET_ROOT / "images" / split / f"{stem}{suffix}")
        with open(DATASET_ROOT / "labels" / split / f"{stem}.txt", "w") as f:
            f.write("\n".join(labels[fname]) + "\n")

    with open(DATASET_ROOT / "data.yaml", "w") as f:
        f.write(f"path: {DATASET_ROOT.resolve()}\ntrain: images/train\nval: images/val\n")
        f.write(f"nc: {len(class_names)}\nnames: {class_names}\n")


def main():
    # clone_and_download()
    all_labels, all_images = {}, []
    for ann_file in [TACO_REPO / "data" / "annotations.json",
                     TACO_REPO / "data" / "annotations_unofficial.json"]:
        if not ann_file.exists():
            continue
        with open(ann_file) as f:
            coco = json.load(f)
        cat_map, class_names = build_class_map(coco["categories"])
        labels, imgs = coco_to_yolo_labels(ann_file, cat_map)
        for k, v in labels.items():
            all_labels.setdefault(k, []).extend(v)
        all_images.extend(imgs)
    create_yolo_dataset(all_labels, all_images, class_names)

if __name__ == "__main__":
    main()