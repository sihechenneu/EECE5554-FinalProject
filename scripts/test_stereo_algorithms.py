#!/usr/bin/env python3
"""
Test stereo algorithms: SGBM, WLS filtering, and trifocal validation.

1. Semi-Global Block Matching (SGBM): better at filling flat surfaces than BM.
   Uses cv2.StereoSGBM_create with configurable block_size (e.g. 11 or 15).

2. Weighted Least Squares (WLS): post-processes disparity using a guide image
   (left or middle/RGB) to respect edges and fill gaps.
   Uses cv2.ximgproc.createDisparityWLSFilter (opencv-contrib).

3. Trifocal validation: left-right disparity is reprojected to the middle camera;
   inconsistent pixels are masked out to reduce ghosting and noise.

Usage:
  # From image files (rectified left/right, optional middle)
  python test_stereo_algorithms.py --left left.png --right right.png [--middle middle.png]

  # From a ROS2 bag (extracts one frame per topic)
  python test_stereo_algorithms.py --bag path/to/bag --topic-left /oakd/left/image_raw \\
      --topic-right /oakd/right/image_raw [--topic-middle /oakd/rgb/image_raw]

  # Options
  --block-size 11          SGBM block size (larger = more context, try 11 or 15)
  --no-wls                 Skip WLS filtering
  --no-trifocal            Skip trifocal validation (needs middle + calibration)
  --calibration YAML       Optional calibration file for Q matrix and middle P (for trifocal).
                           YAML keys: Q (4x4), P_middle (3x4). Example:
                             Q: [[1,0,0,-cx],[0,1,0,-cy],[0,0,0,fx],[0,0,-1/baseline,(cx)/baseline]]
                             P_middle: 3x4 projection matrix from left camera to middle image
  --out-dir DIR            Save visualizations to this directory (required for --batch)
  --batch                  Process all frames in the bag and save to --out-dir (no GUI)
  --max-frames N           When using --batch, process at most N frames (0 = all)
  --sync-window-ms MS      Max time diff (ms) to pair left/right/middle (default 50)

Requirements: opencv-python, numpy. For WLS: opencv-contrib-python. For --bag: ROS2, rosbag2_py.
"""

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np


def load_image(path: str) -> np.ndarray:
    """Load image; convert to BGR if needed."""
    p = Path(path)
    if not p.exists():
        raise FileNotFoundError(f"Image not found: {path}")
    img = cv2.imread(str(p))
    if img is None:
        raise ValueError(f"Could not load image: {path}")
    return img


def to_grayscale(img: np.ndarray) -> np.ndarray:
    if img.ndim == 3:
        return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    return img


def compute_sgbm(
    left: np.ndarray,
    right: np.ndarray,
    block_size: int = 11,
    min_disp: int = 0,
    num_disp: int = 128,
    uniqueness: float = 10.0,
    speckle_window: int = 100,
    speckle_range: int = 32,
) -> np.ndarray:
    """
    Semi-Global Block Matching. Returns 16-bit signed disparity (values * 16).
    """
    if left.ndim == 3:
        left = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
    if right.ndim == 3:
        right = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)
    if block_size % 2 == 0:
        block_size += 1
    if num_disp % 16 != 0:
        num_disp = ((num_disp + 15) // 16) * 16

    matcher = cv2.StereoSGBM_create(
        minDisparity=min_disp,
        numDisparities=num_disp,
        blockSize=block_size,
        P1=8 * 3 * block_size ** 2,
        P2=32 * 3 * block_size ** 2,
        disp12MaxDiff=1,
        uniquenessRatio=int(uniqueness),
        speckleWindowSize=speckle_window,
        speckleRange=speckle_range,
        mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY,
    )
    disp = matcher.compute(left, right)
    return disp


def compute_sgbm_right(left: np.ndarray, right: np.ndarray, block_size: int = 11, num_disp: int = 128) -> np.ndarray:
    """Right matcher for left-right consistency (same params as left matcher)."""
    if left.ndim == 3:
        left = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
    if right.ndim == 3:
        right = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)
    if block_size % 2 == 0:
        block_size += 1
    if num_disp % 16 != 0:
        num_disp = ((num_disp + 15) // 16) * 16
    matcher = cv2.StereoSGBM_create(
        minDisparity=0,
        numDisparities=num_disp,
        blockSize=block_size,
        P1=8 * 3 * block_size ** 2,
        P2=32 * 3 * block_size ** 2,
        disp12MaxDiff=1,
        uniquenessRatio=10,
        speckleWindowSize=100,
        speckleRange=32,
        mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY,
    )
    # Right disparity: swap left and right (right image as "left" view)
    disp_right = matcher.compute(right, left)
    return disp_right


def wls_filter(
    disp_left: np.ndarray,
    left_guide: np.ndarray,
    disp_right: np.ndarray | None = None,
    num_disparities: int = 128,
    lambda_: float = 8000.0,
    sigma_color: float = 1.5,
) -> np.ndarray:
    """
    Weighted Least Squares filter on disparity using left (or guide) image.
    Requires opencv-contrib (cv2.ximgproc).
    """
    try:
        from cv2 import ximgproc
    except ImportError:
        raise ImportError("WLS filter requires opencv-contrib-python: pip install opencv-contrib-python")

    if left_guide.ndim == 3:
        left_guide = cv2.cvtColor(left_guide, cv2.COLOR_BGR2GRAY)
    num_disp = max(16, (num_disparities + 15) // 16 * 16)
    matcher = cv2.StereoSGBM_create(
        minDisparity=0,
        numDisparities=num_disp,
        blockSize=5,
    )
    wls = ximgproc.createDisparityWLSFilter(matcher)
    wls.setLambda(lambda_)
    wls.setSigmaColor(sigma_color)
    filtered = wls.filter(disp_left, left_guide, None, disp_right)
    return filtered


def disparity_to_vis(disp: np.ndarray, scale: float = 1.0, min_disp: float = 0) -> np.ndarray:
    """Convert 16-bit disparity (OpenCV convention: true_disp = disp/16) to BGR vis."""
    disp_float = np.float32(disp) / 16.0
    valid = (disp_float > min_disp) & (disp_float < 1000)
    out = np.zeros((*disp.shape, 3), dtype=np.uint8)
    if np.any(valid):
        min_val = max(min_disp, np.percentile(disp_float[valid], 2))
        max_val = min(1000, np.percentile(disp_float[valid], 98))
        if max_val <= min_val:
            max_val = min_val + 1
        normalized = np.clip((disp_float - min_val) / (max_val - min_val), 0, 1)
        normalized[~valid] = 0
        normalized_uint8 = (normalized * 255).astype(np.uint8)
        out = cv2.applyColorMap(normalized_uint8, cv2.COLORMAP_TURBO)
        out[~valid] = 0
    return out


def trifocal_consistency_mask(
    disp_left: np.ndarray,
    left_img: np.ndarray,
    middle_img: np.ndarray,
    Q: np.ndarray,
    P_middle: np.ndarray,
    max_reproj_px: float = 2.0,
    max_intensity_diff: int = 40,
) -> np.ndarray:
    """
    Compute a mask of pixels that are consistent across left and middle.
    Left disparity -> 3D (via Q) -> project to middle. Compare middle pixel to expected.
    P_middle is 3x4 projection matrix (middle camera). Q is 4x4 from stereoRectify (left-right).
    Returns binary mask (255 = consistent, 0 = invalid).
    """
    h, w = disp_left.shape[:2]
    points_3d = cv2.reprojectImageTo3D(
        np.int16(disp_left), Q, handleMissingValues=True
    )
    valid_disp = (disp_left > 0) & (points_3d[:, :, 2] > 0) & (points_3d[:, :, 2] < 1e6)
    if middle_img.ndim == 3:
        middle_gray = np.asarray(cv2.cvtColor(middle_img, cv2.COLOR_BGR2GRAY), dtype=np.float32)
    else:
        middle_gray = np.asarray(middle_img, dtype=np.float32)
    if left_img.ndim == 3:
        left_gray = np.asarray(cv2.cvtColor(left_img, cv2.COLOR_BGR2GRAY), dtype=np.float32)
    else:
        left_gray = np.asarray(left_img, dtype=np.float32)

    pts = points_3d.reshape(-1, 3)
    pts_h = np.hstack([pts, np.ones((len(pts), 1), dtype=np.float64)])
    proj = (P_middle @ pts_h.T).T
    proj_uv = (proj[:, :2] / (proj[:, 2:3] + 1e-8)).reshape(h, w, 2)
    proj_z = proj[:, 2].reshape(h, w)
    um_i = np.round(proj_uv[:, :, 0]).astype(np.int32)
    vm_i = np.round(proj_uv[:, :, 1]).astype(np.int32)

    in_bounds = (
        (vm_i >= 0) & (vm_i < middle_gray.shape[0]) &
        (um_i >= 0) & (um_i < middle_gray.shape[1]) &
        (proj_z > 0)
    )
    middle_vals = np.zeros_like(left_gray)
    middle_vals[in_bounds] = middle_gray[vm_i[in_bounds], um_i[in_bounds]]
    diff = np.abs(left_gray - middle_vals)
    reproj_ok = (
        np.abs(proj_uv[:, :, 0] - um_i) <= max_reproj_px &
        np.abs(proj_uv[:, :, 1] - vm_i) <= max_reproj_px
    )
    mask = np.where(
        valid_disp & in_bounds & reproj_ok & (diff <= max_intensity_diff),
        255, 0
    ).astype(np.uint8)
    return mask


def build_q_from_baseline(fx: float, fy: float, cx: float, cy: float, baseline: float) -> np.ndarray:
    """Build 4x4 Q matrix for reprojectImageTo3D (rectified left-right)."""
    Q = np.zeros((4, 4), dtype=np.float64)
    Q[0, 0] = 1.0
    Q[0, 3] = -cx
    Q[1, 1] = 1.0
    Q[1, 3] = -cy
    Q[2, 3] = fx
    Q[3, 2] = -1.0 / baseline
    Q[3, 3] = (cx - 0) / baseline  # right cx often 0 in rectified
    return Q


def load_from_bag(
    bag_path: str,
    topic_left: str,
    topic_right: str,
    topic_middle: str | None = None,
) -> tuple[np.ndarray, np.ndarray, np.ndarray | None]:
    """Load one frame from each topic (first message per topic). Returns (left, right, middle or None)."""
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from sensor_msgs.msg import Image
    except ImportError:
        print("ROS2 and rosbag2_py required for --bag. Source ROS2 and install ros-jazzy-rosbag2-py.", file=sys.stderr)
        sys.exit(1)

    bag_path = Path(bag_path)
    if not bag_path.exists():
        raise FileNotFoundError(f"Bag not found: {bag_path}")
    uri = str(bag_path.resolve())
    storage_options = rosbag2_py.StorageOptions(uri=uri, storage_id="")
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="",
        output_serialization_format="",
    )
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    def image_msg_to_array(msg):
        from sensor_msgs.msg import Image
        if not isinstance(msg, Image):
            return None
        h, w = msg.height, msg.width
        enc = msg.encoding.upper().replace(" ", "")
        if enc in ("8UC1", "8U_C1", "MONO8"):
            return np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w)
        if enc in ("8UC3", "8U_C3", "RGB8", "BGR8"):
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3)
            return cv2.cvtColor(arr, cv2.COLOR_RGB2BGR) if "BGR" not in msg.encoding else arr
        return None

    found = {"left": None, "right": None, "middle": None}
    topics = [topic_left, topic_right]
    if topic_middle:
        topics.append(topic_middle)
    try:
        reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
    except Exception:
        pass

    while reader.has_next():
        rec = reader.read_next()
        topic, data = rec[0], rec[1]
        if topic == topic_left and found["left"] is None:
            msg = deserialize_message(data, Image)
            arr = image_msg_to_array(msg)
            if arr is not None:
                found["left"] = arr
        elif topic == topic_right and found["right"] is None:
            msg = deserialize_message(data, Image)
            arr = image_msg_to_array(msg)
            if arr is not None:
                found["right"] = arr
        elif topic_middle and topic == topic_middle and found["middle"] is None:
            msg = deserialize_message(data, Image)
            arr = image_msg_to_array(msg)
            if arr is not None:
                found["middle"] = arr
        if found["left"] is not None and found["right"] is not None:
            if not topic_middle or found["middle"] is not None:
                break
    if found["left"] is None or found["right"] is None:
        raise RuntimeError("Could not read left and right images from bag.")
    return found["left"], found["right"], found["middle"]


def iter_frames_from_bag(
    bag_path: str,
    topic_left: str,
    topic_right: str,
    topic_middle: str | None = None,
    sync_window_ns: int = 50_000_000,
    max_frames: int = 0,
):
    """
    Yield (left, right, middle_or_none, frame_index, timestamp_ns) for each synced frame.
    Loads messages with timestamps, pairs by closest time within sync_window_ns.
    """
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from sensor_msgs.msg import Image
    except ImportError:
        print("ROS2 and rosbag2_py required. Source ROS2 and install ros-jazzy-rosbag2-py.", file=sys.stderr)
        sys.exit(1)

    bag_path = Path(bag_path)
    if not bag_path.exists():
        raise FileNotFoundError(f"Bag not found: {bag_path}")
    uri = str(bag_path.resolve())
    storage_options = rosbag2_py.StorageOptions(uri=uri, storage_id="")
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="",
        output_serialization_format="",
    )
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    def image_msg_to_array(msg):
        if not isinstance(msg, Image):
            return None
        h, w = msg.height, msg.width
        enc = msg.encoding.upper().replace(" ", "")
        if enc in ("8UC1", "8U_C1", "MONO8"):
            return np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w)
        if enc in ("8UC3", "8U_C3", "RGB8", "BGR8"):
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3)
            return cv2.cvtColor(arr, cv2.COLOR_RGB2BGR) if "BGR" not in msg.encoding else arr
        return None

    topics = [topic_left, topic_right]
    if topic_middle:
        topics.append(topic_middle)
    try:
        reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
    except Exception:
        pass

    left_list = []
    right_list = []
    middle_list = []

    while reader.has_next():
        rec = reader.read_next()
        topic, data, timestamp = rec[0], rec[1], rec[2]
        try:
            msg = deserialize_message(data, Image)
        except Exception:
            continue
        arr = image_msg_to_array(msg)
        if arr is None:
            continue
        ts = int(timestamp) if hasattr(timestamp, "__int__") else getattr(timestamp, "nanoseconds_since_epoch", 0)
        if topic == topic_left:
            left_list.append((ts, arr))
        elif topic == topic_right:
            right_list.append((ts, arr))
        elif topic_middle and topic == topic_middle:
            middle_list.append((ts, arr))

    if not left_list or not right_list:
        raise RuntimeError("No left or right images found in bag.")
    left_list.sort(key=lambda x: x[0])
    right_list.sort(key=lambda x: x[0])
    if middle_list:
        middle_list.sort(key=lambda x: x[0])
    n_right = len(right_list)
    n_middle = len(middle_list) if middle_list else 0
    sync_ns = sync_window_ns

    for idx, (t_l, left) in enumerate(left_list):
        if max_frames and idx >= max_frames:
            break
        # Closest right by time
        best_r = None
        best_dt = float("inf")
        for (t_r, right) in right_list:
            dt = abs(t_l - t_r)
            if dt < best_dt:
                best_dt = dt
                best_r = right
            if t_r > t_l + sync_ns:
                break
        if best_dt > sync_ns or best_r is None:
            continue
        # Closest middle by time (optional)
        middle = None
        if middle_list:
            best_dt_m = float("inf")
            for (t_m, mid) in middle_list:
                dt = abs(t_l - t_m)
                if dt < best_dt_m:
                    best_dt_m = dt
                    middle = mid
                if t_m > t_l + sync_ns:
                    break
            if best_dt_m > sync_ns:
                middle = None
        yield left, best_r, middle, idx, t_l


def run_batch(args, sync_window_ns: int) -> None:
    """Process all frames from the bag and save results to args.out_dir."""
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    block_size = args.block_size
    num_disp = max(16, (args.num_disparities + 15) // 16 * 16)
    topic_middle = args.topic_middle if not args.no_trifocal else None

    Q = None
    P_middle = None
    if args.calibration and not args.no_trifocal:
        try:
            import yaml
            with open(args.calibration) as f:
                cal = yaml.safe_load(f)
            if cal and "Q" in cal and "P_middle" in cal:
                Q = np.array(cal["Q"], dtype=np.float64)
                P_middle = np.array(cal["P_middle"], dtype=np.float64)
        except Exception as e:
            print(f"Calibration load failed: {e}")

    use_wls = not args.no_wls
    try:
        from cv2 import ximgproc
    except ImportError:
        use_wls = False
        print("WLS disabled (opencv-contrib not found).")

    frame_iter = iter_frames_from_bag(
        args.bag,
        args.topic_left,
        args.topic_right,
        topic_middle,
        sync_window_ns=sync_window_ns,
        max_frames=args.max_frames or 0,
    )
    count = 0
    for left, right, middle, idx, timestamp_ns in frame_iter:
        if left.shape[:2] != right.shape[:2]:
            continue
        if middle is not None and left.shape[:2] != middle.shape[:2]:
            middle = cv2.resize(middle, (left.shape[1], left.shape[0]), interpolation=cv2.INTER_LINEAR)

        prefix = out_dir / f"frame_{idx:06d}"
        cv2.imwrite(str(prefix) + "_left.png", left)
        cv2.imwrite(str(prefix) + "_right.png", right)
        if middle is not None:
            cv2.imwrite(str(prefix) + "_middle.png", middle)

        disp_sgbm = compute_sgbm(left, right, block_size=block_size, num_disp=num_disp)
        vis_sgbm = disparity_to_vis(disp_sgbm)
        cv2.imwrite(str(prefix) + "_disparity_sgbm.png", vis_sgbm)

        disp_wls = None
        if use_wls:
            try:
                disp_right = compute_sgbm_right(left, right, block_size=block_size, num_disp=num_disp)
                disp_wls = wls_filter(disp_sgbm, left, disp_right=disp_right, num_disparities=num_disp)
                vis_wls = disparity_to_vis(disp_wls)
                cv2.imwrite(str(prefix) + "_disparity_wls.png", vis_wls)
            except Exception as e:
                if count == 0:
                    print("WLS failed:", e)
        if disp_wls is None:
            disp_wls = disp_sgbm

        if middle is not None and Q is not None and P_middle is not None:
            mask_trifocal = trifocal_consistency_mask(
                disp_wls, left, middle, Q, P_middle,
            )
            cv2.imwrite(str(prefix) + "_trifocal_mask.png", mask_trifocal)
            disp_masked = disp_wls.copy()
            disp_masked[mask_trifocal == 0] = 0
            vis_masked = disparity_to_vis(disp_masked)
            cv2.imwrite(str(prefix) + "_disparity_trifocal.png", vis_masked)

        count += 1
        if count % 10 == 0 or count == 1:
            print(f"  Saved frame {idx} (total {count})")

    print(f"Done. Saved {count} frames to {out_dir}")


def main():
    parser = argparse.ArgumentParser(
        description="Test SGBM, WLS, and trifocal stereo algorithms.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--left", type=str, help="Path to left (rectified) image")
    parser.add_argument("--right", type=str, help="Path to right (rectified) image")
    parser.add_argument("--middle", type=str, default=None, help="Path to middle/RGB image (for WLS guide and trifocal)")
    parser.add_argument("--bag", type=str, help="Path to ROS2 bag (use with --topic-left/right/middle)")
    parser.add_argument("--topic-left", type=str, default="/oakd/left/image_raw")
    parser.add_argument("--topic-right", type=str, default="/oakd/right/image_raw")
    parser.add_argument("--topic-middle", type=str, default="/oakd/rgb/image_raw")
    parser.add_argument("--block-size", type=int, default=11, help="SGBM block size (e.g. 11 or 15)")
    parser.add_argument("--num-disparities", type=int, default=128, help="SGBM numDisparities (multiple of 16)")
    parser.add_argument("--no-wls", action="store_true", help="Skip WLS filtering")
    parser.add_argument("--no-trifocal", action="store_true", help="Skip trifocal validation")
    parser.add_argument("--calibration", type=str, default=None, help="YAML with Q, P_middle (optional, for trifocal)")
    parser.add_argument("--out-dir", type=str, default=None, help="Save visualizations to this directory (required for --batch)")
    parser.add_argument("--batch", action="store_true", help="Process all frames in bag and save to --out-dir (no GUI)")
    parser.add_argument("--max-frames", type=int, default=0, help="With --batch: process at most N frames (0 = all)")
    parser.add_argument("--sync-window-ms", type=float, default=50.0, help="Max time diff (ms) to pair left/right/middle")
    args = parser.parse_args()

    sync_window_ns = int(args.sync_window_ms * 1e6)
    use_batch = bool(args.bag and args.batch)
    if use_batch and not args.out_dir:
        print("--batch requires --out-dir.", file=sys.stderr)
        sys.exit(1)

    if args.bag and not use_batch:
        if not args.left and not args.right:
            print("Loading one frame from bag...")
            left, right, middle = load_from_bag(
                args.bag, args.topic_left, args.topic_right, args.topic_middle if not args.no_trifocal else None
            )
            print(f"  Left: {left.shape}, Right: {right.shape}, Middle: {middle.shape if middle is not None else 'N/A'}")
        else:
            print("Use either --bag or --left/--right, not both.", file=sys.stderr)
            sys.exit(1)
    elif use_batch:
        # Batch mode: run on all frames and save (no single-frame load here)
        run_batch(args, sync_window_ns)
        return
    else:
        if not args.left or not args.right:
            print("Provide --left and --right image paths, or --bag with topics.", file=sys.stderr)
            sys.exit(1)
        left = load_image(args.left)
        right = load_image(args.right)
        middle = load_image(args.middle) if args.middle else None

    if left.shape[:2] != right.shape[:2]:
        print("Left and right images must have the same size.", file=sys.stderr)
        sys.exit(1)
    if middle is not None and left.shape[:2] != middle.shape[:2]:
        print("Middle image size should match left/right for WLS; resizing middle to left size.")
        middle = cv2.resize(middle, (left.shape[1], left.shape[0]), interpolation=cv2.INTER_LINEAR)

    out_dir = Path(args.out_dir) if args.out_dir else None
    if out_dir:
        out_dir.mkdir(parents=True, exist_ok=True)

    block_size = args.block_size
    num_disp = max(16, (args.num_disparities + 15) // 16 * 16)

    # ----- 1) SGBM -----
    print("Running SGBM (block_size=%d, num_disparities=%d)..." % (block_size, num_disp))
    disp_sgbm = compute_sgbm(left, right, block_size=block_size, num_disp=num_disp)
    vis_sgbm = disparity_to_vis(disp_sgbm)
    print("  Done.")

    # ----- 2) WLS -----
    disp_wls = None
    if not args.no_wls:
        try:
            print("Running WLS filter (guide = left image)...")
            disp_right = compute_sgbm_right(left, right, block_size=block_size, num_disp=num_disp)
            disp_wls = wls_filter(disp_sgbm, left, disp_right=disp_right, num_disparities=num_disp)
            vis_wls = disparity_to_vis(disp_wls)
            print("  Done.")
        except ImportError as e:
            print("  WLS skipped:", e)

    # ----- 3) Trifocal -----
    mask_trifocal = None
    if middle is not None and not args.no_trifocal:
        Q = None
        P_middle = None
        if args.calibration:
            try:
                import yaml
                with open(args.calibration) as f:
                    cal = yaml.safe_load(f)
                if cal and "Q" in cal and "P_middle" in cal:
                    Q = np.array(cal["Q"], dtype=np.float64)
                    P_middle = np.array(cal["P_middle"], dtype=np.float64)
            except Exception as e:
                print(f"  Calibration load failed: {e}")
        if Q is not None and P_middle is not None:
            print("Running trifocal consistency check...")
            mask_trifocal = trifocal_consistency_mask(
                disp_wls if disp_wls is not None else disp_sgbm,
                left, middle, Q, P_middle,
            )
            print("  Done.")
        else:
            print("Trifocal skipped (no --calibration with Q and P_middle).")

    # ----- Show and save -----
    def show(name: str, img: np.ndarray, save_path: Path | None = None):
        cv2.imshow(name, img)
        if save_path:
            cv2.imwrite(str(save_path), img)

    scale = 1.0
    if max(left.shape[0], left.shape[1]) > 900:
        scale = 900 / max(left.shape[0], left.shape[1])
    def resize_for_display(img):
        if scale == 1.0:
            return img
        w, h = int(img.shape[1] * scale), int(img.shape[0] * scale)
        return cv2.resize(img, (w, h), interpolation=cv2.INTER_AREA)

    show("Left", resize_for_display(left), out_dir / "left.png" if out_dir else None)
    show("SGBM disparity", resize_for_display(vis_sgbm), out_dir / "disparity_sgbm.png" if out_dir else None)
    if disp_wls is not None:
        show("WLS disparity", resize_for_display(vis_wls), out_dir / "disparity_wls.png" if out_dir else None)
    if mask_trifocal is not None:
        show("Trifocal mask", resize_for_display(cv2.cvtColor(mask_trifocal, cv2.COLOR_GRAY2BGR)), out_dir / "trifocal_mask.png" if out_dir else None)
        # Optionally show SGBM or WLS masked by trifocal
        disp_used = disp_wls if disp_wls is not None else disp_sgbm
        disp_masked = disp_used.copy()
        disp_masked[mask_trifocal == 0] = 0
        vis_masked = disparity_to_vis(disp_masked)
        show("Disparity (trifocal validated)", resize_for_display(vis_masked), out_dir / "disparity_trifocal.png" if out_dir else None)
    if middle is not None:
        show("Middle", resize_for_display(middle), out_dir / "middle.png" if out_dir else None)

    print("Close windows or press any key to exit.")
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    print("Done.")


if __name__ == "__main__":
    main()
