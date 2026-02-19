#!/usr/bin/env python3
"""
Visualize any image topic from a ROS2 bag. Single-channel as heatmap; multi-channel as RGB or per-channel heatmap.
Channel is chosen via --channel (0|1|2|all|auto).

Usage:
  python visualize_image_bag.py <path_to_bag> [--topic TOPIC] [--channel 0|1|2|all|auto]
  python visualize_image_bag.py ./my_bag --topic /camera/rgb/image_raw --channel auto

Controls: [Space] pause | [A]/[D] or arrows step | [Q] quit
Requirements: rosbag2_py, sensor_msgs, numpy, opencv-python, matplotlib (for colormaps)
"""

import argparse
import sys
import time
from pathlib import Path

import numpy as np
import cv2


def get_reader_and_types(bag_path: str):
    """Open rosbag2 and return reader plus topic name -> type name mapping."""
    try:
        import rosbag2_py
    except ImportError:
        print("rosbag2_py not found. Do BOTH of the following:", file=sys.stderr)
        print("  1. Source ROS 2: source /opt/ros/jazzy/setup.bash", file=sys.stderr)
        print("  2. Install: sudo apt install ros-jazzy-rosbag2-py", file=sys.stderr)
        sys.exit(1)

    bag_path = Path(bag_path)
    if not bag_path.exists():
        print(f"Bag path does not exist: {bag_path}", file=sys.stderr)
        sys.exit(1)
    uri = str(bag_path.resolve())
    storage_options = rosbag2_py.StorageOptions(uri=uri, storage_id="")
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="",
        output_serialization_format="",
    )
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    topic_types = {}
    try:
        for topic_info in reader.get_all_topics_and_types():
            name = getattr(topic_info, "name", None)
            typ = getattr(topic_info, "type", "sensor_msgs/msg/Image")
            if name is not None:
                topic_types[name] = typ
    except Exception:
        pass
    return reader, topic_types


def image_msg_to_array(msg):
    """
    Convert sensor_msgs/Image to (array, num_channels, is_float).
    array: (H, W) or (H, W, C); float32 for depth-like, uint8 for 8-bit.
    """
    try:
        from sensor_msgs.msg import Image
    except ImportError:
        print("Source ROS2: source /opt/ros/jazzy/setup.bash", file=sys.stderr)
        sys.exit(1)
    if not isinstance(msg, Image):
        return None, 0, False
    h, w = msg.height, msg.width
    enc = msg.encoding.upper().replace(" ", "")
    if enc in ("8UC1", "8U_C1", "MONO8"):
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w)
        return arr, 1, False
    if enc in ("8UC3", "8U_C3", "RGB8", "BGR8"):
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3)
        if "BGR" in msg.encoding:
            arr = cv2.cvtColor(arr, cv2.COLOR_BGR2RGB)
        return arr, 3, False
    if enc in ("16UC1", "16U_C1"):
        arr = np.frombuffer(msg.data, dtype=np.uint16).reshape(h, w).astype(np.float32)
        return arr, 1, True
    if enc in ("32FC1", "32F_C1"):
        arr = np.frombuffer(msg.data, dtype=np.float32).reshape(h, w).copy()
        return arr, 1, True
    return None, 0, False


def channel_to_heatmap(ch: np.ndarray, min_val: float, max_val: float, colormap_name: str = "turbo", add_legend: bool = True) -> np.ndarray:
    """Map single-channel float/uint to BGR heatmap with optional legend strip."""
    ch = np.asarray(ch, dtype=np.float64)
    valid = np.isfinite(ch)
    if not np.any(valid):
        out = np.zeros((*ch.shape, 3), dtype=np.uint8)
        if add_legend:
            out = _add_legend_strip(out, min_val, max_val, colormap_name)
        return out
    out = np.zeros((*ch.shape, 3), dtype=np.uint8)
    normalized = np.clip((ch - min_val) / (max_val - min_val) if max_val > min_val else np.zeros_like(ch), 0, 1)
    normalized[~valid] = 0
    try:
        import matplotlib.pyplot as plt
        cmap = plt.get_cmap(colormap_name)
    except Exception:
        cmap = None
    if cmap is not None:
        rgba = (cmap(normalized) * 255).astype(np.uint8)[:, :, :3]
        out = cv2.cvtColor(rgba, cv2.COLOR_RGB2BGR)
    else:
        gray = (normalized * 255).astype(np.uint8)
        out = cv2.applyColorMap(gray, cv2.COLORMAP_TURBO)
    if add_legend:
        out = _add_legend_strip(out, min_val, max_val, colormap_name)
    return out


def _add_legend_strip(img: np.ndarray, min_val: float, max_val: float, colormap_name: str, strip_height: int = 24) -> np.ndarray:
    h, w = img.shape[:2]
    try:
        import matplotlib.pyplot as plt
        cmap = plt.get_cmap(colormap_name)
    except Exception:
        cmap = None
    if cmap is None:
        bar_bgr = np.tile(np.linspace(0, 255, w).astype(np.uint8).reshape(1, -1), (strip_height, 1))
        bar_bgr = cv2.applyColorMap(bar_bgr, cv2.COLORMAP_TURBO)
    else:
        x = np.linspace(0, 1, w)
        rgba = (cmap(x) * 255).astype(np.uint8)[:, :3]
        bar_rgb = rgba[np.newaxis, :, :]
        bar_bgr = cv2.cvtColor(bar_rgb, cv2.COLOR_RGB2BGR)
        strip = np.repeat(bar_bgr, strip_height, axis=0)
        bar_bgr = strip
    out = np.vstack([img, bar_bgr])
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(out, f"{min_val:.2g}", (4, h + strip_height - 6), font, 0.4, (255, 255, 255), 1)
    cv2.putText(out, f"{max_val:.2g}", (w - 50, h + strip_height - 6), font, 0.4, (255, 255, 255), 1)
    return out


IMAGE_TOPIC_KEYWORDS = ("rgb", "left", "right", "stereo")


def _is_compressed_topic(topic_name: str, type_str: str) -> bool:
    """CompressedImage types / compressed-named topics cannot be deserialized as Image; skip them."""
    return "CompressedImage" in (type_str or "") or "compressed" in (topic_name or "").lower()


def get_image_topic_candidates(topic_types: dict) -> list:
    """Return list of uncompressed Image topic names (no CompressedImage)."""
    return [
        t for t, typ in topic_types.items()
        if "Image" in typ and not _is_compressed_topic(t, typ)
    ]


def find_image_topic(topic_types: dict, hint: str) -> str:
    image_topics = get_image_topic_candidates(topic_types)
    if (hint or "").strip():
        if hint in topic_types and not _is_compressed_topic(hint, topic_types.get(hint, "")):
            return hint
        for t in topic_types:
            if hint in t and not _is_compressed_topic(t, topic_types.get(t, "")):
                return t
    for kw in IMAGE_TOPIC_KEYWORDS:
        for t in image_topics:
            if kw in t.lower():
                return t
    return image_topics[0] if image_topics else ""


def prompt_choose_topic(candidates: list) -> str:
    """Prompt user to choose a topic by number or by full name. Returns chosen topic name."""
    if not candidates:
        return ""
    if len(candidates) == 1:
        print(f"Only one image topic: {candidates[0]}")
        return candidates[0]
    print("Image topics (compressed excluded):")
    for i, name in enumerate(candidates, 1):
        print(f"  {i}. {name}")
    while True:
        try:
            raw = input("Choose topic (number or full name): ").strip()
            if not raw:
                print("Enter a number or topic name.", file=sys.stderr)
                continue
            # Try as number first
            n = int(raw)
            if 1 <= n <= len(candidates):
                return candidates[n - 1]
            print(f"Enter a number between 1 and {len(candidates)}.", file=sys.stderr)
        except ValueError:
            # Treat as topic name (full or partial)
            for t in candidates:
                if raw == t or raw in t:
                    return t
            print(f"No topic matching '{raw}'. Choose from the list above.", file=sys.stderr)


def main():
    parser = argparse.ArgumentParser(description="Visualize any image topic from a ROS2 bag.")
    parser.add_argument("bag_path", type=str, help="Path to the ROS2 bag")
    parser.add_argument("--topic", "-t", type=str, default=None, help="Image topic (if not set, prompt to choose)")
    parser.add_argument("--channel", type=str, default="auto",
                        help="Channel to show: 0, 1, 2, 'all', or 'auto' (single->heatmap, multi->all)")
    parser.add_argument("--min-val", type=float, default=0.0, help="Min value for heatmap (single-channel)")
    parser.add_argument("--max-val", type=float, default=5.0, help="Max value for heatmap (e.g. depth in m)")
    parser.add_argument("--colormap", "-c", type=str, default="turbo")
    parser.add_argument("--max-frames", type=int, default=0, help="Max frames to load (0 = all)")
    parser.add_argument("--fps", type=float, default=30.0, help="Playback fps when not paused")
    args = parser.parse_args()

    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Image

    reader, topic_types = get_reader_and_types(args.bag_path)
    candidates = get_image_topic_candidates(topic_types)
    if not candidates:
        print("No Image topic found (compressed excluded). Available:", list(topic_types.keys()), file=sys.stderr)
        print("Use --topic /your/image_topic if the topic type is different.", file=sys.stderr)
        sys.exit(1)

    if (args.topic or "").strip():
        hint = (args.topic or "").strip()
        if hint in candidates:
            image_topic = hint
        else:
            match = [t for t in candidates if hint in t]
            image_topic = match[0] if len(match) == 1 else (match[0] if match else "")
        if not image_topic:
            print(f"No image topic matching '{hint}'. Choose from: {candidates}", file=sys.stderr)
            sys.exit(1)
        print(f"Topic: {image_topic}")
    else:
        image_topic = prompt_choose_topic(candidates)
        if not image_topic:
            sys.exit(1)
        print(f"Topic: {image_topic}")

    try:
        reader.set_filter(rosbag2_py.StorageFilter(topics=[image_topic]))
    except Exception:
        pass

    frames = []
    failed_indices = []
    msg_index = -1
    while reader.has_next():
        rec = reader.read_next()
        if rec[0] != image_topic:
            continue
        msg_index += 1
        try:
            msg = deserialize_message(rec[1], Image)
        except Exception as e:
            failed_indices.append(msg_index)
            if len(failed_indices) <= 5:
                print(f"Warning: skipped message index {msg_index} (deserialize failed): {e}", file=sys.stderr)
            continue
        arr, num_ch, is_float = image_msg_to_array(msg)
        if arr is None:
            continue
        frames.append((arr, num_ch, is_float))
        if args.max_frames and len(frames) >= args.max_frames:
            break
    if failed_indices:
        if len(failed_indices) <= 20:
            print(f"Deserialization failed for message indices on '{image_topic}': {failed_indices}", file=sys.stderr)
        else:
            print(f"Deserialization failed for {len(failed_indices)} messages (indices: {failed_indices[:15]} ... {failed_indices[-3:]})", file=sys.stderr)

    if not frames:
        print("No image messages found on that topic.", file=sys.stderr)
        sys.exit(1)
    print(f"Loaded {len(frames)} frames. [Space] pause | [A]/[D] step | [Q] quit")

    _, num_ch, _ = frames[0]
    channel_mode = args.channel.lower()
    if channel_mode == "auto":
        channel_mode = "all" if num_ch >= 3 else "0"
    use_heatmap = (num_ch == 1) or (channel_mode in ("0", "1", "2") and num_ch >= 3)
    channel_idx = 0
    if channel_mode in ("0", "1", "2"):
        channel_idx = int(channel_mode)

    frame_interval = 1.0 / args.fps if args.fps > 0 else 1.0 / 30.0
    idx = 0
    paused = False
    last_frame_time = time.monotonic()
    win_name = "Image (Space=pause, A/D=step, Q=quit)"

    while True:
        arr, num_ch, is_float = frames[idx]

        if use_heatmap:
            if num_ch >= 3:
                ch = arr[:, :, channel_idx].astype(np.float64)
            else:
                ch = arr.astype(np.float64) if is_float else arr.astype(np.float64)
            display = channel_to_heatmap(ch, args.min_val, args.max_val, args.colormap)
        else:
            display = arr if arr.ndim == 3 else cv2.cvtColor(arr, cv2.COLOR_GRAY2BGR)
            if display.dtype != np.uint8:
                display = (np.clip(display, 0, 1) * 255).astype(np.uint8)

        h, w = display.shape[:2]
        if max(h, w) > 1200:
            scale = 1200 / max(h, w)
            display = cv2.resize(display, (int(w * scale), int(h * scale)), interpolation=cv2.INTER_AREA)
        status = f"Frame {idx+1}/{len(frames)}" + (" [PAUSED]" if paused else "")
        cv2.putText(display, status, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.imshow(win_name, display)

        key = cv2.waitKey(1)
        if key != -1:
            key = key & 0xFF
            if key == ord("q"):
                break
            if key == ord(" "):
                paused = not paused
            if key == ord("a") or key == 81 or key == 2:
                idx = max(0, idx - 1)
            if key == ord("d") or key == 83 or key == 3:
                idx = min(len(frames) - 1, idx + 1)

        if not paused:
            now = time.monotonic()
            if now - last_frame_time >= frame_interval:
                idx = (idx + 1) % len(frames)
                last_frame_time = now

    cv2.destroyAllWindows()
    print("Done.")


if __name__ == "__main__":
    main()
