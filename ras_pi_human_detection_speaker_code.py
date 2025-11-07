import argparse
import sys
from functools import lru_cache
 
import cv2
import numpy as np
 
from picamera2 import MappedArray, Picamera2
from picamera2.devices import IMX500
from picamera2.devices.imx500 import (NetworkIntrinsics,
                                      postprocess_nanodet_detection)
 
# --- NEW: time + subprocess + random
import time
import subprocess
import random
from collections import deque  # --- NEW
 
last_detections = []
DEBUG_TEST = ""     ### NEW FOR TESTING ###
 
# ====== SOUND CONFIG ======
APLAY_DEV = 'plughw:CARD=Headphones,DEV=0'   # --- CHANGED (stable across HDMI/no-HDMI)
SOUND_PATH = '/home/pi/sounds/gtts_download_hello.wav' # greeting file path
 
SAY_COOLDOWN_S = 3.0
_last_sound_time = 0.0
 
def make_sound():
    """Play a short WAV through the analog jack."""
    subprocess.Popen([
        "bash", "-lc",
        f'aplay -q -D "{APLAY_DEV}" "{SOUND_PATH}"'
    ])
# ==========================
 
class Detection:
    def __init__(self, coords, category, conf, metadata):
        """Create a Detection object, recording the bounding box, category and confidence."""
        self.category = category
        self.conf = conf
        self.box = imx500.convert_inference_coords(coords, metadata, picam2)
 
def parse_detections(metadata: dict):
    """Parse the output tensor into a number of detected objects, scaled to the ISP output."""
    global last_detections
    bbox_normalization = intrinsics.bbox_normalization
    bbox_order = intrinsics.bbox_order
    threshold = args.threshold
    iou = args.iou
    max_detections = args.max_detections
 
    np_outputs = imx500.get_outputs(metadata, add_batch=True)
    input_w, input_h = imx500.get_input_size()
    if np_outputs is None:
        return last_detections
    if intrinsics.postprocess == "nanodet":
        boxes, scores, classes = \
            postprocess_nanodet_detection(outputs=np_outputs[0], conf=threshold, iou_thres=iou,
                                          max_out_dets=max_detections)[0]
        from picamera2.devices.imx500.postprocess import scale_boxes
        boxes = scale_boxes(boxes, 1, 1, input_h, input_w, False, False)
    else:
        boxes, scores, classes = np_outputs[0][0], np_outputs[1][0], np_outputs[2][0]
        if bbox_normalization:
            boxes = boxes / input_h
        if bbox_order == "xy":
            boxes = boxes[:, [1, 0, 3, 2]]
        boxes = np.array_split(boxes, 4, axis=1)
        boxes = zip(*boxes)
 
    last_detections = [
        Detection(box, category, score, metadata)
        for box, score, category in zip(boxes, scores, classes)
        if score > threshold
    ]
    return last_detections
 
@lru_cache
def get_labels():
    labels = intrinsics.labels
    if intrinsics.ignore_dash_labels:
        labels = [label for label in labels if label and label != "-"]
    return labels
 
# (kept for preview builds; we’re headless now)
def draw_detections(request, stream="main"):
    detections = last_results
    if detections is None:
        return
    labels = get_labels()
    with MappedArray(request, stream) as m:
        for detection in detections:
            x, y, w, h = detection.box
            label = f"{labels[int(detection.category)]} ({detection.conf:.2f})"
            (tw, th), baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            text_x, text_y = x + 5, y + 15
            overlay = m.array.copy()
            cv2.rectangle(overlay, (text_x, text_y - th), (text_x + tw, text_y + baseline), (255,255,255), cv2.FILLED)
            cv2.addWeighted(overlay, 0.30, m.array, 0.70, 0, m.array)
            cv2.putText(m.array, label, (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1)
            cv2.rectangle(m.array, (x, y), (x + w, y + h), (0,255,0,0), thickness=2)
           
            ### NEW FOR TESTING ###
            global W, H
            area_frac = (w * h) / float(W * H)
            cv2.putText(m.array, f"conf={float(detection.conf):.2f} area={area_frac*100:.1f}%", (x, max(0, y-6)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,255), 1, cv2.LINE_AA)
            ### NEW FOR TESTING ###
 
        if intrinsics.preserve_aspect_ratio:
            b_x, b_y, b_w, b_h = imx500.get_roi_scaled(request)
            color = (255, 0, 0)
            cv2.putText(m.array, "ROI", (b_x + 5, b_y + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
            cv2.rectangle(m.array, (b_x, b_y), (b_x + b_w, b_y + b_h), (255,0,0,0))
 
def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", type=str,
                        default="/usr/share/imx500-models/imx500_network_ssd_mobilenetv2_fpnlite_320x320_pp.rpk")
    parser.add_argument("--fps", type=int, help="Frames per second")
    parser.add_argument("--bbox-normalization", action=argparse.BooleanOptionalAction)
    parser.add_argument("--bbox-order", choices=["yx", "xy"], default="yx")
    parser.add_argument("--threshold", type=float, default=0.55)
    parser.add_argument("--iou", type=float, default=0.65)
    parser.add_argument("--max-detections", type=int, default=10)
    parser.add_argument("--ignore-dash-labels", action=argparse.BooleanOptionalAction)
    parser.add_argument("--postprocess", choices=["", "nanodet"], default=None)
    parser.add_argument("-r", "--preserve-aspect-ratio", action=argparse.BooleanOptionalAction)
    parser.add_argument("--labels", type=str)
    return parser.parse_args()
 
# ====== NEAR/FAR FILTER CONFIG ======
# CONF_THRESH_NEAR = 0.70        # higher than default to reduce false positives  # --- NEW
# PERSIST_FRAMES   = 3           # need N consecutive frames                     # --- NEW
# MIN_AREA_FRAC    = 0.06        # >= 6% of frame area ⇒ “near-ish”             # --- NEW
# MAX_AREA_FRAC    = 0.80        # ignore absurdly large boxes                   # --- NEW
# IGNORE_TOP_FRAC  = 0.15        # ignore top band (far/doors)                   # --- NEW
# IGNORE_BOTTOM_FRAC = 0.05      # small bottom band ignore if needed            # --- NEW
 
CONF_THRESH_NEAR = 0.55        # higher than default to reduce false positives  # --- NEW
PERSIST_FRAMES   = 2           # need N consecutive frames                     # --- NEW
MIN_AREA_FRAC    = 0.10        # >= 6% of frame area ⇒ “near-ish”             # --- NEW
MAX_AREA_FRAC    = 0.80        # ignore absurdly large boxes                   # --- NEW
IGNORE_TOP_FRAC  = 0.05        # ignore top band (far/doors)                   # --- NEW
IGNORE_BOTTOM_FRAC = 0.01
 
_presence = deque(maxlen=PERSIST_FRAMES)  # rolling history                     # --- NEW
# =====================================
 
if __name__ == "__main__":
    args = get_args()
 
    # Must be called before Picamera2
    imx500 = IMX500(args.model)
    intrinsics = imx500.network_intrinsics
    if not intrinsics:
        intrinsics = NetworkIntrinsics()
        intrinsics.task = "object detection"
    elif intrinsics.task != "object detection":
        print("Network is not an object detection task", file=sys.stderr)
        exit()
 
    # Override intrinsics from args
    for key, value in vars(args).items():
        if key == 'labels' and value is not None:
            with open(value, 'r') as f:
                intrinsics.labels = f.read().splitlines()
        elif hasattr(intrinsics, key) and value is not None:
            setattr(intrinsics, key, value)
 
    # Defaults
    if intrinsics.labels is None:
        with open("assets/coco_labels.txt", "r") as f:
            intrinsics.labels = f.read().splitlines()
    intrinsics.update_with_defaults()
 
    # if args.print_intrinsics:
        # print(intrinsics)
        # exit()
    if getattr(args, "print_intrinsics", False):
        print(intrinsics)
        sys.exit(0)
 
    picam2 = Picamera2(imx500.camera_num)
    config = picam2.create_preview_configuration(controls={"FrameRate": intrinsics.inference_rate}, buffer_count=12)
 
    imx500.show_network_fw_progress_bar()
    # picam2.start(config, show_preview=True)
    # Headless over SSH
    picam2.start(config, show_preview=False)   # --- CHANGED
 
    if intrinsics.preserve_aspect_ratio:
        imx500.set_auto_aspect_ratio()
 
    last_results = None
    # picam2.pre_callback = draw_detections  # keep disabled when headless
 
    # cache labels
    _labels = None
    try:
        _labels = get_labels()
    except Exception:
        _labels = None
 
    # Get ISP output size from config for area fraction (fallback to 640x480)
    try:
        W = config["main"]["size"][0]   # --- NEW
        H = config["main"]["size"][1]
    except Exception:
        W, H = 640, 480                 # --- NEW
 
    while True:
        last_results = parse_detections(picam2.capture_metadata())
 
        saw_person_this_frame = False   # --- NEW
        if last_results:
            labels = _labels or get_labels()
 
            for d in last_results:
                cat = int(d.category)
                if not (0 <= cat < len(labels)):
                    continue
                if str(labels[cat]).lower() != "person":
                    continue
                if float(d.conf) < CONF_THRESH_NEAR:
                    continue
 
                x, y, w, h = d.box
                # ROI gating (ignore distant/top & tiny bottom)
                if y < int(H * IGNORE_TOP_FRAC):
                    continue
                if (y + h) > int(H * (1 - IGNORE_BOTTOM_FRAC)):
                    continue
 
                # Size gating as distance proxy
                area_frac = (w * h) / float(W * H)
                if area_frac < MIN_AREA_FRAC or area_frac > MAX_AREA_FRAC:
                    continue
 
                # Passed all filters → “near” this frame
                saw_person_this_frame = True
                break
 
        # persistence across frames
        _presence.append(saw_person_this_frame)          # --- NEW
        saw_person_persistent = all(_presence)           # --- NEW
 
        now = time.monotonic()
        if saw_person_persistent and (now - _last_sound_time) >= SAY_COOLDOWN_S:
            make_sound()
            _last_sound_time = now
 