import os
import time
import math
import threading

import cv2
import numpy as np
import mediapipe as mp

from mediapipe.tasks import python
from mediapipe.tasks.python import vision


MODEL_PATH = "hand_landmarker.task"
CAMERA_INDEX = 0

# 如果你发现镜像显示下左右手标签和你的直觉相反，
# 把这个改成 "Right" 即可。
TARGET_HAND = "Left"

# 只在这个手势下开始画线
DRAW_GESTURE = "POINT"
STOP_GESTURE = "FIST"

# 21 个手部关键点连接关系
HAND_CONNECTIONS = [
    (0, 1), (1, 2), (2, 3), (3, 4),          # thumb
    (0, 5), (5, 6), (6, 7), (7, 8),          # index
    (5, 9), (9, 10), (10, 11), (11, 12),     # middle
    (9, 13), (13, 14), (14, 15), (15, 16),   # ring
    (13, 17), (17, 18), (18, 19), (19, 20),  # pinky
    (0, 17)                                  # palm edge
]

# -----------------------------
# Shared state updated by callback
# -----------------------------
latest_result = None
latest_timestamp_ms = -1
result_lock = threading.Lock()


def result_callback(result, output_image, timestamp_ms):
    global latest_result, latest_timestamp_ms
    with result_lock:
        latest_result = result
        latest_timestamp_ms = timestamp_ms


def dist(lm1, lm2):
    return math.hypot(lm1.x - lm2.x, lm1.y - lm2.y)


def finger_extended(landmarks, tip_idx, pip_idx):
    # Image coordinates: y increases downward
    return landmarks[tip_idx].y < landmarks[pip_idx].y


def classify_gesture(landmarks):
    """
    Simple heuristic gesture classifier based on 21 hand landmarks.
    """
    wrist = landmarks[0]

    thumb_extended = dist(landmarks[4], wrist) > dist(landmarks[3], wrist) * 1.08
    index_extended = finger_extended(landmarks, 8, 6)
    middle_extended = finger_extended(landmarks, 12, 10)
    ring_extended = finger_extended(landmarks, 16, 14)
    pinky_extended = finger_extended(landmarks, 20, 18)

    thumb_up = thumb_extended and landmarks[4].y < landmarks[0].y and landmarks[4].y < landmarks[3].y
    thumb_down = thumb_extended and landmarks[4].y > landmarks[0].y and landmarks[4].y > landmarks[3].y

    all_extended = thumb_extended and index_extended and middle_extended and ring_extended and pinky_extended
    all_curled = (not thumb_extended) and (not index_extended) and (not middle_extended) and (not ring_extended) and (not pinky_extended)

    if all_extended:
        label = "OPEN_PALM"
    elif all_curled:
        label = "FIST"
    elif thumb_up and (not index_extended) and (not middle_extended) and (not ring_extended) and (not pinky_extended):
        label = "THUMB_UP"
    elif thumb_down and (not index_extended) and (not middle_extended) and (not ring_extended) and (not pinky_extended):
        label = "THUMB_DOWN"
    elif index_extended and middle_extended and (not ring_extended) and (not pinky_extended):
        label = "PEACE"
    elif index_extended and (not middle_extended) and (not ring_extended) and (not pinky_extended):
        label = "POINT"
    else:
        label = "UNKNOWN"

    finger_state = f"T:{int(thumb_extended)} I:{int(index_extended)} M:{int(middle_extended)} R:{int(ring_extended)} P:{int(pinky_extended)}"
    return label, finger_state


def draw_landmarks(frame, hand_landmarks, handedness_label, gesture_label):
    """
    Draw 21 keypoints and connections on the frame.
    """
    h, w = frame.shape[:2]

    for lm in hand_landmarks:
        cx = int(lm.x * w)
        cy = int(lm.y * h)
        cv2.circle(frame, (cx, cy), 4, (0, 255, 0), -1)
        cv2.circle(frame, (cx, cy), 7, (0, 0, 0), 1)

    for start_idx, end_idx in HAND_CONNECTIONS:
        x1 = int(hand_landmarks[start_idx].x * w)
        y1 = int(hand_landmarks[start_idx].y * h)
        x2 = int(hand_landmarks[end_idx].x * w)
        y2 = int(hand_landmarks[end_idx].y * h)
        cv2.line(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)

    wx = int(hand_landmarks[0].x * w)
    wy = int(hand_landmarks[0].y * h)
    text = f"{handedness_label}: {gesture_label}"
    cv2.putText(frame, text, (wx + 10, max(20, wy - 10)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2, cv2.LINE_AA)


def main():
    if not os.path.exists(MODEL_PATH):
        raise FileNotFoundError(
            f"Missing model file: {MODEL_PATH}\n"
            f"Download the official Hand Landmarker task model and place it next to this script."
        )

    BaseOptions = python.BaseOptions
    HandLandmarkerOptions = vision.HandLandmarkerOptions
    RunningMode = vision.RunningMode
    HandLandmarker = vision.HandLandmarker

    options = HandLandmarkerOptions(
        base_options=BaseOptions(model_asset_path=MODEL_PATH),
        running_mode=RunningMode.LIVE_STREAM,
        num_hands=2,
        min_hand_detection_confidence=0.5,
        min_hand_presence_confidence=0.5,
        min_tracking_confidence=0.5,
        result_callback=result_callback,
    )

    landmarker = HandLandmarker.create_from_options(options)

    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open camera index {CAMERA_INDEX}")

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    trail_canvas = None
    last_draw_point = None
    last_print_signature = None
    last_print_time = 0.0

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("Failed to read frame from camera.")
                break

            frame = cv2.flip(frame, 1)

            if trail_canvas is None or trail_canvas.shape != frame.shape:
                trail_canvas = np.zeros_like(frame)

            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            timestamp_ms = int(time.time() * 1000)
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
            landmarker.detect_async(mp_image, timestamp_ms)

            with result_lock:
                result = latest_result

            left_hand_point_active = False

            if result is not None and result.hand_landmarks:
                for i, hand_landmarks in enumerate(result.hand_landmarks):
                    handedness_label = "Unknown"
                    if result.handedness and i < len(result.handedness) and len(result.handedness[i]) > 0:
                        handedness_label = result.handedness[i][0].category_name

                    gesture_label, finger_state = classify_gesture(hand_landmarks)
                    draw_landmarks(frame, hand_landmarks, handedness_label, gesture_label)

                    # 只对左手做画笔逻辑
                    if handedness_label == TARGET_HAND:
                        if gesture_label == DRAW_GESTURE:
                            h, w = frame.shape[:2]
                            tip = hand_landmarks[8]  # index fingertip
                            tip_pt = (int(tip.x * w), int(tip.y * h))

                            if last_draw_point is not None:
                                cv2.line(trail_canvas, last_draw_point, tip_pt, (0, 255, 255), 3)

                            last_draw_point = tip_pt
                            left_hand_point_active = True

                        elif gesture_label == STOP_GESTURE:
                            last_draw_point = None

                        else:
                            last_draw_point = None

                    signature = (i, handedness_label, gesture_label, finger_state)
                    now = time.time()
                    if signature != last_print_signature or (now - last_print_time) > 0.5:
                        print(f"Hand {i} [{handedness_label}] -> {gesture_label} | {finger_state}")
                        last_print_signature = signature
                        last_print_time = now

            if not left_hand_point_active and result is None:
                last_draw_point = None

            composed = cv2.addWeighted(frame, 1.0, trail_canvas, 0.8, 0.0)

            cv2.putText(
                composed,
                "Left hand POINT to draw. FIST to stop.",
                (20, 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.9,
                (255, 255, 255),
                2,
                cv2.LINE_AA
            )
            cv2.putText(
                composed,
                "Press Q to quit",
                (20, 70),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (255, 255, 255),
                2,
                cv2.LINE_AA
            )

            cv2.imshow("Hand Gesture Demo", composed)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:
                break

    finally:
        cap.release()
        landmarker.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()