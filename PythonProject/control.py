import os
import time
import math
import threading

import cv2
import numpy as np
import mediapipe as mp
import serial
from serial import SerialException

from mediapipe.tasks import python
from mediapipe.tasks.python import vision


MODEL_PATH = "hand_landmarker.task"
CAMERA_INDEX = 0

# 和你的 MATLAB / Arduino 协议保持一致
SERIAL_PORT = "COM5"
SERIAL_BAUD = 38400

# 如果你发现镜像画面里左右手标签和你的直觉相反，就改成 "Right"
TARGET_HAND = "Left"

# 21 个手部关键点连接关系
HAND_CONNECTIONS = [
    (0, 1), (1, 2), (2, 3), (3, 4),          # thumb
    (0, 5), (5, 6), (6, 7), (7, 8),          # index
    (5, 9), (9, 10), (10, 11), (11, 12),     # middle
    (9, 13), (13, 14), (14, 15), (15, 16),   # ring
    (13, 17), (17, 18), (18, 19), (19, 20),  # pinky
    (0, 17)                                  # palm edge
]

WRIST = 0
THUMB_TIP = 4
INDEX_TIP = 8
PINKY_MCP = 17

latest_result = None
latest_timestamp_ms = -1
result_lock = threading.Lock()


def result_callback(result, output_image, timestamp_ms):
    global latest_result, latest_timestamp_ms
    with result_lock:
        latest_result = result
        latest_timestamp_ms = timestamp_ms


def clamp(value, low, high):
    return max(low, min(high, value))


def draw_triangle_marker(frame, pt, color, size=8, thickness=2):
    x, y = pt
    pts = np.array([
        [x, y - size],
        [x - size, y + size],
        [x + size, y + size],
    ], dtype=np.int32)
    cv2.polylines(frame, [pts], isClosed=True, color=color, thickness=thickness)


def draw_landmarks(frame, hand_landmarks, handedness_label):
    h, w = frame.shape[:2]

    for start_idx, end_idx in HAND_CONNECTIONS:
        x1 = int(hand_landmarks[start_idx].x * w)
        y1 = int(hand_landmarks[start_idx].y * h)
        x2 = int(hand_landmarks[end_idx].x * w)
        y2 = int(hand_landmarks[end_idx].y * h)
        cv2.line(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)

    for lm in hand_landmarks:
        cx = int(lm.x * w)
        cy = int(lm.y * h)
        cv2.circle(frame, (cx, cy), 3, (0, 255, 0), -1)

    wrist = (int(hand_landmarks[WRIST].x * w), int(hand_landmarks[WRIST].y * h))
    pinky_mcp = (int(hand_landmarks[PINKY_MCP].x * w), int(hand_landmarks[PINKY_MCP].y * h))
    index_tip = (int(hand_landmarks[INDEX_TIP].x * w), int(hand_landmarks[INDEX_TIP].y * h))
    thumb_tip = (int(hand_landmarks[THUMB_TIP].x * w), int(hand_landmarks[THUMB_TIP].y * h))

    draw_triangle_marker(frame, wrist, (255, 255, 255), size=8, thickness=2)
    draw_triangle_marker(frame, pinky_mcp, (255, 255, 0), size=8, thickness=2)
    draw_triangle_marker(frame, index_tip, (0, 0, 255), size=8, thickness=2)
    draw_triangle_marker(frame, thumb_tip, (0, 255, 0), size=8, thickness=2)

    cv2.putText(frame, "W", (wrist[0] + 6, wrist[1] - 6),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(frame, "P", (pinky_mcp[0] + 6, pinky_mcp[1] - 6),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2, cv2.LINE_AA)
    cv2.putText(frame, "I", (index_tip[0] + 6, index_tip[1] - 6),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2, cv2.LINE_AA)
    cv2.putText(frame, "T", (thumb_tip[0] + 6, thumb_tip[1] - 6),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, cv2.LINE_AA)

    top_left_x = min(wrist[0], pinky_mcp[0], index_tip[0], thumb_tip[0])
    top_left_y = max(20, min(wrist[1], pinky_mcp[1], index_tip[1], thumb_tip[1]) - 20)
    cv2.putText(frame, f"{handedness_label}", (top_left_x, top_left_y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2, cv2.LINE_AA)

    return wrist, pinky_mcp, index_tip, thumb_tip


def map_pinky_to_xyz(frame_shape, pinky_mcp_px, wrist_px, index_tip_px, thumb_tip_px):
    """
    New mapping:

    X:
      based on distance between pinky MCP and wrist
      50 px  -> 22
      100 px -> 0
      => X = (100 - d) * 0.44
      range approx [0, 22]

    Y:
      screen left-right movement
      left  -> negative
      right -> positive
      range approx [-22, 22]

    Z:
      screen up-down movement
      up   -> positive
      down -> negative
      range approx [-22, 22]

    error:
      ratio = (index_tip to thumb_tip distance) / (pinky_mcp to wrist distance)
      ratio 0 -> 0
      ratio 1 -> 55
      => error = clamp(ratio * 55, 0, 55)
    """
    h, w = frame_shape[:2]

    px_x, px_y = pinky_mcp_px
    wrist_x, wrist_y = wrist_px
    index_x, index_y = index_tip_px
    thumb_x, thumb_y = thumb_tip_px

    center_x = w / 2.0
    center_y = h / 2.0

    pinky_wrist_dist = math.hypot(px_x - wrist_x, px_y - wrist_y)
    index_thumb_dist = math.hypot(index_x - thumb_x, index_y - thumb_y)

    # X: 50 px -> 22, 100 px -> 0
    x_val = (100.0 - pinky_wrist_dist) * 0.44
    x_val = clamp(x_val, 0.0, 22.0)

    # Y: left negative, right positive
    y_val = ((px_x - center_x) / (w / 2.0)) * 22.0
    y_val = clamp(y_val, -22.0, 22.0)

    # Z: up positive, down negative
    z_val = ((center_y - px_y) / (h / 2.0)) * 22.0
    z_val = clamp(z_val, -22.0, 22.0)

    # error
    if pinky_wrist_dist < 1e-6:
        ratio = 0.0
    else:
        ratio = index_thumb_dist / pinky_wrist_dist
    error_val = clamp(ratio * 55.0, 0.0, 55.0)

    return x_val, y_val, z_val, error_val, index_thumb_dist, pinky_wrist_dist


def open_serial_port():
    try:
        ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=1, write_timeout=1)
        time.sleep(2.0)
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        print(f"Serial connected: {SERIAL_PORT} @ {SERIAL_BAUD}")
        return ser
    except SerialException as e:
        print(f"Serial disabled (could not open {SERIAL_PORT}): {e}")
        return None


def send_xyz_error(ser, x, y, z, error_val):
    """
    Match MATLAB writeline() style:
    one line, same frame format, same LF terminator.
    """
    if ser is None:
        return

    msg = f"#{x:+06.1f},{y:+06.1f},{z:+06.1f},{error_val:+06.1f}$\n"
    ser.write(msg.encode("utf-8"))
    ser.flush()


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
    ser = open_serial_port()

    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open camera index {CAMERA_INDEX}")

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    last_print_signature = None
    last_print_time = 0.0

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("Failed to read frame from camera.")
                break

            frame = cv2.flip(frame, 1)

            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            timestamp_ms = int(time.monotonic() * 1000)
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
            landmarker.detect_async(mp_image, timestamp_ms)

            with result_lock:
                result = latest_result

            if result is not None and result.hand_landmarks:
                target_found = False

                for i, hand_landmarks in enumerate(result.hand_landmarks):
                    handedness_label = "Unknown"
                    if result.handedness and i < len(result.handedness) and len(result.handedness[i]) > 0:
                        handedness_label = result.handedness[i][0].category_name

                    wrist, pinky_mcp, index_tip, thumb_tip = draw_landmarks(frame, hand_landmarks, handedness_label)

                    if handedness_label == TARGET_HAND:
                        target_found = True

                        h, w = frame.shape[:2]
                        wrist_px = (int(hand_landmarks[WRIST].x * w), int(hand_landmarks[WRIST].y * h))
                        pinky_mcp_px = (int(hand_landmarks[PINKY_MCP].x * w), int(hand_landmarks[PINKY_MCP].y * h))
                        index_tip_px = (int(hand_landmarks[INDEX_TIP].x * w), int(hand_landmarks[INDEX_TIP].y * h))
                        thumb_tip_px = (int(hand_landmarks[THUMB_TIP].x * w), int(hand_landmarks[THUMB_TIP].y * h))

                        x_val, y_val, z_val, error_val, index_thumb_dist, pinky_wrist_dist = map_pinky_to_xyz(
                            frame.shape, pinky_mcp_px, wrist_px, index_tip_px, thumb_tip_px
                        )

                        signature = (
                            handedness_label,
                            round(x_val, 2),
                            round(y_val, 2),
                            round(z_val, 2),
                            round(error_val, 2),
                            round(index_thumb_dist, 2),
                            round(pinky_wrist_dist, 2),
                        )

                        now = time.time()
                        if signature != last_print_signature or (now - last_print_time) > 0.1:
                            print(
                                f"[{handedness_label}] "
                                f"X={x_val:.2f}, Y={y_val:.2f}, Z={z_val:.2f}, error={error_val:.2f} | "
                                f"thumb-index={index_thumb_dist:.2f}px, pinky-wrist={pinky_wrist_dist:.2f}px"
                            )
                            last_print_signature = signature
                            last_print_time = now

                        send_xyz_error(ser, x_val, y_val, z_val, error_val)

                        overlay1 = f"X={x_val:.2f}  Y={y_val:.2f}  Z={z_val:.2f}  error={error_val:.2f}"
                        overlay2 = f"thumb-index={index_thumb_dist:.1f}px  pinky-wrist={pinky_wrist_dist:.1f}px"
                        cv2.putText(frame, overlay1, (20, 35),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2, cv2.LINE_AA)
                        cv2.putText(frame, overlay2, (20, 70),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)

                if not target_found:
                    cv2.putText(frame, f"Target hand: {TARGET_HAND} not found", (20, 35),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2, cv2.LINE_AA)

            cv2.putText(
                frame,
                "Press Q to quit",
                (20, frame.shape[0] - 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (255, 255, 255),
                2,
                cv2.LINE_AA
            )

            cv2.imshow("Hand XYZ & Error Demo", frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:
                break

    finally:
        cap.release()
        landmarker.close()
        if ser is not None:
            ser.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()