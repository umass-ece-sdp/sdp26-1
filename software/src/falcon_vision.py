"""
Falcon Vision logic flow:
1. Parse the desired target color and distance, preparing HSV thresholds and target area.
2. Connect to the Tello drone, start the video stream, and load the YOLO11n assistance model.
3. Continuously capture frames, prioritize HSV color masking, and optionally fall back to YOLO for detection (OFF RN).
4. Annotate detections, compute yaw/altitude/distance corrections, and send commands to stay on target.
5. Hover and search when the target is lost, land after a timeout or on manual quit, and always shut down the stream safely.
"""

import time
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple
import cv2
import numpy as np
from djitellopy import Tello
from ultralytics import YOLO

# ----- Parameters -----

TARGET_COLOR: str = "#ff8800"
TARGET_DISTANCE: float = 100.0

FRAME_WIDTH = 960
FRAME_HEIGHT = 720
HSV_TOLERANCE = (12, 90, 110)

REFERENCE_DISTANCE_CM = 80.0
REFERENCE_AREA = 20000.0

YAW_SPEED = 35
FORWARD_BACK_SPEED = 30
UP_DOWN_SPEED = 30
CENTER_DEADBAND = 40
AREA_TOLERANCE_RATIO = 0.12

SEARCH_DURATION = 5.0
SEARCH_YAW_SPEED = 20
FRAME_SLEEP = 0.03

YOLO_MODEL_PATH = "yolo11n.pt"
YOLO_CONFIDENCE = 0.35
YOLO_IMAGE_SIZE = 640

# ----- Utility Functions -----

@dataclass
class Detection:
	"""Represents a tracked object's bounding box and metadata."""

	x1: int
	y1: int
	x2: int
	y2: int
	center_x: int
	center_y: int
	area: float
	source: str


def clamp(value, minimum, maximum):
	"""Clamp ``value`` to the inclusive range ``[minimum, maximum]`` and cast to int."""

	return int(max(minimum, min(maximum, value)))


def to_hsv(color_hex):
	"""Convert a HEX string into a bounded HSV triple (h, s, v)."""

	hex_value = color_hex.lstrip("#")
	if len(hex_value) == 3:
		hex_value = "".join(ch * 2 for ch in hex_value)
	r, g, b = (int(hex_value[i : i + 2], 16) for i in (0, 2, 4))
	rgb_array = np.array([[(r, g, b)]], dtype=np.uint8)
	h, s, v = cv2.cvtColor(rgb_array, cv2.COLOR_RGB2HSV)[0, 0]
	return int(h), int(s), int(v)


def build_hsv_ranges(target_hsv, tolerance):
	"""Create one or two HSV ranges around ``target_hsv`` based on ``tolerance`` for hue wrap-around."""
	h, s, v = target_hsv
	tol_h, tol_s, tol_v = tolerance

	s_lower = clamp(s - tol_s, 0, 255)
	s_upper = clamp(s + tol_s, 0, 255)
	v_lower = clamp(v - tol_v, 0, 255)
	v_upper = clamp(v + tol_v, 0, 255)

	lower_h = h - tol_h
	upper_h = h + tol_h

	ranges = []
	# Hue wrap-around for lower bound
	if lower_h < 0:
		ranges.append((np.array([0, s_lower, v_lower]), np.array([upper_h, s_upper, v_upper])))
		ranges.append((np.array([180 + lower_h, s_lower, v_lower]), np.array([179, s_upper, v_upper])))
	# Hue wrap-around for upper bound
	elif upper_h > 179:
		ranges.append((np.array([lower_h, s_lower, v_lower]), np.array([179, s_upper, v_upper])))
		ranges.append((np.array([0, s_lower, v_lower]), np.array([upper_h - 180, s_upper, v_upper])))
	# No wrap-around
	else:
		ranges.append((np.array([lower_h, s_lower, v_lower]), np.array([upper_h, s_upper, v_upper])))

	return ranges


def detect_with_color(hsv_frame, ranges, min_area=1000.0):
	"""Detect the target via HSV masking and return the resulting detection."""

	mask = np.zeros(hsv_frame.shape[:2], dtype=np.uint8)
	for lower, upper in ranges:
		mask = cv2.bitwise_or(mask, cv2.inRange(hsv_frame, lower, upper))

	contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	if not contours:
		return None

	contour = max(contours, key=cv2.contourArea)
	if cv2.contourArea(contour) < min_area:
		return None

	x, y, w, h = cv2.boundingRect(contour)
	return Detection(x, y, x + w, y + h, x + w // 2, y + h // 2, float(w * h), "color")


def detect_with_yolo(
	frame,
	hsv_frame,
	model,
	target_hsv,
	tolerance,
):
	"""Use ``model`` predictions on ``frame`` and filter candidates near ``target_hsv`` within ``tolerance``."""

	results = model.predict(frame, imgsz=YOLO_IMAGE_SIZE, conf=YOLO_CONFIDENCE, verbose=False)
	if not results:
		return None

	best_detection: Optional[Detection] = None
	hue_goal, sat_goal, val_goal = target_hsv
	tol_h, tol_s, tol_v = tolerance

	for result in results:
		if result.boxes is None:
			continue
		for box in result.boxes:
			xyxy = box.xyxy.cpu().numpy().astype(int)[0]
			x1, y1, x2, y2 = xyxy
			x1 = clamp(x1, 0, frame.shape[1] - 1)
			y1 = clamp(y1, 0, frame.shape[0] - 1)
			x2 = clamp(x2, 0, frame.shape[1] - 1)
			y2 = clamp(y2, 0, frame.shape[0] - 1)
			if x2 <= x1 or y2 <= y1:
				continue

			center_x = int((x1 + x2) / 2)
			center_y = int((y1 + y2) / 2)
			sample_h, sample_s, sample_v = hsv_frame[center_y, center_x]

			hue_diff = min(abs(sample_h - hue_goal), 180 - abs(sample_h - hue_goal))
			if hue_diff > tol_h or abs(sample_s - sat_goal) > tol_s or abs(sample_v - val_goal) > tol_v:
				continue

			area = float((x2 - x1) * (y2 - y1))
			candidate = Detection(int(x1), int(y1), int(x2), int(y2), center_x, center_y, area, "yolo")
			if best_detection is None or candidate.area > best_detection.area:
				best_detection = candidate

	return best_detection


def compute_target_area(distance_cm):
	"""Estimate the target bounding-box area for ``distance_cm`` using inverse-square scaling."""

	ratio = REFERENCE_DISTANCE_CM / max(distance_cm, 1.0)
	target_area = REFERENCE_AREA * (ratio ** 2)
	return target_area


def draw_annotations(frame, detection, target_area, frame_center):
	"""Render bounding-box visuals and telemetry overlays for ``detection`` on ``frame``."""

	x1, y1, x2, y2 = detection.x1, detection.y1, detection.x2, detection.y2
	cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
	cv2.circle(frame, (detection.center_x, detection.center_y), 6, (255, 0, 0), -1)
	cv2.circle(frame, frame_center, 6, (0, 0, 255), -1)
	cv2.putText(
		frame,
		f"Area {int(detection.area)} | Target {int(target_area)}",
		(10, 30),
		cv2.FONT_HERSHEY_SIMPLEX,
		0.7,
		(0, 255, 0),
		2,
	)
	cv2.putText(
		frame,
		f"Offset ({detection.center_x - frame_center[0]}, {frame_center[1] - detection.center_y})",
		(10, 60),
		cv2.FONT_HERSHEY_SIMPLEX,
		0.7,
		(0, 255, 0),
		2,
	)
	cv2.putText(
		frame,
		f"Source: {detection.source}",
		(10, 90),
		cv2.FONT_HERSHEY_SIMPLEX,
		0.7,
		(0, 255, 0),
		2,
	)


def apply_movement(tello, for_back, up_down, yaw, last_command):
	"""Send RC control signals to ``tello`` and log human-readable changes versus ``last_command``."""

	actions = []
	if yaw > 0:
		actions.append("Turning right")
	elif yaw < 0:
		actions.append("Turning left")
	if up_down > 0:
		actions.append("Moving up")
	elif up_down < 0:
		actions.append("Moving down")
	if for_back > 0:
		actions.append("Moving forward")
	elif for_back < 0:
		actions.append("Moving backward")
	if not actions:
		actions.append("Hovering")
	command_description = ", ".join(actions)

	if command_description != last_command:
		print(command_description)
	# Note: No strafing movements in current implementation
	# Question: Gradual adjustments vs immediate adjustments
	tello.send_rc_control(0, for_back, up_down, yaw)
	return command_description

# ----- Main Loop (Object Detection + Drone Movement) -----

# Assign color/area constants after all required functions are defined and before main logic
TARGET_HSV = to_hsv(TARGET_COLOR)
COLOR_RANGES = build_hsv_ranges(TARGET_HSV, HSV_TOLERANCE)
TARGET_AREA = compute_target_area(TARGET_DISTANCE)

def run_tracking():
	"""Execute the full tracking loop, handling connection, control, and safety fallbacks."""

	def setup_drone_and_model():
		"""Connect to drone, setup stream, and load YOLO model."""
		tello = Tello()
		tello.connect()
		print(f"Battery: {tello.get_battery()}%")
		tello.streamoff()
		tello.streamon()
		model = YOLO(YOLO_MODEL_PATH)
		return tello, model

	def compute_control(detection, frame_center):
		"""Derive forward, vertical, and yaw corrections for ``detection`` relative to ``frame_center``."""

		offset_x = detection.center_x - frame_center[0]
		offset_y = frame_center[1] - detection.center_y
		area_delta = TARGET_AREA - detection.area

		yaw_velocity = 0
		forward_velocity = 0
		up_down_velocity = 0

		if abs(offset_x) > CENTER_DEADBAND:
			yaw_velocity = YAW_SPEED if offset_x > 0 else -YAW_SPEED

		if abs(offset_y) > CENTER_DEADBAND:
			up_down_velocity = UP_DOWN_SPEED if offset_y > 0 else -UP_DOWN_SPEED

		if abs(area_delta) > TARGET_AREA * AREA_TOLERANCE_RATIO:
			forward_velocity = FORWARD_BACK_SPEED if area_delta > 0 else -FORWARD_BACK_SPEED

		return int(forward_velocity), int(up_down_velocity), int(yaw_velocity)

	def process_frame(frame, tello, model, frame_center, last_command):
		"""Process one ``frame`` to update drone control, returning the new command label, timestamp, and detection."""

		resized_frame = cv2.resize(frame, (FRAME_WIDTH, FRAME_HEIGHT))
		hsv_frame = cv2.cvtColor(resized_frame, cv2.COLOR_BGR2HSV)

		detection = detect_with_color(hsv_frame, COLOR_RANGES)
		"""if detection is None:
			detection = detect_with_yolo(resized_frame, hsv_frame, model, TARGET_HSV, HSV_TOLERANCE)"""

		if detection:
			draw_annotations(resized_frame, detection, TARGET_AREA, frame_center)
			for_back, up_down, yaw = compute_control(detection, frame_center)
			last_command = apply_movement(tello, for_back, up_down, yaw, last_command)
			detection_timestamp = time.time()
		else:
			tello.send_rc_control(0, 0, 0, 0)
			detection_timestamp = None

		cv2.imshow("Falcon Vision", resized_frame)
		return last_command, detection_timestamp, detection

	tello, model = setup_drone_and_model()
	frame_reader = tello.get_frame_read()

	time.sleep(1.5)
	tello.takeoff()

	frame_center = (FRAME_WIDTH // 2, FRAME_HEIGHT // 2)
	last_command = None
	last_detection_time = time.time()

	try:
		while True:
			frame = frame_reader.frame
			if frame is None:
				continue

			last_command, detection_time, detection = process_frame(
				frame, tello, model, frame_center, last_command
			)

			if detection_time is not None:
				last_detection_time = detection_time
			else:
				tello.send_rc_control(0, 0, 0, SEARCH_YAW_SPEED)
				print("Searching for target...")
				if time.time() - last_detection_time > SEARCH_DURATION:
					print("Target lost. Landing.")
					tello.land()
					break

			if cv2.waitKey(1) & 0xFF == ord("q"):
				print("Manual exit requested. Landing.")
				tello.land()
				break

			time.sleep(FRAME_SLEEP)

	except KeyboardInterrupt:
		print("Interrupted. Landing drone.")
		tello.land()

	finally:
		tello.send_rc_control(0, 0, 0, 0)
		tello.streamoff()
		cv2.destroyAllWindows()
		tello.end()


def main():
	run_tracking()

if __name__ == "__main__":
	main()

