"""Hàm dùng chung cho các script chụp dữ liệu line từ ROS topic."""

import os
from datetime import datetime

import cv2
import numpy as np
import yaml
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data


DEFAULT_TOPIC = "/depth_cam/image"
DEFAULT_OUTPUT_DIR = os.path.join(
    os.path.expanduser("~"), "line AI", "dataset", "line"
)
DEFAULT_LAB_CONFIG = os.path.abspath(os.path.join(
    os.path.dirname(__file__), "..", "src", "line_follow", "config",
    "line_follow.yaml",
))


def add_shared_arguments(parser):
    """Thêm các tham số dòng lệnh chung cho script chụp ảnh."""
    parser.add_argument(
        "--topic", type=str, default=DEFAULT_TOPIC,
        help="ROS topic kiểu sensor_msgs/Image cần chụp",
    )
    parser.add_argument(
        "--output", type=str, default=DEFAULT_OUTPUT_DIR,
        help="Thư mục gốc dataset (ảnh ở images/, nhãn ở labels/)",
    )
    parser.add_argument(
        "--prefix", type=str, default="img",
        help="Tiền tố tên file ảnh",
    )
    parser.add_argument(
        "--format", type=str, default="jpg", choices=["jpg", "png"],
        help="Định dạng ảnh",
    )
    parser.add_argument(
        "--lab-config", type=str, default=DEFAULT_LAB_CONFIG,
        help="File YAML chứa ngưỡng LAB color cho nhãn tự động",
    )


def count_existing_files(output_dir, prefix):
    """Đếm ảnh trong images/ để chụp tiếp mà không ghi đè file cũ."""
    images_dir = os.path.join(output_dir, "images")
    if not os.path.isdir(images_dir):
        return 0

    return sum(
        1 for filename in os.listdir(images_dir)
        if filename.startswith(prefix)
        and os.path.isfile(os.path.join(images_dir, filename))
    )


def save_frame(frame, output_dir, prefix, counter, fmt):
    """Lưu khung hình gốc vào images/ và trả về đường dẫn file vừa tạo."""
    images_dir = os.path.join(output_dir, "images")
    labels_dir = os.path.join(output_dir, "labels")
    os.makedirs(images_dir, exist_ok=True)
    os.makedirs(labels_dir, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"{prefix}_{counter:04d}_line_{timestamp}.{fmt}"
    filepath = os.path.join(images_dir, filename)

    if not cv2.imwrite(filepath, frame):
        raise OSError(f"Không thể lưu ảnh: {filepath}")

    return filepath


def label_path_for_image(output_dir, image_path):
    """Trả về đường dẫn nhãn YOLO tương ứng trong labels/."""
    filename = os.path.splitext(os.path.basename(image_path))[0] + ".txt"
    return os.path.join(output_dir, "labels", filename)


def load_lab_thresholds(config_path):
    """Đọc và kiểm tra ngưỡng LAB từ phần color: của file YAML."""
    if not os.path.isfile(config_path):
        raise FileNotFoundError(f"Không tìm thấy file cấu hình LAB: {config_path}")

    try:
        with open(config_path, "r", encoding="utf-8") as config_file:
            config = yaml.safe_load(config_file)
    except yaml.YAMLError as error:
        raise ValueError(
            f"Không đọc được YAML cấu hình LAB '{config_path}': {error}"
        ) from error

    if not isinstance(config, dict) or not isinstance(config.get("color"), dict):
        raise ValueError(
            f"Cấu hình LAB '{config_path}' phải có phần 'color:' dạng mapping"
        )

    color = config["color"]
    missing_keys = [
        key for key in ("lab_min", "lab_max", "min_area_px")
        if key not in color
    ]
    if missing_keys:
        raise ValueError(
            f"Phần 'color:' trong '{config_path}' thiếu khóa: "
            + ", ".join(missing_keys)
        )

    def validate_lab_triplet(key):
        value = color[key]
        if (
            not isinstance(value, (list, tuple))
            or len(value) != 3
            or any(isinstance(channel, bool) or not isinstance(channel, int)
                   for channel in value)
            or any(channel < 0 or channel > 255 for channel in value)
        ):
            raise ValueError(
                f"'{key}' trong '{config_path}' phải là 3 số nguyên từ 0 đến 255"
            )
        return tuple(value)

    lab_min = validate_lab_triplet("lab_min")
    lab_max = validate_lab_triplet("lab_max")
    if any(lower > upper for lower, upper in zip(lab_min, lab_max)):
        raise ValueError(
            f"'lab_min' không được lớn hơn 'lab_max' trong '{config_path}'"
        )

    min_area_px = color["min_area_px"]
    if (
        isinstance(min_area_px, bool)
        or not isinstance(min_area_px, (int, float))
        or min_area_px <= 0
    ):
        raise ValueError(
            f"'min_area_px' trong '{config_path}' phải là số dương"
        )

    return lab_min, lab_max, min_area_px


def detect_line_polygons(frame, lab_min, lab_max, min_area_px):
    """Tìm các polygon line trên toàn khung hình và chuẩn hóa tọa độ YOLO."""
    if frame is None or frame.size == 0:
        raise ValueError("Không thể tạo nhãn từ khung hình rỗng")

    height, width = frame.shape[:2]
    if height <= 0 or width <= 0:
        raise ValueError("Không thể tạo nhãn từ khung hình không có kích thước")

    lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
    mask = cv2.inRange(
        lab,
        np.array(lab_min, dtype=np.uint8),
        np.array(lab_max, dtype=np.uint8),
    )
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    eroded = cv2.erode(mask, kernel)
    opened = cv2.dilate(eroded, kernel)
    contours = cv2.findContours(
        opened, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )[-2]

    polygons = []
    for contour in contours:
        if cv2.contourArea(contour) < min_area_px:
            continue

        epsilon = 0.002 * cv2.arcLength(contour, closed=True)
        simplified = cv2.approxPolyDP(contour, epsilon, closed=True)
        points = simplified.reshape(-1, 2)
        if len(points) < 3:
            continue

        polygons.append([
            (
                min(max(float(x) / width, 0.0), 1.0),
                min(max(float(y) / height, 0.0), 1.0),
            )
            for x, y in points
        ])

    return polygons


def write_yolo_seg_label(label_path, polygons, class_id=0):
    """Ghi nhãn YOLO-seg; danh sách rỗng tạo file nhãn rỗng hợp lệ."""
    label_dir = os.path.dirname(os.path.abspath(label_path))
    os.makedirs(label_dir, exist_ok=True)

    with open(label_path, "w", encoding="utf-8") as label_file:
        for polygon in polygons:
            coordinates = " ".join(
                f"{x:.6f} {y:.6f}" for x, y in polygon
            )
            label_file.write(f"{class_id} {coordinates}\n")


def subscribe_to_latest_frame(node, topic, bridge):
    """Đăng ký ROS topic và trả về nơi chứa khung hình mới nhất."""
    latest_frame = [None]

    def image_callback(msg):
        try:
            latest_frame[0] = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as error:
            node.get_logger().warning(f"Không chuyển đổi được ảnh từ {topic}: {error}")

    subscription = node.create_subscription(
        Image,
        topic,
        image_callback,
        qos_profile_sensor_data,
    )
    return latest_frame, subscription
