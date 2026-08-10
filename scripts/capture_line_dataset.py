#!/usr/bin/env python3
"""
Chương trình chụp ảnh từ ROS topic để thu thập dữ liệu line.
- Nhận sensor_msgs/Image từ camera Gazebo qua ROS topic.
- Nhấn phím để chụp ảnh vào images/ và nhãn YOLO-seg vào labels/.
- Nhãn được tạo tự động bằng LAB/contour, nên vẫn cần spot-check vì đó là
  nhãn heuristic, không phải ground truth hoàn hảo.
- Tự động tiếp tục đánh số thứ tự, không ghi đè ảnh cũ.

Cần có: rclpy, cv_bridge và opencv-python.
"""

import argparse
import time

import cv2
import rclpy
import rclpy.executors
from cv_bridge import CvBridge

from _line_capture_common import (
    add_shared_arguments,
    count_existing_files,
    detect_line_polygons,
    label_path_for_image,
    load_lab_thresholds,
    save_frame,
    subscribe_to_latest_frame,
    write_yolo_seg_label,
)


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Chụp ảnh ROS topic vào images/ và tạo nhãn YOLO-seg "
            "LAB/contour vào labels/ (nên spot-check nhãn heuristic)"
        )
    )
    add_shared_arguments(parser)
    return parser.parse_args()


def main():
    args = parse_args()
    lab_min, lab_max, min_area_px = load_lab_thresholds(args.lab_config)
    counter = count_existing_files(args.output, args.prefix)
    initial_count = counter

    print(f"Topic: {args.topic}")
    print(f"Dataset: {args.output} (images/ và labels/)")
    print(f"Cấu hình LAB: {args.lab_config}")
    print(f"Đã có sẵn: {counter} ảnh với tiền tố '{args.prefix}'.")
    print("\n=== HƯỚNG DẪN ===")
    print("  [SPACE] hoặc [C] : chụp và lưu ảnh")
    print("  [Q] hoặc [ESC]   : thoát")
    print("==================\n")

    rclpy.init()
    node = rclpy.create_node("capture_line_dataset")
    latest_frame, _subscription = subscribe_to_latest_frame(
        node, args.topic, CvBridge()
    )

    try:
        waiting_for_frame = False
        while True:
            rclpy.spin_once(node, timeout_sec=0.05)
            frame = latest_frame[0]
            if frame is None:
                if not waiting_for_frame:
                    print("Đang chờ ảnh đầu tiên từ ROS topic...")
                    waiting_for_frame = True
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), ord("Q"), 27):
                    break
                time.sleep(0.05)
                continue
            waiting_for_frame = False

            # Hiển thị thông tin lên khung xem trước, không lưu chữ vào ảnh gốc.
            preview = frame.copy()
            cv2.putText(
                preview, f"Topic: {args.topic}", (10, 35),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2,
            )
            cv2.putText(
                preview, f"Da chup: {counter}", (10, 70),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2,
            )
            cv2.putText(
                preview, "SPACE/C: chup | Q/ESC: thoat", (10, 105),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2,
            )
            cv2.imshow("Thu thap du lieu - ROS topic", preview)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), ord("Q"), 27):
                break
            if key in (ord(" "), ord("c"), ord("C")):
                filepath = save_frame(
                    frame, args.output, args.prefix, counter, args.format
                )
                polygons = detect_line_polygons(
                    frame, lab_min, lab_max, min_area_px
                )
                label_path = label_path_for_image(args.output, filepath)
                write_yolo_seg_label(label_path, polygons)
                counter += 1
                print(f"Đã lưu: {filepath}")
                print(f"  Nhãn: {len(polygons)} vùng line -> {label_path}")
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("\n=== TỔNG KẾT ===")
        print(f"  Đã chụp trong lần này: {counter - initial_count} ảnh")
        print("================")


if __name__ == "__main__":
    main()
