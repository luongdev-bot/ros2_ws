#!/usr/bin/env python3
"""
Chương trình tự động chụp ảnh từ ROS topic để thu thập dữ liệu line.
- Khác với capture_line_dataset.py, script này tự động chụp theo chu kỳ
  thời gian, không cần nhấn phím.
- Ảnh lưu trong images/ và nhãn YOLO-seg LAB/contour lưu trong labels/;
  đây là nhãn heuristic nên vẫn cần spot-check.
- Tự động tiếp tục đánh số thứ tự, không ghi đè ảnh cũ.

Cần có: rclpy, cv_bridge và opencv-python.
"""

import argparse

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
            "Tự động chụp ROS topic vào images/ và tạo nhãn YOLO-seg "
            "LAB/contour vào labels/ (nên spot-check nhãn heuristic)"
        )
    )
    add_shared_arguments(parser)
    parser.add_argument(
        "--interval", type=float, default=1.0,
        help="Số giây giữa hai lần chụp tự động (mặc định: 1.0)",
    )
    parser.add_argument(
        "--show", action="store_true",
        help="Hiển thị cửa sổ xem trước để kiểm tra ảnh",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    if args.interval <= 0:
        raise SystemExit("--interval phải lớn hơn 0.")

    lab_min, lab_max, min_area_px = load_lab_thresholds(args.lab_config)
    counter = count_existing_files(args.output, args.prefix)
    initial_count = counter

    print(f"Topic: {args.topic}")
    print(f"Dataset: {args.output} (images/ và labels/)")
    print(f"Cấu hình LAB: {args.lab_config}")
    print(f"Chu kỳ chụp: {args.interval} giây")
    print(f"Đã có sẵn: {counter} ảnh với tiền tố '{args.prefix}'.")
    print("Nhấn Ctrl+C để dừng.\n")

    rclpy.init()
    node = rclpy.create_node("capture_line_dataset_auto")
    latest_frame, _subscription = subscribe_to_latest_frame(
        node, args.topic, CvBridge()
    )

    def capture_frame():
        nonlocal counter
        frame = latest_frame[0]
        if frame is None:
            print("Chưa nhận được ảnh, bỏ qua lần chụp này.")
            return

        filepath = save_frame(
            frame, args.output, args.prefix, counter, args.format
        )
        polygons = detect_line_polygons(frame, lab_min, lab_max, min_area_px)
        label_path = label_path_for_image(args.output, filepath)
        write_yolo_seg_label(label_path, polygons)
        counter += 1
        print(f"Đã lưu: {filepath}")
        print(f"  Nhãn: {len(polygons)} vùng line -> {label_path}")

    node.create_timer(args.interval, capture_frame)

    if args.show:
        def show_preview():
            frame = latest_frame[0]
            if frame is not None:
                # Cửa sổ chỉ để kiểm tra; ảnh lưu luôn là khung gốc.
                preview = frame.copy()
                cv2.putText(
                    preview, f"Topic: {args.topic}", (10, 35),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2,
                )
                cv2.putText(
                    preview, f"Da chup: {counter}", (10, 70),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2,
                )
                cv2.imshow("Thu thap du lieu - ROS topic", preview)
            cv2.waitKey(1)

        node.create_timer(0.03, show_preview)

    try:
        rclpy.spin(node)
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
