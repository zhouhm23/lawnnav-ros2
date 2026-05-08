#!/usr/bin/env python3
"""
test_aruco.py — 快速测试 ArUco 标记检测（单张图片）

用法:
    python test_aruco.py --image aruco_id0.png          # 检测指定图片
    python test_aruco.py --image aruco_id0.png --camera  # 用摄像头实时检测（按 q 退出）

依赖: pip install opencv-python
"""

import argparse
import sys
import cv2
import numpy as np

# 默认 ArUco 字典、检测 ID 和显示参数
DICT_TYPE = cv2.aruco.DICT_4X4_50
EXPECTED_IDS = {0, 1, 2, 3, 4}


def detect_and_show(image: np.ndarray) -> None:
    """在图片中检测 ArUco 标记并显示标注结果。"""
    aruco_dict = cv2.aruco.getPredefinedDictionary(DICT_TYPE)
    params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, params)

    corners, ids, _ = detector.detectMarkers(image)

    if ids is None or len(ids) == 0:
        print("⚠ 未检测到任何 ArUco 标记")
    else:
        cv2.aruco.drawDetectedMarkers(image, corners, ids)
        for i, c in enumerate(corners):
            mid = int(ids[i][0])
            center = tuple(np.mean(c[0], axis=0).astype(int))
            status = "✓" if mid in EXPECTED_IDS else "?"
            print(f"  {status} ID={mid}, 中心=({center[0]}, {center[1]})")

    cv2.imshow("ArUco Test — 按任意键关闭", image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def test_camera() -> None:
    """实时检测摄像头中的 ArUco 标记。"""
    aruco_dict = cv2.aruco.getPredefinedDictionary(DICT_TYPE)
    params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, params)

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("无法打开摄像头")
        sys.exit(1)

    print("摄像头已启动，按 q 退出...")
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        corners, ids, _ = detector.detectMarkers(frame)
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        cv2.imshow("ArUco Camera Test — 按 q 退出", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="快速测试 ArUco 标记检测")
    parser.add_argument("--image", help="图片路径 (.png/.jpg)")
    parser.add_argument("--camera", action="store_true", help="使用摄像头实时检测")
    args = parser.parse_args()

    if args.camera:
        test_camera()
    elif args.image:
        img = cv2.imread(args.image)
        if img is None:
            print(f"无法读取图片: {args.image}")
            sys.exit(1)
        print(f"检测图片: {args.image} ({img.shape[1]}×{img.shape[0]})")
        detect_and_show(img)
    else:
        print("请指定 --image <路径> 或 --camera")
        print("示例: python test_aruco.py --image aruco_id0.png")
        sys.exit(1)
