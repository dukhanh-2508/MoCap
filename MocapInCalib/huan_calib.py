import cv2
import os

folder = 'calib_images'
if not os.path.exists(folder):
    os.makedirs(folder)

cap = cv2.VideoCapture(0)
count = 0

print("Nhấn 's' để CHỤP, 'q' để THOÁT.")

while True:
    ret, frame = cap.read()
    if not ret: break

    display = frame.copy()
    # Tìm góc
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    ret_find, corners = cv2.findChessboardCorners(gray, (8, 5), None)

    if ret_find:
        cv2.drawChessboardCorners(display, (8, 5), corners, ret_find)
        cv2.putText(display, "READY TO SAVE", (10, 30), 1, 1, (0, 255, 0), 2)
    else:
        cv2.putText(display, "NOT FOUND", (10, 30), 1, 1, (0, 0, 255), 2)

    cv2.imshow("Capture Calib", display)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('s') and ret_find:
        count += 1
        fn = f"{folder}/calib_{count}.jpg"
        cv2.imwrite(fn, frame)  # Lưu ảnh gốc (không có vẽ góc)
        print(f"Đã lưu ảnh thứ {count}: {fn}")

    elif key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

##############################3

import cv2
import numpy as np
import glob

CHESS_SIZE = (8, 5)
SQUARE_SIZE = 20.0

# Chuẩn bị tọa độ 3D thực tế (0,0,0), (20,0,0), (40,0,0)...
objp = np.zeros((CHESS_SIZE[0] * CHESS_SIZE[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHESS_SIZE[0], 0:CHESS_SIZE[1]].T.reshape(-1, 2) * SQUARE_SIZE

objpoints = []  # Điểm 3D thực tế (World Points)
imgpoints = []  # Điểm 2D trên ảnh (Image Points)

images = glob.glob('calib_images/*.jpg')

print(f"Đang xử lý {len(images)} ảnh hiệu chuẩn...")

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Tìm các góc bàn cờ 
    ret, corners = cv2.findChessboardCorners(gray, CHESS_SIZE, None)

    if ret:
        objpoints.append(objp)
        # Tinh chỉnh tọa độ góc đến mức dưới pixel 
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners2)

        # Vẽ lên màn hình để kiểm tra
        cv2.drawChessboardCorners(img, CHESS_SIZE, corners2, ret)
        cv2.imshow('Checking Chessboard Corners', img)
        cv2.waitKey(100)

cv2.destroyAllWindows()

# Hàm này để tính toán Ma trận K và các hệ số biến dạng ống kính
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

if ret:
    print("\n Hiệu chỉnh thành công")
    print(f"Ma trận Nội thông số (K):\n{mtx}")
    print(f"Hệ số biến dạng (Distortion):\n{dist}")

    # Tính sai số trung bình (RMS error) - Càng gần 0 càng tốt (< 0.5)
    mean_error = 0
    for i in range(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
        error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
        mean_error += error
    print(f"Sai số tổng thể (RMS Error): {mean_error / len(objpoints):.4f} pixel")

    # Lưu la
    np.savez("camera_params.npz", mtx=mtx, dist=dist)
    print("\nĐã lưu vào file 'camera_params.npz'")
else:
    print(" Không thể hiệu chuẩn! ")