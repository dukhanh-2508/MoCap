import cv2
import numpy as np
import os
# Ép Qt sử dụng xcb (X11 tương thích ngược) hoặc wayland để triệt tiêu lỗi
os.environ["QT_QPA_PLATFORM"] = "xcb"

"""
RMS Error
0.191229501326295

Camera Matrix
[[500.54937452   0.         351.05078124]
 [  0.         501.26668079 219.20812061]
 [  0.           0.           1.        ]]

Distortion
[[ 0.12011192 -0.16491477 -0.00207192 -0.00053048 -0.03867355]]

Mean reprojection error
0.02244168963746111

"""

# ==========================================
# 1. KHỞI TẠO THÔNG SỐ (TỪ BLENDER MÀ RA)
# ==========================================
# Số lượng "góc trong" của bàn cờ 8x11
CHECKERBOARD = (8, 5) # 10 cột, 7 hàng
SQUARE_SIZE = 0.025 # 100mm = 0.1m

# Ma trận nội tại K (Intrinsic Matrix) tính tay từ thông số Blender
K = np.array([[500.54937452, 0.0,    351.05078124],
              [0.0,    501.26668079, 219.20812061],
              [0.0,    0.0,    1.0   ]], dtype=np.float64)

# Hệ số méo (0 hoàn toàn vì EEVEE render hoàn hảo)
dist_coeffs = np.array([0.12011192, -0.16491477, -0.00207192, -0.00053048, -0.03867355])
print(dist_coeffs)
print(dist_coeffs.shape)

# ==========================================
# 2. XÂY DỰNG TỌA ĐỘ 3D THẾ GIỚI (WCS)
# ==========================================
# Bàn cờ đang được dựng đứng trên mặt phẳng YZ, vuông góc tại gốc O(0,0,0)
# Trục X = 0. Các góc giao nằm trải dài trên Y và Z.
objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
# Tạo lưới tọa độ Y, Z và nhân với kích thước thật
grid = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)

# Gán tọa độ vào trục Y và Z (Trục X = 0)
objp[:, 1] = grid[:, 0] * SQUARE_SIZE  # Trục Y (Chiều ngang bàn cờ)
objp[:, 2] = grid[:, 1] * SQUARE_SIZE

# ==========================================
# 3. XỬ LÝ ẢNH & TÍNH TOÁN EXTRINSIC
# ==========================================
def calculate_extrinsics(image_path):
    # Đọc ảnh và chuyển sang Trắng Đen (Grayscale)
    img = cv2.imread(image_path)
    if img is None:
        print(f"Không tìm thấy ảnh: {image_path}")
        return
    
    '''
    #############
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Mô phỏng cách OpenCV binarize bức ảnh để tìm bàn cờ
    # Khối block size 11 hoặc 15 là thông số tiêu chuẩn
    binary = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, 
                                cv2.THRESH_BINARY, 15, 5)

    cv2.imshow("What OpenCV Sees (Binary)", binary)
    cv2.waitKey(0)
    corners_raw = cv2.goodFeaturesToTrack(gray, maxCorners=150, qualityLevel=0.01, minDistance=10)

    img_show = img.copy()
    if corners_raw is not None:
        corners_raw = np.intp(corners_raw)
        for i in corners_raw:
            x, y = i.ravel()
            # Chấm điểm màu đỏ lên các góc tìm được
            cv2.circle(img_show, (x, y), 3, (0, 0, 255), -1)

    cv2.imshow("Raw Corners", img_show)
    cv2.waitKey(0)
    #############
    '''

    cv2.imshow(os.path.basename(image_path), img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
        
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    cv2.imshow(f"Grayed: {os.path.basename(image_path)}", gray)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    # Quét ảnh để tìm các góc bàn cờ (Image Points 2D)
    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

    if ret:
        print(f"\n--- ĐÃ TÌM THẤY BÀN CỜ TRONG {image_path} ---")
        
        # Tinh chỉnh tọa độ pixel chính xác tới số thập phân (Sub-pixel accuracy)
        """"
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        """
        winSize = (3, 3) 
        zeroZone = (-1, -1)
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        corners_refined = cv2.cornerSubPix(gray, corners, winSize, zeroZone, criteria)

        # Trọng tâm: Giải phương trình PnP để tìm Ma trận Ngoại
        success, rvec, tvec = cv2.solvePnP(objp, corners_refined, K, dist_coeffs)
        projected_points, _ = cv2.projectPoints(objp, rvec, tvec, K, dist_coeffs)

        # Tính tổng khoảng cách Euclid (L2 Norm) giữa điểm chiếu và điểm thực
        error = cv2.norm(corners_refined, projected_points, cv2.NORM_L2) / len(projected_points)

        print(f"Reprojection Error: {error:.4f} pixels")

        axis_length = 0.3
        cv2.drawFrameAxes(img, K, dist_coeffs, rvec, tvec, axis_length)

        cv2.imwrite(image_path.replace(".jpg", "_axes.jpg"), img)

        if success:
            # rvec là Vector xoay. Cần dùng Rodrigues để dịch ra Ma trận xoay 3x3
            R, _ = cv2.Rodrigues(rvec)
            
            R_mat = np.matrix(R)
            tvec_mat = np.matrix(tvec)

            # Tính tọa độ thực của Camera trong không gian thế giới (WCS)
            camera_pos_wcs = -R_mat.T * tvec_mat
            offset = [[0],[0.025],[0.025]]
            camera_pos_wcs += offset

            print("Tọa độ của Camera trong không gian WCS (X, Y, Z):")
            print(camera_pos_wcs)
            print("\nRotation Matrix (Góc chúi/nghiêng của Camera):")
            print(R)
            
            # (Tùy chọn) Vẽ đè lên ảnh để test xem nó nhận diện đúng chưa
            cv2.drawChessboardCorners(img, CHECKERBOARD, corners_refined, ret)
            cv2.imwrite(image_path.replace(".jpg", "_calib.jpg"), img)
            
            return R, tvec, camera_pos_wcs
    else:
        print(f"THẤT BẠI: Thuật toán bị mù, không thấy đủ {CHECKERBOARD} góc trong {image_path}")
        return None, None, None

# Chạy thử cho 2 file ảnh
# R1, T1, c1 = calculate_extrinsics("/home/khanh/Blender/blender-5.1.2-linux-x64/mocap_Cam_1.jpg")
# R2, T2, c2 = calculate_extrinsics("/home/khanh/Blender/blender-5.1.2-linux-x64/mocap_Cam_2.jpg")
R1, T1, c1 = calculate_extrinsics("/home/khanh/Programming/Thesis/Thesis_1/MocapExCalib/cam0/frame_000017_cam_0.jpg")
R2, T2, c2 = calculate_extrinsics("/home/khanh/Programming/Thesis/Thesis_1/MocapExCalib/cam1/frame_000017_cam_1.jpg")
cam1_loc = (-4.0, 0.0, 1.2)
cam2_loc = (-4.0, 4.62, 1.2)

print(f"Check cam 0 error: {np.linalg.norm((cam1_loc - c1.ravel()))}")
print(f"Check cam 1 error: {np.linalg.norm((cam2_loc - c2.ravel()))}")
