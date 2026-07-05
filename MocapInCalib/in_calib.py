import cv2
import numpy as np
import glob

# ===========================
# Checkerboard setting
# ===========================

rows = 7          # số góc trong theo chiều dọc
cols = 10          # số góc trong theo chiều ngang
square_size = 1.0

# ===========================

objp = np.zeros((rows*cols,3), np.float32)
objp[:,:2] = np.mgrid[0:cols,0:rows].T.reshape(-1,2)
objp *= square_size

objpoints = []
imgpoints = []

images = glob.glob("/home/khanh/Programming/Thesis/Thesis_1/MocapInCalib/images/*.png")
print(images)
criteria = (
    cv2.TERM_CRITERIA_EPS +
    cv2.TERM_CRITERIA_MAX_ITER,
    30,
    0.001
)

for fname in images:

    img = cv2.imread(fname)

    gray = cv2.cvtColor(img,
                        cv2.COLOR_BGR2GRAY)

    ret, corners = cv2.findChessboardCorners(
        gray,
        (cols, rows),
        None
    )

    if not ret:
        print(fname, "not found")
        continue

    corners = cv2.cornerSubPix(
        gray,
        corners,
        (3,3),
        (-1,-1),
        criteria
    )

    objpoints.append(objp)
    imgpoints.append(corners)

    cv2.drawChessboardCorners(
        img,
        (cols,rows),
        corners,
        True
    )

    cv2.imshow("Corners", img)
    cv2.waitKey(200)

cv2.destroyAllWindows()

ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(
    objpoints,
    imgpoints,
    gray.shape[::-1],
    None,
    None
)

print("\n==============")
print("RMS Error")
print(ret)

print("\nCamera Matrix")
print(K)

print("\nDistortion")
print(dist)

# Reprojection error

total_error = 0

for i in range(len(objpoints)):

    imgpoints2, _ = cv2.projectPoints(
        objpoints[i],
        rvecs[i],
        tvecs[i],
        K,
        dist
    )

    error = cv2.norm(
        imgpoints[i],
        imgpoints2,
        cv2.NORM_L2
    ) / len(imgpoints2)

    total_error += error

print("\nMean reprojection error")
print(total_error / len(objpoints))

np.savez(
    "intrinsic.npz",
    cameraMatrix=K,
    distCoeffs=dist,
    rvecs=rvecs,
    tvecs=tvecs
)

print("\nSaved to intrinsic.npz")