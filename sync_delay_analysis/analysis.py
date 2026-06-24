"""
Code to analyze photos taken to measure synchronization delay between 2 slave camera.
Assume all files are saved as JPGs

"""
import os
import re
import json
import csv
os.environ["QT_QPA_PLATFORM"] = "xcb"
import cv2



REFERENCE_FILE_NAME = "reference.jpg"
FIRST_CAMERA_FOLDER = "/home/khanh/Programming/Thesis/Thesis_1/sync_delay_analysis/native_slave/mocap_frames" 
SECOND_CAMERA_FOLDER = "/home/khanh/Programming/Thesis/Thesis_1/sync_delay_analysis/outside_slave/mocap_frames" 
FIRST_ANNOTATED_FOLDER = "/home/khanh/Programming/Thesis/Thesis_1/sync_delay_analysis/annotated/native_slave"
SECOND_ANNOTATED_FOLDER = "/home/khanh/Programming/Thesis/Thesis_1/sync_delay_analysis/annotated/outside_slave"
REPORT_FOLDER = "/home/khanh/Programming/Thesis/Thesis_1/sync_delay_analysis/annotated"

# FIRST_CAMERA_FOLDER = "/home/khanh/Programming/Thesis/Thesis_1/sync_delay_analysis/auto_analyze_dataset/ref"


# Dict structure to save coordinates of each LED according to their bit order
amountOfBits = 15
one = amountOfBits + 2
# leds_1 = {f"bit_{i}":[0, 0] for i in range(15 + 1)}
# leds_2 = {f"bit_{i}":[0, 0] for i in range(15 + 1)}

def find_ref_bit_led_coord(refPicPath, bitAmount, lightThrs=200):
    file_name = os.path.basename(refPicPath)
    if refPicPath != REFERENCE_FILE_NAME:
        print("Not reference image")
        # return {}
    
    leds = {}
    
    image = cv2.imread(refPicPath, cv2.IMREAD_GRAYSCALE) # Open the image as a grayscaled image
    # use cv2.imshow to show the image
    copyImg = cv2.imread(refPicPath)
    cv2.imshow("Normal Img", copyImg)
    cv2.waitKey(0)

    # Process the grayscale reference image with the following pipeline:
    # binary thresholding -> find contour -> find center using moment
    _, thresholdImg = cv2.threshold(image, lightThrs, 255, cv2.THRESH_BINARY)
    cv2.imshow("Threshold Img", thresholdImg)
    cv2.waitKey(0)
    contours, _ = cv2.findContours(thresholdImg, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if len(contours) > bitAmount:
        print(f"Redundant contours detected: {len(contours) - bitAmount}")
    # cv2.drawContours(copyImg, contours, -1, (0, 0, 255), 1)
    centers = [] 
    for contour in contours:
        M = cv2.moments(contour)
        print(f"Moment: {M["m00"]} - {M["m01"]} - {M["m10"]}")
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            centers.append([cx, cy])
            print((cx, cy))
            # cv2.circle(copyImg, (cx, cy), radius=2, color=(255, 0, 0), thickness=-1)
        else:
            x, y, w, h = cv2.boundingRect(contour)
            cx = int(x + w / 2.0)
            cy = int(y + h / 2.0)
            
            if w == 0:
                w = 1
            centers.append([cx, cy])
            print((cx, cy))
    
    # Assume LSB is in the most left of the bit sequence
    centers.sort(key=lambda point: point[0])
    # Filtering out redundant centers to deal with fragmentation of the LED threshold drawing
    # Using the asumption that the LEDs are distributed along the x axis, points
    # that have approximately the same coordinate are considered within the same LED.
    # The brightest point will be chosen.
    threshold = 3 # Abitrary value
    temp = []
    filteredCenters = []
    prevX = None
    for center in centers:
        if prevX == None:
            prevX = center[0]
            temp.append(center)
            continue

        if abs(center[0] - prevX) <= threshold:
            temp.append(center)
            prevX = center[0]
        else: # This point is of another LED
            # Find the brightest point of the previous LED
            temp.sort(key=lambda point: point[1])
            brightestPoint = temp[len(temp) - 1]
            filteredCenters.append(brightestPoint)

            # Prepare for this new LED
            temp.clear()
            prevX = center[0]
            temp.append(center)
    # Caculate for the last LED
    if len(temp) > 0:
        temp.sort(key=lambda point: point[0])
        brightestPoint = temp[len(temp) - 1]
        filteredCenters.append(brightestPoint)

    # For debugging, drawing the filtered centers
    for point in filteredCenters:
        print(f"Filtered center {point}")
        cv2.circle(copyImg, point, 4, (0, 0, 255), -1)

    # For debugging, show the contours and centers overlayed on the original image
    cv2.imshow("", copyImg)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    return {f"bit{i}":point for i in range(bitAmount) for point in centers}


def find_ref_bit_led_coord_dynamic_LED_threshold(refPicPath, bitAmount):
    file_name = os.path.basename(refPicPath)
    if refPicPath != REFERENCE_FILE_NAME:
        print("Not reference image")
        # return {}
    
    leds = {}
    
    image = cv2.imread(refPicPath, cv2.IMREAD_GRAYSCALE) # Open the image as a grayscaled image
    # use cv2.imshow to show the image
    copyImg = cv2.imread(refPicPath)
    cv2.imshow("Normal Img", copyImg)
    cv2.waitKey(0)

    # Process the grayscale reference image with the following pipeline:
    # binary thresholding -> find contour -> find center using moment
    _, thresholdImg = cv2.threshold(image, 200, 255, cv2.THRESH_BINARY)
    cv2.imshow("Threshold Img", thresholdImg)
    cv2.waitKey(0)
    contours, _ = cv2.findContours(thresholdImg, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if len(contours) > bitAmount:
        print(f"Redundant contours detected: {len(contours) - bitAmount}")
    # cv2.drawContours(copyImg, contours, -1, (0, 0, 255), 1)
    
    centers = [] 
    for contour in contours:
        M = cv2.moments(contour)
        print(f"Moment: {M['m00']} - {M['m01']} - {M['m10']}")
        
        # Bắt buộc lấy Bounding Box cho mọi contour để đo chiều rộng w
        x, y, w, h = cv2.boundingRect(contour)
        if w == 0:
            w = 1
            
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            centers.append([cx, cy, w]) # Nhét thêm w vào mảng
            print((cx, cy))
        else:
            cx = int(x + w / 2.0)
            cy = int(y + h / 2.0)
            centers.append([cx, cy, w]) # Nhét thêm w vào mảng
            print((cx, cy))
    
    # Assume LSB is in the most left of the bit sequence
    centers.sort(key=lambda point: point[0])
    
    # Filtering out redundant centers to deal with fragmentation of the LED threshold drawing
    
    # --- LOGIC DYNAMIC THRESHOLD ---
    if len(centers) > 0:
        max_width = max(point[2] for point in centers)
        threshold = max_width * 1.5
    else:
        threshold = 3
    # -------------------------------
    
    temp = []
    filteredCenters = []
    prevX = None
    for center in centers:
        if prevX == None:
            prevX = center[0]
            temp.append(center)
            continue

        if abs(center[0] - prevX) <= threshold:
            temp.append(center)
        else: # This point is of another LED
            # Find the brightest point of the previous LED
            temp.sort(key=lambda point: point[0])
            brightestPoint = temp[len(temp) - 1]
            filteredCenters.append(brightestPoint)

            # Prepare for this new LED
            temp.clear()
            prevX = center[0]
            temp.append(center)
            
    # Caculate for the last LED
    if len(temp) > 0:
        temp.sort(key=lambda point: point[0])
        brightestPoint = temp[len(temp) - 1]
        filteredCenters.append(brightestPoint)

    # For debugging, drawing the filtered centers
    for point in filteredCenters:
        print(f"Filtered center {point[:2]}")
        # Chỉ lấy cx, cy để vẽ, bỏ w đi
        cv2.circle(copyImg, (point[0], point[1]), 4, (0, 0, 255), -1)

    # For debugging, show the contours and centers overlayed on the original image
    cv2.imshow("", copyImg)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    # Trả về filteredCenters (đã cắt bỏ w) thay vì mảng centers gốc chưa lọc
    return {f"bit{i}":point[:2] for i in range(bitAmount) for point in filteredCenters}


def find_ref_bit(refPicPath, bitAmount, lightThrs=200, saveFolder="", isSaveData=False):
    if os.path.basename(refPicPath) != REFERENCE_FILE_NAME:
        print("Not reference image")
        return {}
        
    image = cv2.imread(refPicPath, cv2.IMREAD_GRAYSCALE) # Open the image as a grayscaled image

    # Process the grayscale reference image with the following pipeline:
    # binary thresholding -> find contour -> find center using moment
    _, thresholdImg = cv2.threshold(image, lightThrs, 255, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(thresholdImg, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    centers = [] 
    for contour in contours:
        M = cv2.moments(contour)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            centers.append([cx, cy])
        else:
            x, y, w, h = cv2.boundingRect(contour)
            cx = int(x + w / 2.0)
            cy = int(y + h / 2.0)
            
            if w == 0:
                w = 1
            centers.append([cx, cy])
    
    # Assume LSB is in the most left of the bit sequence
    centers.sort(key=lambda point: point[0])
    # Filtering out redundant centers to deal with fragmentation of the LED threshold drawing
    # Using the asumption that the LEDs are distributed along the x axis, points
    # that have approximately the same coordinate are considered within the same LED.
    # The brightest point will be chosen.
    threshold = 3 # Abitrary value
    temp = []
    filteredCenters = []
    prevX = None
    for center in centers:
        if prevX == None:
            prevX = center[0]
            temp.append(center)
            continue

        if abs(center[0] - prevX) <= threshold:
            temp.append(center)
            prevX = center[0]
        else: # This point is of another LED
            # Find the brightest point of the previous LED
            temp.sort(key=lambda point: point[1])
            brightestPoint = temp[len(temp) - 1]
            filteredCenters.append(brightestPoint)

            # Prepare for this new LED
            temp.clear()
            prevX = center[0]
            temp.append(center)
    # Caculate for the last LED
    if len(temp) > 0:
        temp.sort(key=lambda point: point[0])
        brightestPoint = temp[len(temp) - 1]
        filteredCenters.append(brightestPoint)

    if len(filteredCenters) > bitAmount:
        print(f"File {os.path.basename(refPicPath)}: LED detected OVER the specified amount")
        return {}
    elif len(filteredCenters) < bitAmount:
        print(f"File {os.path.basename(refPicPath)}: LED detected BELOW the specified amount")
        return {}
    
    returnDict = {f"bit{i}": filteredCenters[i] for i in range(bitAmount)}

    if saveFolder != "" and isSaveData == True:
        with open(os.path.join(saveFolder, "ref_coord.json"), "w") as file:
            json.dump(returnDict, file, indent=4)

        copyImg = cv2.imread(refPicPath)
        for center in filteredCenters:
            cv2.circle(copyImg, center, 4, (0, 0, 255), -1)
            cv2.imwrite(os.path.join(saveFolder, "reference.jpg"), copyImg)

    return returnDict

def decode_single_image(img_path, led_coord, lightThreshold=200, saveFolder="", isSaveData=False):
    img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        return 0
            
    buffer = 0
    bit_amount = len(led_coord)
        
    copyImg = None
    if saveFolder != "" and isSaveData == True:
        copyImg = cv2.imread(img_path)

    for i in range(bit_amount):
        # Lấy tọa độ (x, y) của bit thứ i
        x, y = led_coord[f"bit{i}"]
            
        # Đọc vùng 3x3 quanh tâm để chống nhiễu đốm đen trên LED
        y_min, y_max = max(0, y - 20), min(img.shape[0], y + 20)
        x_min, x_max = max(0, x - 1), min(img.shape[1], x + 2)
        roi = img[y_min:y_max, x_min:x_max]
            
        # Nếu điểm sáng nhất trong vùng 3x3 vượt ngưỡng -> Bit = 1
        if roi.max() >= lightThreshold:
            buffer |= (1 << i)

            if saveFolder != "" and isSaveData == True:
                cv2.circle(copyImg, (x, y),  4, (0, 0, 255), -1)
        else:
            if saveFolder != "" and isSaveData == True:
                cv2.circle(copyImg, (x, y),  2, (0, 255, 255), -1)

    
    if saveFolder != "" and isSaveData == True:
        note = f"{os.path.basename(img_path)}: {buffer} - {[f'{b:08b}' for b in buffer.to_bytes(2, byteorder='little')]}"
        cv2.putText(copyImg, note, org=(0, 50), color=(0, 255, 255), thickness=1, fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.5, lineType=cv2.LINE_AA)
        cv2.imwrite(os.path.join(saveFolder, os.path.basename(img_path)), copyImg)
                
    return buffer

def save_decode_results_to_csv(result_dict, save_folder):
    with open(os.path.join(save_folder, "delayMeasureResult.csv"), "w", newline="") as file:
        writer = csv.writer(file)
        
        writer.writerow(["Frame ID", "Camera 0 Value", "Camera 1 Value", "Difference (Cam2 - Cam1)"])
        
        # 2. Quét cái dict lồng nhau và bóp phẳng (flatten) nó ra thành từng hàng
        for frame_name, data in result_dict.items():
            writer.writerow([
                frame_name, 
                data["cam_0"], 
                data["cam_1"], 
                data["diff"]
            ])
            
def decode_bit_led(img_dir1, img_dir2, led_coord1, led_coord2, lightThreshold=200, saveFolder="", isSaveData=False):
    """
    img_dir1, img_dir2 are the paths to the dirs of the slave images
    led_coord1, led_coord2 are the dict given by find_ref_bit and contain the reference coordinate for the LED arrays
    """
    result = {}
    
    # Biểu thức Regex để bắt định dạng: "frame_XXXXXX_cam_1.jpg"
    pattern_cam1 = re.compile(r"frame_(\d{6})_cam_0\.jpg")
    # Quét toàn bộ file trong thư mục của Camera 1
    for filename1 in sorted(os.listdir(img_dir1)):
        match = pattern_cam1.match(filename1)
        if not match:
            continue # Bỏ qua nếu không đúng định dạng (VD: file reference)
            
        frame_num = match.group(1) # Lấy ra chuỗi 6 số (VD: "000001")
        
        # Suy ra tên file tương ứng của Camera 2
        filename2 = f"frame_{frame_num}_cam_1.jpg"
        
        path1 = os.path.join(img_dir1, filename1)
        path2 = os.path.join(img_dir2, filename2)
        
        # Nếu frame này bị rớt ở cam 2 thì bỏ qua đo đạc frame này
        if not os.path.exists(path2):
            continue 
            
        # Giải mã giá trị Decimal cho cả 2 camera
        val1 = decode_single_image(path1, led_coord1, lightThreshold, saveFolder=FIRST_ANNOTATED_FOLDER, isSaveData=isSaveData)
        val2 = decode_single_image(path2, led_coord2, lightThreshold, saveFolder=SECOND_ANNOTATED_FOLDER, isSaveData=isSaveData)
        
        # Ghi vào Dictionary tổng
        result[f"frame_{frame_num}"] = {
            "cam_0": val1,
            "cam_1": val2,
            "diff": val2 - val1
        }

        if saveFolder != "" and isSaveData == True:
            save_decode_results_to_csv(result, saveFolder)
        
    return result

lightThreshold = 95
detectionThrs = 60

"""
for file in os.listdir(FIRST_CAMERA_FOLDER):
    print(f"Analyzing file {file}")
    find_ref_bit_led_coord(os.path.join(FIRST_CAMERA_FOLDER, REFERENCE_FILE_NAME), 15)
"""
# find_ref_bit_led_coord(os.path.join(SECOND_CAMERA_FOLDER, REFERENCE_FILE_NAME), 15, lightThreshold)


# Process the reference pictures and populate the coordinate dict
refPic1 = os.path.join(FIRST_CAMERA_FOLDER, REFERENCE_FILE_NAME)
refPic2 = os.path.join(SECOND_CAMERA_FOLDER, REFERENCE_FILE_NAME)
leds_1 = find_ref_bit(refPic1, amountOfBits, lightThreshold, saveFolder=FIRST_ANNOTATED_FOLDER, isSaveData=True)
leds_2 = find_ref_bit(refPic2, amountOfBits, lightThreshold, saveFolder=SECOND_ANNOTATED_FOLDER, isSaveData=True)
frameCount = 50

result = decode_bit_led(FIRST_CAMERA_FOLDER, SECOND_CAMERA_FOLDER, leds_1, leds_2, detectionThrs, saveFolder=REPORT_FOLDER, isSaveData=True)

