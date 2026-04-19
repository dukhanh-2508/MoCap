#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include <string>
#include <chrono>
#include <thread>

/*
First task: To detect the bright blob that is the marker

Algorithm: SimpleBlobDetector, as the name implies, is based on a rather simple algorithm described below.
The algorithm is controlled by parameters ( shown in bold below )  and has the following steps. 
    Thresholding : Convert the source images to several binary images by thresholding the source image with thresholds starting at minThreshold. These thresholds are incremented  by thresholdStep until maxThreshold.
    So the first threshold is minThreshold, the second is minThreshold + thresholdStep, the third is minThreshold + 2 x thresholdStep, and so on.
    
    Grouping : In each binary image, connected white pixels are grouped together. 
    Let’s call these binary blobs.
    
    Merging : The centers of the binary blobs in the binary images are computed, and  blobs located closer than minDistBetweenBlobs are merged.
    
    Center & Radius Calculation :  The centers and radii of the new merged blobs are computed and returned.
The algorithm above applies multi-level thresholding. It is more stable than the single-threshold approach presented below:
    Thresholding with just one threshold ==> One binary image

    Contour extraction: Extract the boundaries of the blobs

    Geometric Filtering: Remove blobs that aren't markers using area and circularity.

    Find the center of mass of the blob ==> Get coord

*/

/*
Line: ~2.6m away from the camera, where cant recognize the points
Settings: 
params.minThreshold = 150;
    params.maxThreshold = 240;
    params.thresholdStep = 10; // Cắt lát mỗi 10 đơn vị cường độ sáng

    // --- BỘ LỌC HÌNH HỌC (GEOMETRIC FILTERS) ---
    // A. Lọc theo màu sắc: Chỉ tìm các đốm sáng trắng (pixel tiến về 255)
    params.filterByColor = true;
    params.blobColor = 255;

    // B. Lọc theo diện tích: Loại bỏ nhiễu hạt tiêu và vệt lóa lớn
    params.filterByArea = true;
    params.minArea = 50.0f;    // Ngưỡng an toàn cho bán kính ~5px đã tính toán
    params.maxArea = 10000.0f;

    // C. Lọc theo độ tròn (Circularity = 4*pi*Area / Perimeter^2)
    params.filterByCircularity = true;
    params.minCircularity = 0.5f; // Chấp nhận độ tròn từ 80% trở lên

    // D. Lọc theo độ thuôn dài (Inertia): Loại bỏ đốm sáng bị kéo dẹt
    params.filterByInertia = false;
    params.minInertiaRatio = 0.1f;

    // E. Lọc theo độ lồi (Convexity): Loại bỏ hình khuyết dạng bán nguyệt
    params.filterByConvexity = false;
    params.minConvexity = 0.8f;

    // 2. Khởi tạo con trỏ thông minh (Smart Pointer) chứa bộ dò
    cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);

    int kernelSize = 5;
    Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(kernelSize, kernelSize));
    // Perform morphological opening
    morphologyEx(img, img, MORPH_OPEN, kernel);

int brightness_thrs = 240;
    double curv_thrs = 0.6;
    double area_thrs = 20;

cap.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G'));
    cap.set(CAP_PROP_FRAME_WIDTH, 1280);
    cap.set(CAP_PROP_FRAME_HEIGHT, 720);
    cap.set(CAP_PROP_FPS, 30);

    // cap.set(cv::CAP_PROP_AUTOFOCUS, 0); 
    // Thiết lập mốc lấy nét bằng tay (Giá trị từ 0 đến 255 tùy loại webcam, ví dụ 50 là mốc 3-4 mét)
    // cap.set(cv::CAP_PROP_FOCUS, 50); 

    // 2. Tắt Auto-White Balance
    cap.set(cv::CAP_PROP_AUTO_WB, 0);

    // 3. Tắt Auto-Exposure (Phơi sáng tự động)
    cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 1); // Trong V4L2: 1 là Manual, 3 là Auto
    // Đặt thời gian phơi sáng cực thấp để chống Motion Blur (Giá trị tùy loại cam)
    // cap.set(cv::CAP_PROP_EXPOSURE, 150);

Img4: ~20cm before the line
Img5: Right before the line
Img6: Right after the line
Img7: ~20cm after the line
*/

using namespace std;
using namespace cv;

typedef vector<vector<Point> > Contours;
typedef SimpleBlobDetector::Params blobParams;


int take_photo() {
    cv::VideoCapture cap(0, cv::CAP_V4L2); 

    if (!cap.isOpened()) {
        std::cerr << "LỖI: Không thể mở camera." << std::endl;
        return -1;
    }

    // 2. Ép cấu hình khớp 100% với phần cứng (dựa trên lệnh v4l2-ctl)
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
    cap.set(cv::CAP_PROP_FPS, 30);

    // In ra kiểm tra xem cấu hình đã nhận chưa
    std::cout << "Cau hinh hien tai: " 
              << cap.get(cv::CAP_PROP_FRAME_WIDTH) << "x" 
              << cap.get(cv::CAP_PROP_FRAME_HEIGHT) << " @ " 
              << cap.get(cv::CAP_PROP_FPS) << "fps" << std::endl;

    cv::Mat frame;
    cv::Mat grayFrame;

    // Bỏ qua 5 frame đầu tiên để sensor ổn định ánh sáng
    for(int i = 0; i < 5; i++) {
        cap.read(frame);
    }

    while (true) {
        cap.read(frame); 
        
        if (frame.empty()) {
            std::cerr << "Mat ket noi luong video!" << std::endl;
            break; 
        }

        cv::cvtColor(frame, grayFrame, cv::COLOR_BGR2GRAY);

        cv::imshow("1. Anh mau (1280x720)", frame); 
        cv::imshow("2. Anh xam", grayFrame);

        if ((char)cv::waitKey(0) == 27) { // Nhan ESC de thoat
            break;
        }
    }

    cap.release();
    cv::destroyAllWindows();

    return 1;
}

vector<Point> single_threshold_blob_detection(string img, int thrs, double curv_thrs, double area_thrs, Contours& contour) {
    Mat frame = imread(img, IMREAD_GRAYSCALE);
    Mat bi_frame;
    vector<Point> centers;

    if(frame.empty()) {
        cout << "Failed to load image" << endl;
        return centers;
    }

    // Apply thresholding
    threshold(frame, bi_frame, thrs, 255, THRESH_BINARY);
    imshow("After thresholding", bi_frame);
    waitKey(0);

    // Perform morphological opening to remove connected component noise
    // Initiate the kernel
    int kernelSize = 5;
    Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(kernelSize, kernelSize));
    // Perform morphological opening
    // morphologyEx(bi_frame, bi_frame, MORPH_OPEN, kernel);
    // imshow("After morphological opening", bi_frame);
    // waitKey(0);

    // Perform contour extraction
    findContours(bi_frame, contour, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    // Filtering by curvature
    for(int i = contour.size() - 1; i >= 0; i--) {
        double area = contourArea(contour[i]);
        double perimeter = arcLength(contour[i], true);
        double curvature = (4 * CV_PI * area) / (perimeter * perimeter);
        cout << curvature << " - " << area << endl;
        
        if(curvature < curv_thrs || area < area_thrs) {
            // The blob is not round enough
            cout << "Elimination reason: " << (curvature < curv_thrs) << ", " << (area < area_thrs) << endl;
            contour.erase(contour.begin() + i);
        }
    }

    cout << "Number of contour after filtering: " << contour.size() << endl;

    // Find center of mass
    for(int i = 0; i < contour.size(); i++) {
        Moments M = moments(contour[i]);
        Point center;

        if (M.m00 != 0) {
            center.x = (int) (M.m10 / M.m00);
            center.y = (int) (M.m01 / M.m00);
            
            //std::cout << "Toa do tam: X=" << center.x << ", Y=" << center.y << std::endl;
        } else {
            center.x = 0;
            center.y = 0;
        }
        centers.push_back(center);
    }

    return centers;
}

vector<cv::Point2f> detect_mocap_blobs(Mat img) {
    cvtColor(img, img, COLOR_BGR2GRAY);

    blobParams params;

    // --- CẤU HÌNH THRESHOLDING ĐA NGƯỠNG ---
    params.minThreshold = 200;
    params.maxThreshold = 250;
    params.thresholdStep = 10; // Cắt lát mỗi 10 đơn vị cường độ sáng

    // --- BỘ LỌC HÌNH HỌC (GEOMETRIC FILTERS) ---
    // A. Lọc theo màu sắc: Chỉ tìm các đốm sáng trắng (pixel tiến về 255)
    params.filterByColor = true;
    params.blobColor = 255;

    // B. Lọc theo diện tích: Loại bỏ nhiễu hạt tiêu và vệt lóa lớn
    params.filterByArea = true;
    params.minArea = 20.0f;    // Ngưỡng an toàn cho bán kính ~5px đã tính toán
    params.maxArea = 10000.0f;

    // C. Lọc theo độ tròn (Circularity = 4*pi*Area / Perimeter^2)
    params.filterByCircularity = true;
    params.minCircularity = 0.8f; // Chấp nhận độ tròn từ 80% trở lên

    // D. Lọc theo độ thuôn dài (Inertia): Loại bỏ đốm sáng bị kéo dẹt
    params.filterByInertia = true;
    params.minInertiaRatio = 0.8f;

    // E. Lọc theo độ lồi (Convexity): Loại bỏ hình khuyết dạng bán nguyệt
    params.filterByConvexity = false;
    params.minConvexity = 0.8f;

    // 2. Khởi tạo con trỏ thông minh (Smart Pointer) chứa bộ dò
    cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);
    
    // 3. Khai báo mảng KeyPoint và thực thi lệnh nhận diện
    std::vector<cv::KeyPoint> keypoints;
    detector->detect(img, keypoints);

    // 4. Trích xuất tọa độ Sub-pixel từ mảng KeyPoint sang mảng Point2f
    std::vector<cv::Point2f> centers;
    centers.reserve(keypoints.size()); // Cấp phát bộ nhớ trước để tối ưu tốc độ
    
    for (size_t i = 0; i < keypoints.size(); i++) {
        // Thuộc tính .pt chứa tọa độ tâm (x, y) định dạng float
        centers.push_back(keypoints[i].pt); 
    }

    return centers;
}

vector<cv::Point2f> detect_mocap_blobs_inspect(Mat img) {
    cvtColor(img, img, COLOR_BGR2GRAY);

    blobParams params;

    // --- CẤU HÌNH THRESHOLDING ĐA NGƯỠNG ---
    params.minThreshold = 150;
    params.maxThreshold = 240;
    params.thresholdStep = 10; // Cắt lát mỗi 10 đơn vị cường độ sáng

    // --- BỘ LỌC HÌNH HỌC (GEOMETRIC FILTERS) ---
    // A. Lọc theo màu sắc: Chỉ tìm các đốm sáng trắng (pixel tiến về 255)
    params.filterByColor = true;
    params.blobColor = 255;

    // B. Lọc theo diện tích: Loại bỏ nhiễu hạt tiêu và vệt lóa lớn
    params.filterByArea = true;
    params.minArea = 50.0f;    // Ngưỡng an toàn cho bán kính ~5px đã tính toán
    params.maxArea = 10000.0f;

    // C. Lọc theo độ tròn (Circularity = 4*pi*Area / Perimeter^2)
    params.filterByCircularity = true;
    params.minCircularity = 0.8f; // Chấp nhận độ tròn từ 80% trở lên

    // D. Lọc theo độ thuôn dài (Inertia): Loại bỏ đốm sáng bị kéo dẹt
    params.filterByInertia = true;
    params.minInertiaRatio = 0.8f;

    // E. Lọc theo độ lồi (Convexity): Loại bỏ hình khuyết dạng bán nguyệt
    params.filterByConvexity = false;
    params.minConvexity = 0.8f;

    // 2. Khởi tạo con trỏ thông minh (Smart Pointer) chứa bộ dò
    cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);

    imshow("After grayscalling", img);
    waitKey(0);

    // 3. Khai báo mảng KeyPoint và thực thi lệnh nhận diện
    std::vector<cv::KeyPoint> keypoints;
    detector->detect(img, keypoints);

    // 4. Trích xuất tọa độ Sub-pixel từ mảng KeyPoint sang mảng Point2f
    std::vector<cv::Point2f> centers;
    centers.reserve(keypoints.size()); // Cấp phát bộ nhớ trước để tối ưu tốc độ
    
    for (size_t i = 0; i < keypoints.size(); i++) {
        // Thuộc tính .pt chứa tọa độ tâm (x, y) định dạng float
        centers.push_back(keypoints[i].pt); 
    }

    return centers;
}


vector<Point> single_threshold_blob_detection_4vid(Mat img, int brightness_thrs, double curv_thrs, double area_thrs, Contours& contour) {
    vector<Point> centers;

    // Grayscaling
    if(img.empty()) return centers;
    cvtColor(img, img, COLOR_BGR2GRAY);

    // Apply thresholding
    threshold(img, img, brightness_thrs, 255, THRESH_BINARY);

    // Perform morphological opening to remove connected component noise
    // Initiate the kernel
    
    int kernelSize = 5;
    Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(kernelSize, kernelSize));
    // Perform morphological opening
    // morphologyEx(img, img, MORPH_OPEN, kernel);
    

    // Perform contour extraction
    findContours(img, contour, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    // Filtering by curvature
    for(int i = contour.size() - 1; i >= 0; i--) {
        double area = contourArea(contour[i]);
        double perimeter = arcLength(contour[i], true);
        double curvature = (4 * CV_PI * area) / (perimeter * perimeter);
        
        if(curvature < curv_thrs || area < area_thrs) {
            // The blob is not round enough
            contour.erase(contour.begin() + i);
        }
    }

    // Find center of mass
    for(int i = 0; i < contour.size(); i++) {
        Moments M = moments(contour[i]);
        Point center;

        if (M.m00 != 0) {
            center.x = (int) (M.m10 / M.m00);
            center.y = (int) (M.m01 / M.m00);
                    } else {
            center.x = 0;
            center.y = 0;
        }
        centers.push_back(center);
    }

    return centers;
}

/*
int main(int argc, char** argv) {
    int brightness_thrs = 240;
    double curv_thrs = 0.6;
    double area_thrs = 20;
    Contours contour;
    vector<Point> centers;

    if(argc > 1) {
        switch(argc) {
            case 2:
                brightness_thrs = stoi(argv[1]);
                break;
            case 3:
                brightness_thrs = stoi(argv[1]);
                curv_thrs = stod(argv[2]);
                break;
            case 4:
                brightness_thrs = stoi(argv[1]);
                curv_thrs = stod(argv[2]);
                area_thrs = stod(argv[3]);
                break;
            default:
                brightness_thrs = stoi(argv[1]);
                curv_thrs = stod(argv[2]);
                area_thrs = stod(argv[3]);
        }
    }

    Mat img = imread("../img/color/img_2.png");

    auto start = std::chrono::steady_clock::now();
    centers = single_threshold_blob_detection("../img/color/img_2.png", brightness_thrs, curv_thrs, area_thrs, contour);
    auto end = std::chrono::steady_clock::now();
    cout << "Processing execution time: " << (std::chrono::duration_cast<chrono::milliseconds>(end - start)).count() << endl;

    drawContours(img, contour, -1, Scalar(0, 0, 255), 3, LINE_AA);
    for(int i = 0; i < centers.size(); i++) {
        circle(img, centers[i], 3, Scalar(0, 0, 255), -1, LINE_AA);
    }

    return 0;
}
*/


int main(int argc, char** argv) {
    VideoCapture cap(0, CAP_V4L2);

    if(!cap.isOpened()) {
        cerr << "Unable to open camera" << endl;
        return -1;
    }

    int brightness_thrs = 240;
    double curv_thrs = 0.5;
    double area_thrs = 20;
    Contours contour;
    vector<Point> centers;
    vector<Point2f> centers2f;

    
    if(argc > 1) {
        switch(argc) {
            case 2:
                brightness_thrs = stoi(argv[1]);
                break;
            case 3:
                brightness_thrs = stoi(argv[1]);
                curv_thrs = stod(argv[2]);
                break;
            case 4:
                brightness_thrs = stoi(argv[1]);
                curv_thrs = stod(argv[2]);
                area_thrs = stod(argv[3]);
                break;
            default:
                brightness_thrs = stoi(argv[1]);
                curv_thrs = stod(argv[2]);
                area_thrs = stod(argv[3]);
        }
    }
    

    cap.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G'));
    cap.set(CAP_PROP_FRAME_WIDTH, 1280);
    cap.set(CAP_PROP_FRAME_HEIGHT, 720);
    cap.set(CAP_PROP_FPS, 120);

    // cap.set(cv::CAP_PROP_AUTOFOCUS, 0); 
    // Thiết lập mốc lấy nét bằng tay (Giá trị từ 0 đến 255 tùy loại webcam, ví dụ 50 là mốc 3-4 mét)
    // cap.set(cv::CAP_PROP_FOCUS, 50); 

    // 2. Tắt Auto-White Balance
    cap.set(cv::CAP_PROP_AUTO_WB, 0);

    // 3. Tắt Auto-Exposure (Phơi sáng tự động)
    cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 1); // Trong V4L2: 1 là Manual, 3 là Auto
    // Đặt thời gian phơi sáng cực thấp để chống Motion Blur (Giá trị tùy loại cam)
    // cap.set(cv::CAP_PROP_EXPOSURE, 150);

    Mat frame;
    /*
    single_threshold_blob_detection("../img/color/img_8.png", brightness_thrs, curv_thrs, area_thrs, contour);
    // single_threshold_blob_detection("../img/color/img_9.png", brightness_thrs, curv_thrs, area_thrs, contour);
    // single_threshold_blob_detection("../img/color/img_10.png", brightness_thrs, curv_thrs, area_thrs, contour);

    return 0;
    */

    /* 
    std::this_thread::sleep_for(std::chrono::milliseconds(10000));
    cap.read(frame);
    imwrite("../img/color/img_10.png", frame);
    
    return 0;
    */
    
    /*
    frame = imread(argv[1]);
    centers2f = detect_mocap_blobs_inspect(frame);
    cout << "Centers found: " << centers2f.size() << endl;
    
    for(int i = 0; i < centers2f.size(); i++) {
        Point centerInt(static_cast<int>(centers2f[i].x), static_cast<int>(centers2f[i].y));
            circle(frame, centerInt, 20, Scalar(0, 0, 255), -1, LINE_AA);
    }
    imshow("Result", frame);
    waitKey(0);

    return 0;
    */
    
    

    while(true) {
        bool read = cap.read(frame);
        if(!read || frame.empty()) break;

        // Process the image
        
        // centers = single_threshold_blob_detection_4vid(frame, brightness_thrs, curv_thrs, area_thrs, contour);
        centers2f = detect_mocap_blobs(frame);
        for(int i = 0; i < centers2f.size(); i++) {
            Point centerInt(static_cast<int>(centers2f[i].x), static_cast<int>(centers2f[i].y));
            circle(frame, centerInt, 20, Scalar(0, 0, 255), -1, LINE_AA);
            cout << "Centers: " << centerInt.x << ", " << centerInt.y << endl;
        }
        
        /*
        centers = single_threshold_blob_detection_4vid(frame, brightness_thrs, curv_thrs, area_thrs, contour);
        for(int i = 0; i < centers.size(); i++) {
            circle(frame, centers[i], 20, Scalar(0, 0, 255), -1, LINE_AA);
            cout << "Centers: " << centers[i].x << ", " << centers[i].y << endl;
        }
        */

        // Display the frame
        imshow("Result", frame);

        if(waitKey(1) == 'q') break;
    }

    cap.release();

    return 0;
}


/*
int main(int argc, char** argv) {
    string gray_img = "../img/gray/gray_1.png";
    Mat img = imread(gray_img, COLOR_GRAY2BGR);

    cout << img.type() << endl;
    imshow("", img);
    waitKey(0);

    Contours contour;
    vector<Point> centers = single_threshold_blob_detection(gray_img, 240, 0.6, 20, contour);
    cout << "Number of centers: " << centers.size() << endl;

    // Draw contours
    drawContours(img, contour, -1, Scalar(0, 0, 255), 2, LINE_AA);
    // Draw centers
    for(int i = 0; i < centers.size(); i++) {
        circle(img, centers[i], 3, Scalar(0, 0, 255), -1, LINE_AA);
    }
    imshow("Result", img);
    waitKey(0);

    return 0;
}
*/

/*
int main(int argc, char** argv) {
    // Đọc ảnh đầu vào với cờ 1 kênh
    std::string img_path = "../img/gray/gray_1.png";
    cv::Mat grayFrame = cv::imread(img_path, cv::IMREAD_GRAYSCALE);

    if(grayFrame.empty()) {
        std::cerr << "Khong the doc file anh" << std::endl;
        return -1;
    }
    imshow("Original image", grayFrame);
    waitKey(0);

    // Thực thi nhận diện
    std::vector<cv::Point2f> centers = detect_mocap_blobs(grayFrame);
    cout << "Number of centers: " << centers.size() << endl;

    // Chuẩn bị ma trận 3 kênh để hiển thị màu
    cv::Mat resultImage;
    cv::cvtColor(grayFrame, resultImage, cv::COLOR_GRAY2BGR);

    // VẼ KẾT QUẢ
    // Có thể dùng lại vòng lặp cv::circle thủ công với mảng centers nếu muốn vẽ dấu chấm.
    // Hoặc vẽ chấm tâm trực tiếp:
    for(size_t i = 0; i < centers.size(); i++) {
        // Ép kiểu float về int để vẽ trên pixel
        cv::Point centerInt(static_cast<int>(centers[i].x), static_cast<int>(centers[i].y));
        cv::circle(resultImage, centerInt, 3, cv::Scalar(0, 0, 255), -1, cv::LINE_AA);
    }

    // Hiển thị
    cv::imshow("Ket qua nhan dien Blob", resultImage);
    cv::waitKey(0);

    return 0;
}
*/


/*
#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include <string>
#include <chrono>
#include <thread>

using namespace std;
using namespace cv;

typedef SimpleBlobDetector::Params blobParams;

typedef struct {
    Point2f center; // (x, y)
    Point2f velocity; // (dx, dy)
    int id;
} StateMarker2D;

class Marker2DTracker {
    private:
        vector<StateMarker2D> prevMarkers; // Used to predict movement

        float pred_x(const StateMarker2D& marker) {
            return marker.center.x + marker.velocity.x;
        }

        float pred_y(const StateMarker2D& marker) {
            return marker.center.y + marker.velocity.y;
        }
    public:
        vector<StateMarker2D> initIdentification(vector<Point2f> centers) {
            this->prevMarkers.reserve(centers.size());

            for(int i = 0; i < centers.size(); i++) {
                StateMarker2D e = {.center = centers[i], .velocity = Point2f(0.0f, 0.0f), .id = i};
                this->prevMarkers.push_back(e);
            }

            return this->prevMarkers;
        }

        vector<StateMarker2D> identifyBlobs(vector<Point2f> centers) {
            if(this->prevMarkers.size() <= 0) { // Not initialized
                return this->initIdentification(centers);
            }

            // Predict the current position of the blobs
            int prevMarkersCount = this->prevMarkers.size();
            int blobCount = centers.size();

            int i = 0, j = 0;
            for(; i < prevMarkersCount; i++) {
                // Calculate a prediction based on the previous state of the marker
                Point2f pred = Point2f(pred_x(this->prevMarkers[i]), pred_y(this->prevMarkers[i]));
                float min_dist = 1000000;
                int closest_blob_idx = -1;

                for(; j < blobCount; j++) {
                    // Check the prediction against the centers of the blobs. Blob with the closest center to the prediction gets assigned the id
                    Point2f diff = centers[i] - pred;
                    float dist = diff.x * diff.x + diff.y * diff.y; // Calculate the Eucledian distance

                    if(dist < min_dist) {
                        min_dist = dist;
                        closest_blob_idx = j;
                    }
                }

                // Update the marker
                this->prevMarkers[i].velocity = Point2f(
                                    centers[closest_blob_idx].x - this->prevMarkers[i].center.x,
                                    centers[closest_blob_idx].y - this->prevMarkers[i].center.y);
                this->prevMarkers[i].center = centers[closest_blob_idx];
            }

            return this->prevMarkers;
        }
};

vector<cv::Point2f> detect_mocap_blobs(Mat img, Ptr<SimpleBlobDetector> detector, vector<KeyPoint> keypoints) {
    cvtColor(img, img, COLOR_BGR2GRAY);

    
    detector->detect(img, keypoints);

    // 4. Trích xuất tọa độ Sub-pixel từ mảng KeyPoint sang mảng Point2f
    std::vector<cv::Point2f> centers;
    centers.reserve(keypoints.size()); // Cấp phát bộ nhớ trước để tối ưu tốc độ
    
    for (size_t i = 0; i < keypoints.size(); i++) {
        // Thuộc tính .pt chứa tọa độ tâm (x, y) định dạng float
        centers.push_back(keypoints[i].pt); 
    }

    return centers;
}

int main(int argc, char** argv) {
    VideoCapture cap(0, CAP_V4L2);

    if(!cap.isOpened()) {
        cerr << "Unable to open camera" << endl;
        return -1;
    }

    vector<Point2f> centers2f;
    vector<StateMarker2D> markers;
    

    cap.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G'));
    cap.set(CAP_PROP_FRAME_WIDTH, 1280);
    cap.set(CAP_PROP_FRAME_HEIGHT, 720);
    cap.set(CAP_PROP_FPS, 30);

    // cap.set(cv::CAP_PROP_AUTOFOCUS, 0); 
    // Thiết lập mốc lấy nét bằng tay (Giá trị từ 0 đến 255 tùy loại webcam, ví dụ 50 là mốc 3-4 mét)
    // cap.set(cv::CAP_PROP_FOCUS, 50); 

    // 2. Tắt Auto-White Balance
    cap.set(cv::CAP_PROP_AUTO_WB, 0);

    // 3. Tắt Auto-Exposure (Phơi sáng tự động)
    cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 1); // Trong V4L2: 1 là Manual, 3 là Auto
    // Đặt thời gian phơi sáng cực thấp để chống Motion Blur (Giá trị tùy loại cam)
    // cap.set(cv::CAP_PROP_EXPOSURE, 150);

    Mat frame;

    blobParams params;

    // --- CẤU HÌNH THRESHOLDING ĐA NGƯỠNG ---
    params.minThreshold = 200;
    params.maxThreshold = 250;
    params.thresholdStep = 10; // Cắt lát mỗi 10 đơn vị cường độ sáng

    // --- BỘ LỌC HÌNH HỌC (GEOMETRIC FILTERS) ---
    // A. Lọc theo màu sắc: Chỉ tìm các đốm sáng trắng (pixel tiến về 255)
    params.filterByColor = true;
    params.blobColor = 255;

    // B. Lọc theo diện tích: Loại bỏ nhiễu hạt tiêu và vệt lóa lớn
    params.filterByArea = true;
    params.minArea = 20.0f;    // Ngưỡng an toàn cho bán kính ~5px đã tính toán
    params.maxArea = 10000.0f;

    // C. Lọc theo độ tròn (Circularity = 4*pi*Area / Perimeter^2)
    params.filterByCircularity = true;
    params.minCircularity = 0.8f; // Chấp nhận độ tròn từ 80% trở lên

    // D. Lọc theo độ thuôn dài (Inertia): Loại bỏ đốm sáng bị kéo dẹt
    params.filterByInertia = true;
    params.minInertiaRatio = 0.8f;

    // E. Lọc theo độ lồi (Convexity): Loại bỏ hình khuyết dạng bán nguyệt
    params.filterByConvexity = false;
    params.minConvexity = 0.8f;

    // 2. Khởi tạo con trỏ thông minh (Smart Pointer) chứa bộ dò
    cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);
    
    // 3. Khai báo mảng KeyPoint và thực thi lệnh nhận diện
    std::vector<cv::KeyPoint> keypoints;

    Mat test = imread("../img/color/img_4.png");
    auto start = std::chrono::steady_clock::now();
    detect_mocap_blobs(test, detector, keypoints);
    auto end = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

    std::cout << "Execution time: " << duration.count() << " microseconds" << std::endl;

    return 0;

    Marker2DTracker tracker;

    while(true) {
        bool read = cap.read(frame);
        if(!read || frame.empty()) break;

        // Process the image
        
        // centers2f = detect_mocap_blobs(frame);

        markers = tracker.identifyBlobs(centers2f);

        for(int i = 0; i < centers2f.size(); i++) {
            Point centerInt(static_cast<int>(markers[i].center.x), static_cast<int>(markers[i].center.y));
            circle(frame, centerInt, 20, Scalar(0, 0, 255), -1, LINE_AA);
            std::string text_id = "ID: " + std::to_string(markers[i].id) + "; " + to_string(centerInt.x) + " - " + to_string(centerInt.y);

            // 3. Tính toán vị trí đặt chữ (Dịch ra xa tâm một chút để không bị đè lên nhau)
            cv::Point2f text_position(centerInt.x + 10, centerInt.y - 10);

            // 4. In chữ lên màn hình
            cv::putText(frame, 
                        text_id,                     // Nội dung chữ
                        text_position,               // Tọa độ góc dưới bên trái của chữ
                        cv::FONT_HERSHEY_SIMPLEX,    // Kiểu font chữ (OpenCV cung cấp sẵn vài font)
                        0.6,                         // Kích thước chữ (Scale)
                        cv::Scalar(0, 0, 255),       // Màu sắc (B, G, R) -> Màu Đỏ
                        2);
            cout << "Centers: " << centerInt.x << ", " << centerInt.y << endl;
        }

        // Display the frame
        imshow("Result", frame);

        if(waitKey(1) == 'q') break;
    }

    cap.release();

    return 0;
}
*/
