#pragma once

#include "../../lib/header/lib.hpp"
#include "../../lib/header/module.hpp"

#include <iostream>
#include <filesystem>

using namespace std;
using namespace cv;

namespace fs = std::filesystem;

template <typename I>
class CalibModule : public IModule<I, CalibOutputResult> {
    private:
        CALIB_MODE calibMode;
        CalibBoardDesc desc;
        string dataFolderIn;
        string dataFolderEx;
        int targetCameraID;
        string resultFolder; // Result for both ex and in calib are put here

        void runIntrinsicCalib();
        void runExtrinsicCalib();
    public:
        CalibModule();
        ~CalibModule();

        void setResultFolderDest(string path);
        void setState(int newState) override;
        void runModule() override;
        bool parseCLIInput(const CLIOutputResult inputData);
};

template <typename I>
void CalibModule<I>::setResultFolderDest(string path) {
    if (!path.empty()) {
        this->resultFolder = path;
    }
}

template <typename I>
CalibModule<I>::CalibModule() : IModule<I, CalibOutputResult>(20) {

}

template <typename I>
CalibModule<I>::~CalibModule() {
    
}

template <typename I>
void CalibModule<I>::setState(int newState) {
    this->moduleState = newState;
}

template <typename I>
void CalibModule<I>::runIntrinsicCalib() {
    printf("[CALIB Intrinsic] Intrinsic Calibration Running...\n");
    
    // Create output folder
    string outFolder = this->resultFolder + "/intrinsic_result_cam_" + to_string(this->targetCameraID);
    if (!fs::exists(outFolder)) fs::create_directory(outFolder);

    // Setup 3D World Coord System (WCS)
    vector<Point3f> objp;
    for(int i = 0; i < this->desc.row; i++) {
        for(int j = 0; j < this->desc.col; j++) {
            objp.push_back(Point3f(j * this->desc.checkSize, i * this->desc.checkSize, 0.0f));
        }
    }

    vector<vector<Point3f>> objpoints; 
    vector<vector<Point2f>> imgpoints; 
    Size imageSize;

    // Scan and process the imgs
    for (const auto & entry : fs::directory_iterator(this->dataFolderIn)) {
        if (entry.path().extension() == ".jpg" || entry.path().extension() == ".png") {
            Mat img = imread(entry.path().string());
            if (img.empty()) continue;
            imageSize = img.size();

            Mat gray;
            cvtColor(img, gray, COLOR_BGR2GRAY);

            vector<Point2f> corners;
            Size boardSize(this->desc.col, this->desc.row);
            
            bool found = findChessboardCorners(gray, boardSize, corners, 
                            CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FAST_CHECK | CALIB_CB_NORMALIZE_IMAGE);

            if (found) {
                cornerSubPix(gray, corners, Size(11, 11), Size(-1, -1),
                                 TermCriteria(TermCriteria::EPS + TermCriteria::MAX_ITER, 30, 0.001));

                objpoints.push_back(objp);
                imgpoints.push_back(corners);

                // Draw and save image
                drawChessboardCorners(img, boardSize, corners, found);
                string savePath = outFolder + "/" + entry.path().filename().string();
                imwrite(savePath, img);
            }
        }
    }

    if (objpoints.empty()) {
        printf("[CALIB Intrinsic] No image found!\n");
        return;
    }

    // Calc K and distortion
    Mat K, dist;
    vector<Mat> rvecs, tvecs;
    double rms = calibrateCamera(objpoints, imgpoints, imageSize, K, dist, rvecs, tvecs);

    // Calc reprojection error
    double totalErr = 0;
    int totalPoints = 0;
    for (size_t i = 0; i < objpoints.size(); ++i) {
        vector<Point2f> imgpoints2;
        projectPoints(objpoints[i], rvecs[i], tvecs[i], K, dist, imgpoints2);
        double err = norm(imgpoints[i], imgpoints2, NORM_L2);
        totalErr += err * err;
        totalPoints += objpoints[i].size();
    }
    double meanError = sqrt(totalErr / totalPoints);

    // Save calib results
    FileStorage fs(outFolder + "/camera_params_cam_" + to_string(this->targetCameraID) + ".yml", FileStorage::WRITE);
    fs << "K" << K;
    fs << "dist" << dist;
    fs << "RMS" << rms;
    fs << "MeanReprojectionError" << meanError;
    fs.release();

    printf("[CALIB Intrinsic] Calib completed. RMS: %lf | Mean Err: %lf\n", rms, meanError);
    printf("[CALIB Intrinsic] Results saved to: %s", outFolder);
}

template <typename I>
void CalibModule<I>::runExtrinsicCalib() {
    printf("[CALIB Extrinsic] Extrinsic Calibration Running...\n");
    
    // Create output folder
    string outFolder = this->resultFolder + "/extrinsic_result_cam_" + to_string(this->targetCameraID); // Use dataFolderIn to make it more convinient to access both intrinsic and extrinsic results
    if (!fs::exists(outFolder)) fs::create_directory(outFolder);

    // Load intrinsic calib results
    Mat K, dist;
    string intrinsicPath = this->resultFolder + "/intrinsic_result" + "/camera_params_cam_" + to_string(this->targetCameraID) + ".yml";
    if (fs::exists(intrinsicPath)) {
        FileStorage fsIn(intrinsicPath, FileStorage::READ);
        fsIn["K"] >> K;
        fsIn["dist"] >> dist;
        fsIn.release();
    } else {
        printf("[CALIB Extrinsic] File camera_params.yml for intrinsic results not found!\n");
        return;
    }

    // Set up 3D WCS
    vector<Point3f> objp;
    for(int i = 0; i < this->desc.row; i++) {
        for(int j = 0; j < this->desc.col; j++) {
            objp.push_back(Point3f(0.0f, j * (this->desc.checkSize / 1000.0f), i * (this->desc.checkSize / 1000.0f))); 
        }
    }

    FileStorage fsOut(outFolder + "/extrinsic_params_cam_" + to_string(this->targetCameraID) + ".yml", FileStorage::WRITE);

    for (const auto & entry : fs::directory_iterator(this->dataFolderEx)) {
        if (entry.path().extension() == ".jpg" || entry.path().extension() == ".png") {
            Mat img = imread(entry.path().string());
            Mat gray;
            cvtColor(img, gray, COLOR_BGR2GRAY);

            vector<Point2f> corners;
            Size boardSize(this->desc.col, this->desc.row);
            
            bool found = findChessboardCorners(gray, boardSize, corners);

            if (found) {
                cornerSubPix(gray, corners, Size(3, 3), Size(-1, -1),
                                 TermCriteria(TermCriteria::EPS + TermCriteria::MAX_ITER, 30, 0.001));

                Mat rvec, tvec;
                bool success = solvePnP(objp, corners, K, dist, rvec, tvec);

                if (success) {
                    // Calculate reprojection error
                    vector<Point2f> projPoints;
                    projectPoints(objp, rvec, tvec, K, dist, projPoints);
                    double err = norm(corners, projPoints, NORM_L2) / projPoints.size();

                    Mat R;
                    Rodrigues(rvec, R);

                    Mat cameraPos = -R.t() * tvec;
                    Mat offset = (Mat_<double>(3, 1) << 0.0, 0.025, 0.025);
                    cameraPos += offset;

                    string nodeName = "img_" + entry.path().stem().string();
                    fsOut << nodeName << "{";
                    fsOut << "ReprojectionError" << err;
                    fsOut << "R_Matrix" << R;
                    fsOut << "T_Vector" << tvec;
                    fsOut << "Camera_WCS_Pos" << cameraPos;
                    fsOut << "}";

                    // Draw axis and save file
                    drawChessboardCorners(img, boardSize, corners, found);
                    drawFrameAxes(img, K, dist, rvec, tvec, 0.3f);
                    
                    string savePath = outFolder + "/" + entry.path().filename().string();
                    imwrite(savePath, img);
                }
            }
        }
    }
    fsOut.release();
    printf("[CALIB Extrinsic] Results saved to: %s", outFolder);
}

template <typename I>
bool CalibModule<I>::parseCLIInput(const CLIOutputResult inputData) {
    if (inputData.cmdOrigin == CLI_CALIB_SET) {
        this->calibMode = get<CalibSettings>(inputData.result).calibMode;
        this->desc = get<CalibSettings>(inputData.result).desc;
        this->dataFolderIn = get<CalibSettings>(inputData.result).dataFolderIn;
        this->dataFolderEx = get<CalibSettings>(inputData.result).dataFolderEx;
        this->targetCameraID = get<CalibSettings>(inputData.result).targetID;

        this->setState(get<CalibSettings>(inputData.result).calibState);

        return true;
    } else return false;
}

template <typename I>
void CalibModule<I>::runModule() {
    while(this->moduleState == CALIB_RUNNING || this->moduleState == CALIB_IDLE) {
        I inputData = this->inputData->pop();

        // Parse input
        bool dataParsed = true;
        if (inputData.cmdOrigin == CLI_CALIB_SET) {
            if(this->parseCLIInput(inputData) == false) {
                printf("[CALIB] Failed to parse input from CLI\n");
                dataParsed = false;
            }
        }

        if (dataParsed == true) {
            if (this->calibMode == CALIB_MODE_IN) {
                this->runIntrinsicCalib();
            } 
            else if (this->calibMode == CALIB_MODE_EX) {
                this->runExtrinsicCalib();
            }
        }
    }
}

