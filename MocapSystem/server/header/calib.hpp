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
        double target_rms = 0.5; 
        size_t min_images = 10; 

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
        void setConfig(string paramName, float value);
};

template <typename I>
void CalibModule<I>::setConfig(string paramName, float value) {
    if (paramName == "TARGET_RMS") {
        this->target_rms = (double) value;
    } else if (paramName == "MIN_IMAGE") {
        this->min_images = (int) value;
    }
}

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
    
    if (this->dataFolderIn.empty() || !fs::exists(this->dataFolderIn)) {
        printf("[CALIB Intrinsic] Error: Input path not found: %s!\n", this->dataFolderIn.c_str());
        return; 
    }

    // Create output folder
    string outFolder = this->resultFolder + "/intrinsic_result_cam_" + to_string(this->targetCameraID);
    if (!fs::exists(outFolder)) fs::create_directory(outFolder);

    int inner_row = this->desc.row - 1;
    int inner_col = this->desc.col - 1;

    // Setup 3D World Coord System (WCS)
    vector<Point3f> objp;
    for(int i = 0; i < inner_row; i++) {
        for(int j = 0; j < inner_col; j++) {
            objp.push_back(Point3f(j * this->desc.checkSize, i * this->desc.checkSize, 0.0f));
        }
    }

    vector<vector<Point3f>> objpoints; 
    vector<vector<Point2f>> imgpoints; 
    vector<string> valid_img_paths; 
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
            Size boardSize(inner_col, inner_row);
            
            bool found = findChessboardCorners(gray, boardSize, corners, 
                            CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FAST_CHECK | CALIB_CB_NORMALIZE_IMAGE);

            if (found) {
                cornerSubPix(gray, corners, Size(11, 11), Size(-1, -1),
                                 TermCriteria(TermCriteria::EPS + TermCriteria::MAX_ITER, 30, 0.001));

                objpoints.push_back(objp);
                imgpoints.push_back(corners);
                valid_img_paths.push_back(entry.path().filename().string());

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

    // Use R_i and T_i that calibrateCamera gives for each picture to eliminate pictures with too high individual RMS.
    // Do this process untill reaching target_rms or min_images
    Mat K, dist;
    vector<Mat> rvecs, tvecs;
    double rms = 0.0; 

    printf("[CALIB Intrinsic] Starting iterative optimization...\n");

    while (true) {
        rms = calibrateCamera(objpoints, imgpoints, imageSize, K, dist, rvecs, tvecs);
        
        printf("[CALIB Intrinsic] Current Total RMS: %lf with %zu images.\n", rms, objpoints.size());

        if (rms <= this->target_rms) {
            printf("[CALIB Intrinsic] Target RMS (<= %lf) reached. Stopping optimization.\n", target_rms);
            break;
        }
        if (objpoints.size() <= this->min_images) {
            printf("[CALIB Intrinsic] Warning: Minimum image limit (%zu) reached. Cannot remove more images.\n", min_images);
            break;
        }

        // Calculate per-view RMS and find the worst image
        double max_err = -1.0;
        int max_err_idx = -1;

        for (size_t i = 0; i < objpoints.size(); ++i) {
            vector<Point2f> projPoints;
            projectPoints(objpoints[i], rvecs[i], tvecs[i], K, dist, projPoints);
            
            double err = norm(imgpoints[i], projPoints, NORM_L2);
            double per_view_rms = err / sqrt(objpoints[i].size());

            if (per_view_rms > max_err) {
                max_err = per_view_rms;
                max_err_idx = i;
            }
        }

        // Delete garbage image
        printf("[CALIB Intrinsic] Removing worst image: '%s' | Per-view Error: %lf pixels\n", valid_img_paths[max_err_idx].c_str(), max_err);
        
        objpoints.erase(objpoints.begin() + max_err_idx);
        imgpoints.erase(imgpoints.begin() + max_err_idx);
        valid_img_paths.erase(valid_img_paths.begin() + max_err_idx);
    }

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
    fs << "ValidImageCount" << (int)objpoints.size();
    fs.release();

    printf("[CALIB Intrinsic] Calib completed. Final RMS: %lf | Mean Err: %lf\n", rms, meanError);
    printf("[CALIB Intrinsic] Results saved to: %s\n", outFolder.c_str());
}

template <typename I>
void CalibModule<I>::runExtrinsicCalib() {
    printf("[CALIB Extrinsic] Extrinsic Calibration Running...\n");
    
    if (this->dataFolderEx.empty() || !fs::exists(this->dataFolderEx)) {
        printf("[CALIB Extrinsic] Error: Input path not found: %s!\n", this->dataFolderEx.c_str());
        return; 
    }

    string outFolder = this->resultFolder + "/extrinsic_result_cam_" + to_string(this->targetCameraID); 
    if (!fs::exists(outFolder)) fs::create_directory(outFolder);

    int inner_row = this->desc.row - 1;
    int inner_col = this->desc.col - 1;

    Mat K, dist;
    string intrinsicPath = this->resultFolder + "/intrinsic_result_cam_" + to_string(this->targetCameraID) + "/camera_params_cam_" + to_string(this->targetCameraID) + ".yml";
    if (fs::exists(intrinsicPath)) {
        FileStorage fsIn(intrinsicPath, FileStorage::READ);
        fsIn["K"] >> K;
        fsIn["dist"] >> dist;
        fsIn.release();
    } else {
        printf("[CALIB Extrinsic] File camera_params.yml for intrinsic results not found!\n");
        return;
    }

    // Build WCS
    vector<Point3f> objp;
    for(int i = 0; i < inner_row; i++) {     
        for(int j = 0; j < inner_col; j++) {
            
            float x = 0.0f;
            float y = j * (this->desc.checkSize / 1000.0f);
            
            float z = (inner_row - 1 - i) * (this->desc.checkSize / 1000.0f);
            
            objp.push_back(Point3f(x, y, z)); 
        }
    }
    
    /*
    vector<Point3f> objp;
    for(int i = 0; i < inner_row; i++) {
        for(int j = 0; j < inner_col; j++) {
            objp.push_back(Point3f(0.0f, j * (this->desc.checkSize / 1000.0f), i * (this->desc.checkSize / 1000.0f))); 
        }
    }
    */

    FileStorage fsOut(outFolder + "/extrinsic_params_cam_" + to_string(this->targetCameraID) + ".yml", FileStorage::WRITE);
    
    bool foundExtrinsic = false; 

    for (const auto & entry : fs::directory_iterator(this->dataFolderEx)) {
        if (entry.path().extension() == ".jpg" || entry.path().extension() == ".png") {
            Mat img = imread(entry.path().string());
            Mat gray;
            cvtColor(img, gray, COLOR_BGR2GRAY);

            vector<Point2f> corners;
            Size boardSize(inner_col, inner_row);
            
            bool found = findChessboardCorners(gray, boardSize, corners);

            if (found) {
                cornerSubPix(gray, corners, Size(5, 5), Size(-1, -1),
                                 TermCriteria(TermCriteria::EPS + TermCriteria::MAX_ITER, 30, 0.001));

                Mat rvec, tvec;
                bool success = solvePnP(objp, corners, K, dist, rvec, tvec, false, SOLVEPNP_ITERATIVE);

                if (success) {
                    // Calculate reprojection error
                    vector<Point2f> projPoints;
                    projectPoints(objp, rvec, tvec, K, dist, projPoints);
                    // double err = norm(corners, projPoints, NORM_L2) / projPoints.size();
                    double mean_err = 0;
                    double rms_err = 0;
                    for (size_t k = 0; k < corners.size(); k++) {
                        mean_err += norm(corners[k] - projPoints[k]);
                        rms_err += pow(norm(corners[k] - projPoints[k]), 2);
                    }
                    mean_err /= corners.size();
                    rms_err = sqrt(rms_err / corners.size());

                    Mat R;
                    Rodrigues(rvec, R);

                    Mat cameraPos = -R.t() * tvec;

                    string nodeName = "img_" + entry.path().stem().string();
                    fsOut << nodeName << "{";
                    fsOut << "MeanReprojectionError" << mean_err;
                    fsOut << "RMSReprojectionError" << rms_err;
                    fsOut << "R_Matrix" << R;
                    fsOut << "T_Vector" << tvec;
                    fsOut << "Camera_WCS_Pos" << cameraPos;
                    fsOut << "Distance" << norm(cameraPos);
                    fsOut << "}";

                    // Draw axis and save file
                    drawChessboardCorners(img, boardSize, corners, found);
                    drawFrameAxes(img, K, dist, rvec, tvec, 0.3f);
                    
                    string savePath = outFolder + "/" + entry.path().filename().string();
                    imwrite(savePath, img);
                    
                    foundExtrinsic = true;
                    break; 
                }
            }
        }
    }
    fsOut.release();
    
    // SỬA LỖI 2: In log an toàn và check thành công
    if (foundExtrinsic) {
        printf("[CALIB Extrinsic] Results saved to: %s\n", outFolder.c_str());
    } else {
        printf("[CALIB Extrinsic] Failed to solve PnP. No valid chessboard found in folder!\n");
    }
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
    while(this->moduleState != CALIB_STOP) { 
        I inputData = this->inputData->pop();

        bool dataParsed = false;
        if (inputData.cmdOrigin == CLI_CALIB_SET) {
            if(this->parseCLIInput(inputData) == true) {
                dataParsed = true;
            } else {
                printf("[CALIB] Failed to parse input from CLI\n");
            }
        }

        if (dataParsed == true && this->moduleState == CALIB_START) {
            if (this->calibMode == CALIB_MODE_IN) {
                this->runIntrinsicCalib();
            } 
            else if (this->calibMode == CALIB_MODE_EX) {
                this->runExtrinsicCalib();
            }
            
            this->moduleState = CALIB_IDLE; 
        }
    }
}

