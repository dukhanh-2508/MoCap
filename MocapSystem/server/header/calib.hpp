#pragma once

#pragma once

#include <variant>
#include <string>
#include <vector>
#include <filesystem>

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>

#include "main.hpp"
#include "cmdI.hpp"

enum CALIB_BLOCK_STATE {
    CALIB_IDLE = 0,
    CALIB_RUNNING = 1,
    CALIB_STOP = 2,
    CALIB_END = 3,
    CALIB_UNKNOWN = 4,
    CALIB_START = 5,
};

enum CALIB_MODE {
    CALIB_MODE_IN = 0,
    CALIB_MODE_EX = 1,
};

// Struct to describe the checker board used in calib
typedef struct {
    int row; // amount of rows of the board, in checker square amount
    int col; // amount of cols of the board, in checker square amount
    int checkSize; // Length of a side of a checker square, in mm
} CalibBoardDesc;

typedef struct {

} CalibNotiPayload;

typedef struct {
    
} CalibOutputResult;

template <typename I>
class CalibModule : public IModule<I, CalibOutputResult> {
    private:
        CALIB_MODE calibMode;
        CalibBoardDesc desc;
        string dataFolderIn;
        string dataFolderEx;

        void runIntrinsicCalib();
        void runExtrinsicCalib();
    public:
        void setState(CALIB_BLOCK_STATE newState) override;
        void runModule() override;
        bool parseCLIInput(const CLIOutputResult inputData);
};
