#include "../header/triangulate.hpp"

#include <iostream>

using namespace std;
using namespace cv;

template <typename I>
void TriangulateModule<I>::setState(TRIANGULATE_BLOCK_STATE newState) {
    this->moduleState = newState;
}

template <typename I>
bool TriangulateModule<I>::parseNetworkInput(const NetworkOutputResult inputData) {
    if (true) {

        return true;
    } else return false;
}

template <typename I>
void TriangulateModule<I>::runModule() {
    while(this->moduleState == TRIANGULATE_IDLE || this->moduleState == TRIANGULATE_RUNNING) {
        I inputData = this->inputData.pop();

        // Parse input
        bool dataParsed = true;
        if (inputData.cmdOrigin == CLI_CALIB_SET) {
            if(this->parseCLIInput(inputData) == false) {
                printf("[CALIB] Failed to parse input from Network\n");
                dataParsed = false;
            }
        }

        if (dataParsed == true) {
        }
    }
}
