#pragma once

#include "../../lib/header/CLI11.hpp"
#include "../../lib/header/lib.hpp"
#include "../../lib/header/module.hpp"

template <typename I>
class CLIModule : public IModule<I, CLIOutputResult> {
    public:
        CLIModule();
        ~CLIModule();

        void setState(int newState) override;
        void runModule() override;
};

template <typename I>
CLIModule<I>::CLIModule() : IModule<I, CLIOutputResult>(20) {

}

template <typename I>
CLIModule<I>::~CLIModule() {
    
}

template <typename I>
void CLIModule<I>::setState(int newState) {
    this->moduleState = newState;
}

template <typename I>
void CLIModule<I>::runModule() {
    CLI::App app;

    // Most basic command: choose system mode and sub-mode (e.g CALIB - Ex Calib)
    string mainMode = "";
    string subMode = "";
    CLI::App* sys_mode = app.add_subcommand("mode", "Choose system mode");
    sys_mode->add_option("--main", mainMode, "Choose system main mode");
    sys_mode->add_option("--sub", subMode, "Choose module mode");

    // Commands for calib block
    CLI::App* calib = app.add_subcommand("calib");
    string imgSrcIn = "";
    calib->add_option("--imgSrcIn", imgSrcIn, "Img folder");
    string imgSrcEx = "";
    calib->add_option("--imgSrcEx", imgSrcEx, "Img folder");
    bool startCalibCalc = false;
    calib->add_option("--startCalib", startCalibCalc, "Start ex/in calib calc");
    string calibMode = "";
    calib->add_option("--mode", calibMode, "Change calib mode (ex / in)");
    int targetCameraID = 0;
    calib->add_option("--id", targetCameraID, "Select camera to calibrate");
    int row = 0, col = 0;
    calib->add_option("--row", row, "Amount of rows of the board, in checker square amount");
    calib->add_option("--col", col, "Amount of cols of the board, in checker square amount");
    float squareSize = 0;
    calib->add_option("--sqSize", squareSize, "Lenth of a side of a checker square, in mm");

    // Commands for the triangulation block
    CLI::App* triangulation = app.add_subcommand("triangulate");
    string triangulateMode = "";
    triangulation->add_option("--mode", triangulateMode);
    string resultFolder = "";
    triangulation->add_option("--calibPath", resultFolder, "Path to a folder with intrinsic and extrinsic calib data. Need to be the same as 'imgSrcIn' of calib");

    // Commands for network block
    CLI::App* set_cmd = app.add_subcommand("set", "Change parameters");

    bool confNetworkMaster = false;
    set_cmd->add_flag("--master", confNetworkMaster, "Config network master");
    bool confNetworkSlave = false;
    set_cmd->add_flag("--slave", confNetworkSlave, "Config network slave");

    bool confTX = false;
    set_cmd->add_flag("--tx", confTX, "Config the sender of master / slave");
    bool confRX = false;
    set_cmd->add_flag("--rx", confRX, "Config the receiver of master / slave");

    int port = 0;
    set_cmd->add_option("--port", port, "Set port");
    string ip = "";
    set_cmd->add_option("--ip", ip, "Set IP");

    // Commands to manage slaves
    CLI::App* slave_cmd = app.add_subcommand("slave", "Slave utility commands");
    string slave_ip = "";
    slave_cmd->add_option("--slaveIP", slave_ip);
    bool getID = false;
    slave_cmd->add_flag("--getID", getID);
    int newSlaveID = 0;
    slave_cmd->add_option("--updateID", newSlaveID);
    string toggle = "";
    slave_cmd->add_option("--toggle", toggle, "Turn this slave on / off");
    
    // Commands for other tasks
    CLI::App* controlled_capture_cmd = app.add_subcommand("ctlcap", "Command slaves to take picture in a controlled manner");
    int takePicAmount = 0;
    controlled_capture_cmd->add_option("--amount", takePicAmount, "Command slaves to take N images. N = -1 means inf times in automatic mode");
    bool autoShot = false;
    controlled_capture_cmd->add_flag("--auto", autoShot, "Slave automatically and consecutively take N images");
    bool manualShot = false;
    controlled_capture_cmd->add_flag("--manual", manualShot, "User press Enter to take picture, or Esc to exit this mode");
    int cameraID = 0;
    controlled_capture_cmd->add_option("--camID", cameraID, "Select camera to take picture");
    string saveFolder = ".";
    controlled_capture_cmd->add_option("--saveFolder", "Folder to save the images");
    int startIdx = 0;
    controlled_capture_cmd->add_option("--startIdx", startIdx, "Idx of the first pic to take in manual mode");

    // Exit the CLI
    CLI::App* switch_cmd = app.add_subcommand("exit", "Exit the system");

    cout << "Use '--help' for help and 'exit' to quit the program'" << endl;

    string input_line;
    while (this->moduleState == CLI_RUNNING) {
        cout << "\n[Server]> ";
        if (!getline(cin, input_line)) break;
        if (input_line.empty()) continue;

        mainMode = ""; subMode = "";
        imgSrcIn = ""; imgSrcEx = ""; startCalibCalc = false; calibMode = ""; targetCameraID = 0; row = 0; col = 0; squareSize = 0;
        triangulateMode = ""; resultFolder = "";
        confNetworkMaster = false; confNetworkSlave = false; 
        confTX = false; confRX = false; port = 0; ip = "";
        slave_ip = ""; getID = false; newSlaveID = 0; toggle = "";
        takePicAmount = 0; autoShot = false; manualShot = false; cameraID = 0; saveFolder = "."; startIdx = 0;

        bool dontSendMore = false; // Avoid sending another payload after this if-else block if a payload has been sent in this if-else block

        try {
            app.clear();
            app.parse(input_line, true);

            CLINotiPayload payload;

            // System Mode
            if (sys_mode->parsed()) {
                systemMode_Info mode_info;
                if (sys_mode->count("--main") > 0) mode_info.mainMode = mainMode;
                if (sys_mode->count("--sub") > 0) mode_info.subMode = subMode;
                
                payload.cmdOrigin = CLI_SYSTEM_MODE_SET;
                payload.info = mode_info;
            }
            // Calib
            else if (calib->parsed()) {
                calib_Info calib_info;
                CLIOutputResult result;
                result.cmdOrigin = CLI_CALIB_SET;

                if (calib->count("--imgSrcIn") > 0) {
                    calib_info.imgSrc = imgSrcIn;
                    get<CalibSettings>(result.result).dataFolderIn = imgSrcIn;
                }
                if (calib->count("--imgSrcEx") > 0) {
                    calib_info.imgSrc = imgSrcEx;
                    get<CalibSettings>(result.result).dataFolderEx = imgSrcEx;
                }
                if (calib->count("--startCalib") > 0) {
                    calib_info.startCalibCalc = true;
                    get<CalibSettings>(result.result).calibState = CALIB_START;
                }
                if (calib->count("--mode") > 0) {
                    calib_info.calibMode = calibMode;
                    if (calibMode == "ex") get<CalibSettings>(result.result).calibMode = CALIB_MODE_EX;
                    else if (calibMode == "in") get<CalibSettings>(result.result).calibMode = CALIB_MODE_IN;
                }
                if (calib->count("--id") > 0) {
                    calib_info.targetID = targetCameraID;
                    get<CalibSettings>(result.result).targetID = targetCameraID;
                }
                if (calib->count("--row") > 0) {
                    get<CalibSettings>(result.result).desc.row = row;
                }
                if (calib->count("--col") > 0) {
                    get<CalibSettings>(result.result).desc.col = col;
                }
                if (calib->count("--sqSize") > 0) {
                    get<CalibSettings>(result.result).desc.checkSize = squareSize;
                }
                // Send to result to output queue
                this->outputData->push(result);
                
                payload.cmdOrigin = CLI_CALIB_SET;
                payload.info = calib_info;
            }
            // Triangulate
            else if (triangulation->parsed()) {
                triangulate_Info tri_info;
                if (triangulation->count("--mode") > 0) tri_info.triangulateMode = triangulateMode;
                if (triangulation->count("--calibPath") > 0) tri_info.calibResultPath = resultFolder;
                
                payload.cmdOrigin = CLI_TRIANGULATE_SET;
                payload.info = tri_info;
            }
            // Network Set
            else if (set_cmd->parsed()) {
                network_Info net_info;
                net_info.confNetworkMaster = confNetworkMaster;
                net_info.confNetworkSlave = confNetworkSlave;
                net_info.confTX = confTX;
                net_info.confRX = confRX;
                
                if (set_cmd->count("--port") > 0) net_info.port = port;
                if (set_cmd->count("--ip") > 0) net_info.ip = ip;
                
                payload.cmdOrigin = CLI_NETWORK_SET;
                payload.info = net_info;
            }
            // Slave Control
            else if (slave_cmd->parsed()) {
                manageSlave_Info slave_info;
                if (slave_cmd->count("--slaveIP") > 0) slave_info.slave_ip = slave_ip;
                slave_info.getID = getID;
                if (slave_cmd->count("--updateID") > 0) slave_info.newSlaveID = newSlaveID;
                if (slave_cmd->count("--toggle") > 0) slave_info.toggle = toggle;
                
                payload.cmdOrigin = CLI_MANAGE_SLAVE_SET;
                payload.info = slave_info;
            }
            // Controlled Capture
            else if (controlled_capture_cmd->parsed()) {
                string savePath = ".";
                if (controlled_capture_cmd->count("--saveFolder") > 0) savePath = saveFolder;
                int camID = 0;
                if (controlled_capture_cmd->count("--camID") > 0) camID = cameraID;
                other_Info oth_info;
                oth_info.saveFolder = savePath;
                oth_info.targetCamID = camID;

                if (controlled_capture_cmd->count("--manual") > 0) {
                    oth_info.isManualMode = true;
                    oth_info.takePicAmount = 1;
                    payload.cmdOrigin = CLI_OTHER_SET;
                    payload.info = oth_info;

                    SystemNotification noti;
                    noti.origin = CLI_BLOCK;
                    noti.payload = payload;

                    int idx = 0;
                    if (controlled_capture_cmd->count("--startIdx") > 0) idx = startIdx;
                    printf("Press 'Enter' to take photo, type 'exit' to exit manual mode.\n\n");
                    while(true) {
                        printf("Current index: %d\n", idx);
                        string input = "";
                        getline(cin, input);
                        if (input == "") { // User pressed 'Enter'
                            get<other_Info>(payload.info).imgIdx = idx;
                            this->outNoti->push(noti);
                            idx++;
                        } else if (input == "exit") break;
                    }
                    dontSendMore = true;

                } else if (controlled_capture_cmd->count("--auto") > 0) {
                    if (controlled_capture_cmd->count("--amount") > 0) oth_info.takePicAmount = takePicAmount;
                    oth_info.isManualMode = false;
                }
                
                payload.cmdOrigin = CLI_OTHER_SET;
                payload.info = oth_info;
            }
            // Exit
            else if (switch_cmd->parsed()) {
                exit_Info ex_info;
                payload.cmdOrigin = CLI_EXIT;
                payload.info = ex_info;
                
                break;
            }

            // Push the payload to main thread
            if (dontSendMore == false) {
                SystemNotification noti;
                noti.origin = CLI_BLOCK;
                noti.payload = payload;
                this->outNoti->push(noti);
            }

        } catch (const CLI::ParseError &e) {
            app.exit(e);
        }

        app.clear();
    }
}

