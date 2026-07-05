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

    // System set
    int systemFPS = -999;
    CLI::App* sys_cfg = app.add_subcommand("syscfg", "Config system parameters");
    sys_cfg->add_option("--fps", systemFPS, "Choose system FPS (Control how quickly trigger messages are boardcasted and camera FPS)");

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

    double target_rms = -999.0f; 
    size_t min_images = -999; 
    calib->add_option("--rms", target_rms, "Final RMS for intrinsic calibration to optimize to");
    calib->add_option("--minImg", min_images, "Min image amount to do for intrinsic calib");

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
    int target_id = -999;
    int newSlaveID_val = -999;
    float thresh_value = -999.0f, max_dis = -999.0f, track_dist = -999.0f; double minArea = -999.0f; double minCircularity = -999.0f;
    float brightness = -999.0f, gain = -999.0f, exposure = -999.0f;
    int resWidth = -999; int resHeight = -999; int camFPS = -999;
    bool camOff = false; bool camOn = false;
    CLI::App* slave_cmd = app.add_subcommand("slave", "Slave utility commands");
    string slave_ip = "";
    slave_cmd->add_option("--slaveIP", slave_ip);

    bool getID = false;
    slave_cmd->add_flag("--getID", getID);
    slave_cmd->add_flag("--camOff", camOff);
    slave_cmd->add_flag("--camOn", camOn);
    
    slave_cmd->add_option("--targetID", target_id, "Target by ID");
    slave_cmd->add_option("--updateID", newSlaveID_val, "Set new ID");
    string toggle = "";
    slave_cmd->add_option("--toggle", toggle, "Turn this slave on / off");
    
    slave_cmd->add_option("--thresh", thresh_value, "Set thresh value");
    slave_cmd->add_option("--maxDis", max_dis, "Set max disappeared frames");
    slave_cmd->add_option("--trackDist", track_dist, "Set tracking distance");
    slave_cmd->add_option("--brightness", brightness, "Set camera brightness");
    slave_cmd->add_option("--gain", gain, "Set camera gain");
    slave_cmd->add_option("--exposure", exposure, "Set camera exposure time");
    slave_cmd->add_option("--w", resWidth, "Set resolution width");
    slave_cmd->add_option("--h", resHeight, "Set resolution height");
    slave_cmd->add_option("--fps", camFPS, "Set cam FPS");
    slave_cmd->add_option("--area", minArea, "Set min marker area");
    slave_cmd->add_option("--cir", minCircularity, "Set min marker circularity");


    
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
    controlled_capture_cmd->add_option("--saveFolder", saveFolder, "Folder to save the images");
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
        systemFPS = -999;
        imgSrcIn = ""; imgSrcEx = ""; startCalibCalc = false; calibMode = ""; targetCameraID = 0; row = 0; col = 0; squareSize = 0; target_rms = -999.0f; min_images = -999;
        triangulateMode = ""; resultFolder = "";
        confNetworkMaster = false; confNetworkSlave = false; 
        confTX = false; confRX = false; port = 0; ip = "";
        slave_ip = ""; getID = false; toggle = ""; camOff = false; camOn = false;
        takePicAmount = 0; autoShot = false; manualShot = false; cameraID = 0; saveFolder = "."; startIdx = 0;
        target_id = -999;
        newSlaveID_val = -999;
        thresh_value = -999.0f; max_dis = -999.0f; track_dist = -999.0f; minArea = -999.0f; minCircularity = -999.0f;
        brightness = -999.0f; gain = -999.0f; exposure = -999.0f; resWidth = -999; resHeight = -999; camFPS = -999;

        bool dontSendMore = false; // Avoid sending another payload after this if-else block if a payload has been sent in this if-else block

        try {
            app.clear();
            app.parse(input_line, false);

            CLINotiPayload payload;

            // System Mode
            if (sys_mode->parsed()) {
                systemMode_Info mode_info;
                if (sys_mode->count("--main") > 0) mode_info.mainMode = mainMode;
                if (sys_mode->count("--sub") > 0) mode_info.subMode = subMode;
                
                payload.cmdOrigin = CLI_SYSTEM_MODE_SET;
                payload.info = mode_info;
            }
            // System config
            if (sys_cfg->parsed()) {
                systemCfg_Info cfg;

                cfg.sysFPS = (sys_cfg->count("--fps") > 0) ? systemFPS : -999;

                payload.cmdOrigin = CLI_SYS_CFG;
                payload.info = cfg;
            }
            else if (calib->parsed()) {
                calib_Info calib_info;
                CLIOutputResult result;
                result.cmdOrigin = CLI_CALIB_SET;

                CalibSettings c_settings;
                c_settings.calibState = CALIB_IDLE; 

                if (calib->count("--imgSrcIn") > 0) {
                    calib_info.imgSrc = imgSrcIn;
                    c_settings.dataFolderIn = imgSrcIn; 
                }
                if (calib->count("--imgSrcEx") > 0) {
                    calib_info.imgSrc = imgSrcEx;
                    c_settings.dataFolderEx = imgSrcEx;
                }
                if (calib->count("--startCalib") > 0) {
                    calib_info.startCalibCalc = true;
                    c_settings.calibState = CALIB_START;
                }
                if (calib->count("--mode") > 0) {
                    calib_info.calibMode = calibMode;
                    if (calibMode == "ex") c_settings.calibMode = CALIB_MODE_EX;
                    else if (calibMode == "in") c_settings.calibMode = CALIB_MODE_IN;
                }
                if (calib->count("--id") > 0) {
                    calib_info.targetID = targetCameraID;
                    c_settings.targetID = targetCameraID;
                }
                if (calib->count("--row") > 0) {
                    c_settings.desc.row = row;
                }
                if (calib->count("--col") > 0) {
                    c_settings.desc.col = col;
                }
                if (calib->count("--sqSize") > 0) {
                    c_settings.desc.checkSize = squareSize;
                }
                if (calib->count("--rms") > 0) {
                    if (calibMode == "in") {
                        calib_info.target_rms = target_rms != -999.0f ? target_rms : -999.0f;
                    }
                }
                if (calib->count("--minImg") > 0) {
                    if (calibMode == "in") {
                        calib_info.min_images = min_images != -999 ? min_images : -999;
                    }
                }
                
                result.result = c_settings;

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
                slave_info.slave_ip = (slave_cmd->count("--slaveIP") > 0) ? slave_ip : "";
                slave_info.target_id = (slave_cmd->count("--targetID") > 0) ? target_id : -999;
                slave_info.newSlaveID = (slave_cmd->count("--updateID") > 0) ? newSlaveID_val : -999;
                slave_info.thresh_value = (slave_cmd->count("--thresh") > 0) ? thresh_value : -999;
                slave_info.max_disappeared = (slave_cmd->count("--maxDis") > 0) ? max_dis : -999;
                slave_info.tracking_dist = (slave_cmd->count("--trackDist") > 0) ? track_dist : -999;
                slave_info.brightness = (slave_cmd->count("--brightness") > 0) ? brightness : -999;
                slave_info.gain = (slave_cmd->count("--gain") > 0) ? gain : -999;
                slave_info.exposure = (slave_cmd->count("--exposure") > 0) ? exposure : -999;
                slave_info.resWidth = (slave_cmd->count("--w") > 0) ? resWidth : -999;
                slave_info.resHeight = (slave_cmd->count("--h") > 0) ? resHeight : -999;
                slave_info.camFPS = (slave_cmd->count("--fps") > 0) ? camFPS : -999;
                slave_info.minArea = (slave_cmd->count("--area") > 0) ? minArea : -999.0f;
                slave_info.minCircularity = (slave_cmd->count("--cir") > 0) ? minCircularity : -999.0f;

                slave_info.camOff = (slave_cmd->count("--camOff") > 0) ? camOff : false;
                slave_info.camOn = (slave_cmd->count("--camOn") > 0) ? camOn : false;


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

