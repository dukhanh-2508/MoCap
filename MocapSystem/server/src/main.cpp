#include <thread>
#include <string>
#include <memory>
#include <cstdio>
#include <cstdlib>

#include "../header/main.hpp"
#include "../header/controller.hpp"
#include "../../lib/header/ringBuffer.hpp"
#include "../../lib/header/threadQueue.hpp"
#include "../../lib/header/module.hpp"

#include "../header/cmdI.hpp"
#include "../header/calib.hpp"
#include "../header/network.hpp"
#include "../header/triangulate.hpp"
#include "../header/controller.hpp"

using namespace std;
using namespace cv;

int main() {
    printf("[SYSTEM] Initializing MoCap System...\n");

    // Initialize main notification queue 
    auto notiQueue = make_shared<ThreadSafeRingBuffer<SystemNotification>>(50);
    auto mainToNetQueue = make_shared<ThreadSafeRingBuffer<NetworkInput>>(50);

    // Instantiate Modules
    // Passing dummy types for inputs where they are overriden or unused
    CLIModule<int> cliModule;
    CalibModule<CLIOutputResult> calibModule;
    NetworkModule<NetworkInput> netModule;
    TriangulateModule<NetworkOutputResult> triModule;

    // Routing
    // Assign central notification queue to all modules
    // Main thread normally sleeps when the notification queue's empty, and wakes up when smth was sent to this queue
    cliModule.assignOuNotiQueue(notiQueue);
    calibModule.assignOuNotiQueue(notiQueue);
    netModule.assignOuNotiQueue(notiQueue);
    triModule.assignOuNotiQueue(notiQueue);

    // Pipe CLI Output -> Calib Input (routing to control calibration block from CLI)
    calibModule.assignInputQueue(cliModule.getResultQueue());

    // Pipe Network Output -> Triangulate Input (routing for 2D tracking coords)
    triModule.assignInputQueue(netModule.getResultQueue());

    // Main Thread -> Network Input (routing for specific manual requests like taking pictures)
    netModule.assignInputQueue(mainToNetQueue);

    // Start Modules Threads
    cliModule.setState(CLI_RUNNING);
    calibModule.setState(CALIB_IDLE);
    netModule.setState(NETWORK_IDLE);
    triModule.setState(TRIANGULATE_IDLE);

    thread cliThread(&CLIModule<int>::runModule, &cliModule);
    thread calibThread(&CalibModule<CLIOutputResult>::runModule, &calibModule);
    thread netThread(&NetworkModule<NetworkInput>::runModule, &netModule);
    thread triThread(&TriangulateModule<NetworkOutputResult>::runModule, &triModule);

    MocapController controller;
    bool isSystemRunning = true;

    printf("[SYSTEM] All modules started. Entering Controller FSM...\n");

    // Main Event Loop (Controller/Router)
    while(isSystemRunning) {
        // Block and wait for events from any module
        SystemNotification noti = notiQueue->pop();

        switch (noti.origin) {
            case CLI_BLOCK: {
                CLINotiPayload payload = get<CLINotiPayload>(noti.payload);

                switch (payload.cmdOrigin) {
                    case CLI_SYSTEM_MODE_SET: {
                        systemMode_Info info = get<systemMode_Info>(payload.info);
                        
                        if (info.mainMode == "track") {
                            controller.changeState(TRACK);
                            calibModule.setState(CALIB_IDLE);
                            netModule.setState(NETWORK_RUNNING);
                            triModule.setState(TRIANGULATE_RUNNING);
                            printf("[ROUTER] System switched to TRACKING MODE.\n");
                        } 
                        else if (info.mainMode == "calib") {
                            controller.changeState(CALIB_STATE);
                            netModule.setState(NETWORK_IDLE);
                            triModule.setState(TRIANGULATE_IDLE);
                            calibModule.setState(CALIB_RUNNING);
                            printf("[ROUTER] System switched to CALIBRATION MODE.\n");
                        }
                        break;
                    }

                    case CLI_CALIB_SET: {
                        printf("[ROUTER] Calibration parameters routed to CalibModule.\n");
                        break;
                    }

                    case CLI_NETWORK_SET: {
                        network_Info info = get<network_Info>(payload.info);
                        
                        NetworkInput netIn;
                        netIn.cmd = NET_CMD_CONFIG;
                        
                        NetCmdConfig cfg;
                        cfg.tcp_port = info.port;
                        cfg.udp_port = info.port + 1; // Auto offset for UDP
                        cfg.bind_ip = info.ip;
                        
                        netIn.payload = cfg;
                        mainToNetQueue->push(netIn);
                        
                        printf("[ROUTER] Network configurations dispatched.\n");
                        break;
                    }

                    case CLI_MANAGE_SLAVE_SET: {
                        manageSlave_Info info = get<manageSlave_Info>(payload.info);
                        
                        if (info.getID) {
                            NetworkInput netIn;
                            netIn.cmd = NET_CMD_QUERY_INFO;
                            
                            NetCmdQueryInfo qInfo;
                            qInfo.slave_id = info.newSlaveID; // Use ID to query
                            netIn.payload = qInfo;
                            
                            mainToNetQueue->push(netIn);
                            printf("[ROUTER] Dispatched Info Query for Slave %d.\n", qInfo.slave_id);
                        }
                        
                        if (info.toggle != "") {
                            NetworkInput netIn;
                            netIn.cmd = NET_CMD_SET_PARAM;
                            
                            NetCmdSetParam setParam;
                            setParam.slave_id = info.newSlaveID;
                            setParam.param_name = "TOGGLE";
                            setParam.value = (info.toggle == "on") ? 1.0f : 0.0f;
                            netIn.payload = setParam;
                            
                            mainToNetQueue->push(netIn);
                            printf("[ROUTER] Dispatched Toggle command for Slave %d.\n", setParam.slave_id);
                        }
                        break;
                    }

                    case CLI_OTHER_SET: {
                        other_Info info = get<other_Info>(payload.info);
                        
                        if (info.isManualMode) {
                            NetworkInput netIn;
                            netIn.cmd = NET_CMD_REQ_IMAGE;
                            
                            NetCmdReqImage req;
                            // Convert string ID from CLI to integer ID
                            req.slave_id = (info.targetCamID != "") ? stoi(info.targetCamID) : 0; 
                            req.saveFolder = info.saveFolder;
                            
                            netIn.payload = req;
                            mainToNetQueue->push(netIn);
                            
                            printf("[ROUTER] Dispatched Image Capture request to Slave %d.\n", req.slave_id);
                        }
                        break;
                    }

                    case CLI_EXIT: {
                        printf("[ROUTER] Exit signal received. Commencing shutdown...\n");
                        cliModule.setState(CLI_STOP);
                        calibModule.setState(CALIB_STOP);
                        netModule.setState(NETWORK_STOP);
                        triModule.setState(TRIANGULATE_STOP);
                        
                        isSystemRunning = false;
                        break;
                    }

                    default:
                        break;
                }
                break;
            }
            
            case NETWORK: {
                break;
            }
            case CALIB_BLOCK: {
                break;
            }
            case TRIANGULATE: {
                break;
            }
            default:
                break;
        }
    }

    // Shutdown (join threads)
    if (cliThread.joinable()) cliThread.join();
    if (calibThread.joinable()) calibThread.join();
    if (netThread.joinable()) netThread.join();
    if (triThread.joinable()) triThread.join();

    printf("[SYSTEM] All modules terminated. Goodbye!\n");
    return 0;
}