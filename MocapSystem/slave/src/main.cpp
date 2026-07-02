#include <thread>
#include <string>
#include <cstdio>
#include <cstdlib>
#include <memory>

#include "../../lib/header/ringBuffer.hpp"
#include "../../lib/header/threadQueue.hpp"
#include "../../lib/header/lib_slave.hpp"

#include "../header/network.hpp"
#include "../header/captureImg.hpp"
#include "../header/processImg.hpp"

using namespace std;
using namespace cv;

int main() {
    printf("[SLAVE] Initializing MoCap Slave System...\n");

    auto notiQueue = make_shared<ThreadSafeRingBuffer<SystemNotification>>(50);
    auto netInputQueue = make_shared<ThreadSafeRingBuffer<SlaveNetworkInput>>(50);

    // Network receives server TCP/UDP commands -> outputs CaptureTrigger
    NetworkModule<SlaveNetworkInput> netModule;
    // Capture receives CaptureTrigger -> outputs FramePacket
    CaptureImgModule<CaptureTrigger> captureModule;
    // Process receives FramePacket -> outputs nothing directly to another module (pushes to FSM)
    ProcessImgModule<FramePacket> processModule;

    // Assign central notification queue to all modules for status/event reporting
    netModule.assignOuNotiQueue(notiQueue);
    captureModule.assignOuNotiQueue(notiQueue);
    processModule.assignOuNotiQueue(notiQueue);

    // Network output -> Capture Input
    captureModule.assignInputQueue(netModule.getResultQueue());
    
    // Capture output -> Process Input
    processModule.assignInputQueue(captureModule.getResultQueue());
    
    // Assign the custom queue to Network so FSM can route multiple data types to it
    netModule.assignInputQueue(netInputQueue);

    // Start Modules Threads
    netModule.setState(NETWORK_IDLE);
    captureModule.setState(CAPTURE_IDLE);
    processModule.setState(PROCESS_IDLE);

    thread netThread(&NetworkModule<SlaveNetworkInput>::runModule, &netModule);
    thread captureThread(&CaptureImgModule<CaptureTrigger>::runModule, &captureModule);
    thread processThread(&ProcessImgModule<FramePacket>::runModule, &processModule);

    bool isSystemRunning = true;

    printf("[SLAVE] All modules started. Entering Controller FSM...\n");

    while(isSystemRunning) {
        // Block and wait for events from any module
        SystemNotification noti = notiQueue->pop();

        switch (noti.origin) {
            case NETWORK: {
                NetworkNotiPayloadSlave payload = get<NetworkNotiPayloadSlave>(noti.payload);
                
                switch (payload.cmdOrigin) {
                    case NET_SERVER_CMD_TRACKING: {
                        if (payload.toggle == "on") {
                            captureModule.setState(CAPTURE_RUNNING);
                            processModule.setState(PROCESS_RUNNING);
                            netModule.setState(NETWORK_RUNNING);
                            printf("[SLAVE FSM] Mode switched to TRACKING.\n");
                        } else {
                            captureModule.setState(CAPTURE_IDLE);
                            processModule.setState(PROCESS_IDLE);
                            netModule.setState(NETWORK_IDLE);
                            printf("[SLAVE FSM] Mode switched to IDLE.\n");
                        }
                        break;
                    }

                    case NET_SERVER_CMD_CONFIG: {
                        printf("[SLAVE FSM] Configuration updated from Server: ID = %d\n", payload.slave_id);
                        break;
                    }

                    case NET_SERVER_CMD_INFO: {
                        SlaveNetworkInput netIn;
                        netIn.type = SLAVE_NET_SEND_INFO;
                        netIn.infoMsg = "Slave operational. State: " + to_string(netModule.getModuleState());
                        netInputQueue->push(netIn);
                        
                        printf("[SLAVE FSM] Handled info query from Server.\n");
                        break;
                    }
                    
                    default:
                        break;
                }
                break;
            }

            case CAPTURE_BLOCK: {
                CaptureNotiPayload payload = get<CaptureNotiPayload>(noti.payload);
                
                if (payload.status == CAPTURE_SAVED_TO_RAM) {
                    SlaveNetworkInput netIn;
                    netIn.type = SLAVE_NET_SEND_FILE;
                    netIn.filepath = payload.filepath;
                    netInputQueue->push(netIn);
                    
                    printf("[SLAVE FSM] Image saved to RAM. Instructed Network to send file: %s\n", payload.filepath.c_str());
                }
                break;
            }

            case PROCESS_BLOCK: {
                ProcessNotiPayload payload = get<ProcessNotiPayload>(noti.payload);
                
                if (payload.status == PROCESS_TRACKING_DONE) {
                    // Forward the CameraPacket to Network for TCP transmission
                    SlaveNetworkInput netIn;
                    netIn.type = SLAVE_NET_SEND_TRACKING;
                    netIn.cameraData = payload.cameraData;
                    netInputQueue->push(netIn);
                }
                break;
            }
            
            case CLI_BLOCK: {
                printf("[SLAVE FSM] Shutdown signal received.\n");
                isSystemRunning = false;
                break;
            }
            
            default:
                break;
        }
    }

    // Shutdown by joining threads
    if (netThread.joinable()) netThread.join();
    if (captureThread.joinable()) captureThread.join();
    if (processThread.joinable()) processThread.join();

    printf("[SLAVE] Modules terminated. Goodbye!\n");
    return 0;
}