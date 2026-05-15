#include <iostream>
#include <thread>
#include "../../lib/CLI11.hpp"

#include "../sender.hpp"
#include "../receiver.hpp"
#include "../imgProcessing.hpp"
#include "../../lib/frameQueue.hpp"
#include "../../lib/lib.hpp"

using namespace std;

int main() {
    GlobalConfig glcfg = {
        .is_running = true,
    };
    ReceiverConfig rcv_cfg = {
        .glcfg = glcfg,
        .rcv_port = 5555,
        .rcv_ip = "0.0.0.0",
    };
    SenderConfig snd_cfg = {
        .glcfg = glcfg,
        .snd_port = 5557,
        .snd_ip = "255.255.255.255", // IP server (as of the testing time, at least)
    };
    CameraConfig cmr_cfg = {
        .glcfg = glcfg,
    };

    DataQueue<FutureTriggerPacket> cmdQueue;
    DataQueue<CameraPacket> resultQueue;

    ReceiverFunctor rcv(rcv_cfg);
    MarkerDetectorFunctor prs(cmr_cfg);
    ResultSenderFunctor snd(snd_cfg);

    thread rcvThread(ref(rcv), ref(cmdQueue));
    thread prsThread(ref(prs), ref(cmdQueue), ref(resultQueue));
    thread sndThread(ref(snd), ref(resultQueue));

    CLI::App app;

    CLI::App* set_cmd = app.add_subcommand("set", "Change parameters");
    bool configRcv = false;
    set_cmd->add_flag("--rcv", configRcv, "Config receiver");
    bool configSnd = false;
    set_cmd->add_flag("--snd", configSnd, "Config sender");
    int moduleID = 0;
    set_cmd->add_option("--id", moduleID, "Config module ID");
    int port = 0;
    set_cmd->add_option("--port", port, "Set port");
    string ip = "";
    set_cmd->add_option("--ip", ip, "Set IP");
    
    CLI::App* switch_cmd = app.add_subcommand("exit", "Exit the system");

    cout << "Use '--help' for help and 'exit' to quit the program'" << endl;

    string input_line;
    while (glcfg.is_running) {
        cout << "\n[Slave]> ";
        if (!getline(cin, input_line)) break;
        if (input_line.empty()) continue;

        try {
            app.parse(input_line);

            if (switch_cmd->parsed()) { 
                glcfg.is_running = false;
                rcv_cfg.glcfg.is_running = false;
                cmr_cfg.glcfg.is_running = false;
                snd_cfg.glcfg.is_running = false;
            }
            if (set_cmd->parsed()) {
                if (set_cmd->count("--id") > 0) { // Change ID
                    glcfg.module_id = moduleID;
                }
                if (configRcv) {
                    if (set_cmd->count("--port") > 0) {
                        rcv_cfg.rcv_port = port;
                    }
                    if (set_cmd->count("--ip") > 0) {
                        rcv_cfg.rcv_ip = ip;
                    }
                    rcv.changeSocket(rcv_cfg);
                    configRcv = false;
                }
                if (configSnd) {
                    if (set_cmd->count("--port") > 0) {
                        snd_cfg.snd_port = port;
                    }
                    if (set_cmd->count("--ip") > 0) {
                        snd_cfg.snd_ip = ip;
                    }
                    snd.changeSocket(snd_cfg);
                    configSnd = false;
                }
            }
        } catch (const CLI::ParseError &e) {
            app.exit(e);
        }

        app.clear();
    }

    cmdQueue.shutdown(); 
    resultQueue.shutdown();

    if (rcvThread.joinable()) rcvThread.join();
    if (prsThread.joinable()) prsThread.join();
    if (sndThread.joinable()) sndThread.join();

    return 0;
}
