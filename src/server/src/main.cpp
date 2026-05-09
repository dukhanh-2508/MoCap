#include <thread>

#include "CLI11.hpp"

#include "orchestrator.hpp"
#include "receiver.hpp"
#include "processor.hpp"
#include "frameQueue.hpp"

#include "lib.hpp"

using namespace std;

int main() {
    DataQueue<AlignedFrame> dataQueue;

    GlobalConfig glcfg = {
        .is_running = true,
    };
    ReceiverConfig rcv_cfg = {
        .glcfg = &glcfg,
    };
    ProcessorConfig prs_cfg = {
        .glcfg = &glcfg,
    };
    OrchestratorConfig orch_cfg = {
        .glcfg = &glcfg,
    };
    
    OrchestratorFunctor orch(orch_cfg);
    ReceiverFunctor rcv(rcv_cfg);
    ProcessorFunctor prs(prs_cfg);

    thread orchThread(orch);
    thread rcvThread(rcv, ref(dataQueue));
    thread prsThread(prs, ref(dataQueue));

    CLI::App app;

    CLI::App* set_cmd = app.add_subcommand("set", "Change parameters");
    bool configOrch = false;
    set_cmd->add_flag("--orch", configOrch, "Config orchestrator");
    bool configRcv = false;
    set_cmd->add_flag("--rcv", configRcv, "Config receiver");
    bool configPrs = false;
    set_cmd->add_flag("--prs", configPrs, "Config processor");
    int port = 0;
    set_cmd->add_option("--port", port, "Set port");
    string ip = "";
    set_cmd->add_option("--ip", ip, "Set IP");
    
    CLI::App* switch_cmd = app.add_subcommand("exit", "Exit the system");

    cout << "Use '--help' for help and 'exit' to quit the program'" << endl;

    string input_line;
    while (glcfg.is_running) {
        cout << "\n[Server]> ";
        if (!getline(cin, input_line)) break;
        if (input_line.empty()) continue;

        try {
            app.parse(input_line);

            if (switch_cmd->parsed()) { 
                glcfg.is_running = false;
            }
            if (set_cmd->parsed()) {
                if (configOrch) {
                    if (set_cmd->count("--port") > 0) {
                        orch_cfg.orch_port = port;
                    }
                    if (set_cmd->count("--ip") > 0) {
                        orch_cfg.orch_ip = ip;
                    }
                    orch.changeSocket(orch_cfg);
                    configOrch = false;
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
                if(configPrs) {
                    if (set_cmd->count("--port") > 0) {
                        prs_cfg.prs_port = port;
                    }
                    if (set_cmd->count("--ip") > 0) {
                        prs_cfg.prs_ip = ip;
                    }
                    prs.changeConnection(prs_cfg);
                    configPrs = false;
                }
            }
        } catch (const CLI::ParseError &e) {
            app.exit(e);
        }

        app.clear();
    }

    dataQueue.shutdown();

    if (orchThread.joinable()) orchThread.join();
    if (rcvThread.joinable()) rcvThread.join();
    if (prsThread.joinable()) prsThread.join();

    return 0;
}