#include <thread>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/dup_filter_sink.h>
#include <spdlog/async.h>

#include "../../lib/CLI11.hpp"
#include "../../lib/frameQueue.hpp"

#include "../orchestrator.hpp"
#include "../receiver.hpp"
#include "../processor.hpp"

#include "../../lib/lib.hpp"

using namespace std;

void setupLogger() {
    // 1. Khởi tạo một hàng đợi chứa tối đa 8192 tin nhắn log, và dùng 1 luồng công nhân
    spdlog::init_thread_pool(8192, 1);

    auto stdout_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    auto dup_filter = std::make_shared<spdlog::sinks::dup_filter_sink_mt>(std::chrono::seconds(5));
    dup_filter->add_sink(stdout_sink);

    // 2. Thay vì make_shared<spdlog::logger>, ta dùng async_logger
    auto logger = std::make_shared<spdlog::async_logger>(
        "MoCapSys", 
        dup_filter, 
        spdlog::thread_pool(), 
        spdlog::async_overflow_policy::block // Nếu hàng đợi đầy thì làm gì (block hoặc drop)
    );
    
    spdlog::set_default_logger(logger);
}

int main() {
    setupLogger();

    DataQueue<AlignedFrame> dataQueue;

    GlobalConfig glcfg = {
        .is_running = true,
    };
    ReceiverConfig rcv_cfg = {
        .glcfg = glcfg,
        .rcv_port = 5557,
        .rcv_ip = "0.0.0.0",
    };
    ProcessorConfig prs_cfg = {
        .glcfg = glcfg,
        .prs_port = 5556,
        .prs_ip = "0.0.0.0",
    };
    OrchestratorConfig orch_cfg = {
        .glcfg = glcfg,
        .orch_port = 5555,
        .orch_ip = "255.255.255.255",
    };
    
    OrchestratorFunctor orch(orch_cfg);
    ReceiverFunctor rcv(rcv_cfg);
    ProcessorFunctor prs(prs_cfg);

    thread orchThread(ref(orch));
    thread rcvThread(ref(rcv), ref(dataQueue));
    thread prsThread(ref(prs), ref(dataQueue));

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
                orch_cfg.glcfg.is_running = false;
                rcv_cfg.glcfg.is_running = false;
                prs_cfg.glcfg.is_running = false;

                break;
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