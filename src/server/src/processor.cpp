#include <cstring>
#include <string>
#include <zmq.hpp>

#include "processor.hpp"

ProcessorFunctor::ProcessorFunctor(ProcessorConfig& cfg) {
    this->isRunning = &(cfg.glcfg.is_running);
    this->sendIP = cfg.prs_ip;
    this->sendPort = cfg.prs_port;
    this->needsUpdate = false;
}

void ProcessorFunctor::changeConnection(ProcessorConfig& prs_cfg) {
    std::lock_guard<std::mutex> lock(configMtx);
    this->sendPort = prs_cfg.prs_port;
    this->sendIP = prs_cfg.prs_ip;
    this->needsUpdate = true;
}

bool ProcessorFunctor::operator()(DataQueue<AlignedFrame>& queue) {
    Processor3D processor(queue);

    zmq::context_t context(1);
    zmq::socket_t publisher(context, zmq::socket_type::pub);
    const char MOCAP_CHANNEL = 0x01;
    
    std::string bind_address = "tcp://*:" + std::to_string(this->sendPort);

    try {
        publisher.bind(bind_address);
    } catch (const zmq::error_t& e) {
        return false;
    }
    
    while (this->isRunning) {
        if (this->needsUpdate) {
            try {
                publisher.unbind(bind_address);
                bind_address = "tcp://*:" + std::to_string(this->sendPort);
                publisher.bind(bind_address);
            } catch (const zmq::error_t& e) {
            }
            this->needsUpdate = false;
        }

        AlignedFrame frame;
        queue.pop(frame);

        vector<ProcessedPoint> points3D;

        for (const auto& pt : points3D) {
            zmq::message_t topic_msg(&MOCAP_CHANNEL, 1);
            publisher.send(topic_msg, zmq::send_flags::sndmore); 
            
            zmq::message_t payload_msg(sizeof(ProcessedPoint));
            std::memcpy(payload_msg.data(), &pt, sizeof(ProcessedPoint));
            publisher.send(payload_msg, zmq::send_flags::none);
        }
    }

    publisher.close();
    context.close();

    return true;
}