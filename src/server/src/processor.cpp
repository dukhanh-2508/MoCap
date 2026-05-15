#include <cstring>
#include <string>
#include <zmq.hpp>
#include <cmath>
#include <spdlog/spdlog.h>

#include "../processor.hpp"

// --- LOGIC XỬ LÝ 2D SANG 3D (ÉP Z = 0) ---
vector<ProcessedPoint> Processor3D::triangulate(const AlignedFrame& frame) {
    vector<ProcessedPoint> points3D;

    // Duyệt qua từng cặp {camera_id, CameraPacket} trong cuốn từ điển camera_data
    for (const auto& cam_pair : frame.camera_data) {
        // uint8_t cam_id = cam_pair.first; // (Tạm thời chưa dùng đến nếu chỉ ép Z=0 đơn thuần)
        const CameraPacket& cam_packet = cam_pair.second;

        // Duyệt qua mảng các điểm 2D (centers) của camera hiện tại
        for (const auto& center : cam_packet.centers) {
            ProcessedPoint pt;
            
            pt.m_id = center.object_id;
            
            // Lấy trực tiếp tọa độ X, Y từ ảnh. 
            // Có thể chia tỷ lệ (ví dụ / 100.0f) nếu client nhận ZMQ (như Unity) dùng đơn vị mét.
            pt.x = center.x; 
            pt.y = center.y;
            
            pt.z = pt.m_id; 
            
            points3D.push_back(pt);
        }
    }

    return points3D;
}
// --------------------------------

ProcessorFunctor::ProcessorFunctor(ProcessorConfig& cfg) {
    this->isRunning = &(cfg.glcfg.is_running);
    this->sendIP = cfg.prs_ip;
    this->sendPort = cfg.prs_port;
    this->needsUpdate = false;
}

void ProcessorFunctor::changeConnection(ProcessorConfig& prs_cfg) {
    lock_guard<mutex> lock(configMtx);
    this->sendPort = prs_cfg.prs_port;
    this->sendIP = prs_cfg.prs_ip;
    this->needsUpdate = true;
}

bool ProcessorFunctor::operator()(DataQueue<AlignedFrame>& queue) {
    Processor3D processor;

    zmq::context_t context(1);
    zmq::socket_t publisher(context, zmq::socket_type::pub);

    // publisher will linger if there are still packets in the network queue when closed
    // ==> Force not to linger
    publisher.set(zmq::sockopt::linger, 0);
    const char MOCAP_CHANNEL = 0x01;
    
    string bind_address = "tcp://" + this->sendIP + ":" + to_string(this->sendPort);

    try {
        publisher.bind(bind_address);
    } catch (const zmq::error_t& e) {
        spdlog::error("[3D RECONSTRUCTOR] Failed to bind connection");
        return false;
    }
    spdlog::info("[3D RECONSTRUCTOR] Connection established at {} - {}", this->sendIP, this->sendPort);
    
    while (((this->isRunning != NULL) && *(this->isRunning))) {
        spdlog::info("[3D RECONSTRUCTOR] Looping");
        if (this->needsUpdate) {
            try {
                publisher.unbind(bind_address);
                bind_address = "tcp://" + this->sendIP + ":" + to_string(this->sendPort);
                publisher.bind(bind_address);
            } catch (const zmq::error_t& e) {
                spdlog::error("[3D RECONSTRUCTOR] Failed to bind connection");
            }
            this->needsUpdate = false;
        }

        spdlog::info("[3D RECONSTRUCTOR] Acquiring frame...");
        AlignedFrame frame;
        if (!queue.pop(frame)) {
            spdlog::error("[3D RECONSTRUCTOR] Pop invalid, ending thread!");
            break; 
        }

        spdlog::info("[3D RECONSTRUCTOR] Processing frame...");
        vector<ProcessedPoint> points3D = processor.triangulate(frame);
        spdlog::info("[3D RECONSTRUCTOR] Processing completed.");

        for (const auto& pt : points3D) {
            try {
            zmq::message_t topic_msg(&MOCAP_CHANNEL, 1);
            publisher.send(topic_msg, zmq::send_flags::sndmore); 
            
            zmq::message_t payload_msg(sizeof(ProcessedPoint));
            memcpy(payload_msg.data(), &pt, sizeof(ProcessedPoint));
            publisher.send(payload_msg, zmq::send_flags::none);
            } catch (const zmq::error_t& e) {
                spdlog::error("ZMQ Send Error: {} - {}", e.num(), e.what());
            }

            spdlog::info("[3D RECONSTRUCTOR] Reconstructed data point: ID {} - {}, {}, {}", pt.m_id, pt.x, pt.y, pt.z);
        }
    }

    publisher.close();
    context.close();

    return true;
}