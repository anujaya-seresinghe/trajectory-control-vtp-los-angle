#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <string>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

// Include standard MAVLink headers
#include <mavlink/common/mavlink.h>

struct WaypointData {
    float t;
    float x;
    float y;
    float vx;
    float vy;
    float ax{0.0f};
    float ay{0.0f};
    float at{0.0f};
    float jt{0.0f};
};

// Helper to calculate numerical derivatives via central differences
void compute_derivatives(std::vector<WaypointData>& wp) {
    size_t n = wp.size();
    if (n < 2) return;

    // 1. Compute accelerations (ax, ay) from velocities (vx, vy)
    for (size_t i = 0; i < n; ++i) {
        float dt = 0.0f;
        if (i == 0) {
            dt = wp[1].t - wp[0].t;
            wp[i].ax = (wp[1].vx - wp[0].vx) / dt;
            wp[i].ay = (wp[1].vy - wp[0].vy) / dt;
        } else if (i == n - 1) {
            dt = wp[n-1].t - wp[n-2].t;
            wp[i].ax = (wp[n-1].vx - wp[n-2].vx) / dt;
            wp[i].ay = (wp[n-1].vy - wp[n-2].vy) / dt;
        } else {
            dt = wp[i+1].t - wp[i-1].t;
            wp[i].ax = (wp[i+1].vx - wp[i-1].vx) / dt;
            wp[i].ay = (wp[i+1].vy - wp[i-1].vy) / dt;
        }
    }

    // 2. Compute centripetal acceleration: at = |vx*ay - vy*ax| / v
    for (size_t i = 0; i < n; ++i) {
        float v = std::hypot(wp[i].vx, wp[i].vy);
        if (v > 1e-4f) {
            wp[i].at = std::abs(wp[i].vx * wp[i].ay - wp[i].vy * wp[i].ax) / v;
        } else {
            wp[i].at = 0.0f;
        }
    }

    // 3. Compute lateral jerk: jt = d(at)/dt
    for (size_t i = 0; i < n; ++i) {
        float dt = 0.0f;
        if (i == 0) {
            dt = wp[1].t - wp[0].t;
            wp[i].jt = (wp[1].at - wp[0].at) / dt;
        } else if (i == n - 1) {
            dt = wp[n-1].t - wp[n-2].t;
            wp[i].jt = (wp[n-1].at - wp[n-2].at) / dt;
        } else {
            dt = wp[i+1].t - wp[i-1].t;
            wp[i].jt = (wp[i+1].at - wp[i-1].at) / dt;
        }
    }
}

int main(int argc, char* argv[]) {
    std::string csv_file = "waypoints.csv";
    if (argc > 1) csv_file = argv[1];

    std::ifstream file(csv_file);
    if (!file.is_open()) {
        std::cerr << "Error opening CSV file: " << csv_file << std::endl;
        return 1;
    }

    std::string line, col;
    std::vector<std::string> headers;
    
    // Read header line
    if (std::getline(file, line)) {
        std::stringstream ss(line);
        while (std::getline(ss, col, ',')) {
            // Trim whitespace/newlines
            col.erase(0, col.find_first_not_of(" \r\n\t"));
            col.erase(col.find_last_not_of(" \r\n\t") + 1);
            headers.push_back(col);
        }
    }

    // Map column names to indices
    int idx_t = -1, idx_x = -1, idx_y = -1, idx_vx = -1, idx_vy = -1;
    for (size_t i = 0; i < headers.size(); ++i) {
        if (headers[i] == "x_t") idx_t = i;
        else if (headers[i] == "x_q_0") idx_x = i;
        else if (headers[i] == "x_q_1") idx_y = i;
        else if (headers[i] == "x_dq_0") idx_vx = i;
        else if (headers[i] == "x_dq_1") idx_vy = i;
    }

    if (idx_t < 0 || idx_x < 0 || idx_y < 0 || idx_vx < 0 || idx_vy < 0) {
        std::cerr << "Missing required columns in CSV header!" << std::endl;
        return 1;
    }

    std::vector<WaypointData> waypoints;
    
    // Parse rows
    while (std::getline(file, line)) {
        if (line.empty()) continue;
        std::stringstream ss(line);
        std::vector<float> row;
        while (std::getline(ss, col, ',')) {
            row.push_back(std::stof(col));
        }

        WaypointData wp;
        wp.t  = row[idx_t];
        wp.x  = row[idx_x];
        wp.y  = row[idx_y];
        wp.vx = row[idx_vx];
        wp.vy = row[idx_vy];
        waypoints.push_back(wp);
    }
    file.close();

    // Compute derivative fields (at and jt)
    compute_derivatives(waypoints);

    // Setup UDP socket for MAVLink transmission to PX4 SITL
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    sockaddr_in target_addr{};
    target_addr.sin_family = AF_INET;
    target_addr.sin_port = htons(14580); // Standard PX4 SITL UDP port
    inet_pton(AF_INET, "127.0.0.1", &target_addr.sin_addr);

    std::cout << "Sending " << waypoints.size() << " trajectory setpoints to PX4..." << std::endl;

    // Send MAVLink packets
    for (uint16_t i = 0; i < waypoints.size(); ++i) {
        const auto& wp = waypoints[i];

        mavlink_message_t msg;
        
        // Pack message id=601 (TRAJECTORY_SETPOINT_UPLOAD)
        // Adjust the packing function name according to your generated headers
        mavlink_msg_trajectory_setpoint_upload_pack(
            1,                  // System ID (GCS)
            200,                // Component ID
            &msg,
            i,                  // index
            10,                 // id = 10
            wp.x,               // x
            wp.y,               // y
            wp.vx,              // vx
            wp.vy,              // vy
            wp.at,              // at
            wp.jt,              // jt
            wp.t
        );

        uint8_t buf[MAVLINK_MAX_PACKET_LEN];
        uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

        sendto(sock, buf, len, 0, (struct sockaddr*)&target_addr, sizeof(target_addr));
        
        // Small delay between packets (1ms) to prevent UDP overflow
        usleep(1000);
    }

    close(sock);
    std::cout << "Upload completed!" << std::endl;
    return 0;
}
