#pragma once
#include "IMavlinkVehicle.hpp"
#include "MavlinkConfig.hpp"
#include <mavlink/common/mavlink.h>
#include <atomic>
#include <thread>
#include <mutex>
#include <map>
#include <cstdint>
#include <vector>

class SITLMavlinkVehicle : public IMavlinkVehicle {
private:
    int link_fd{-1};
    std::atomic<bool> is_running{false};
    
    std::thread io_thread;
    std::thread telemetry_thread;
    std::mutex write_mutex;

    // Стан транспортного засобу
    std::atomic<uint32_t> current_mode{0};
    std::atomic<bool> cmd_accepted{false};
    std::atomic<uint16_t> awaiting_cmd{0};
    std::atomic<bool> rc_override_active{false};
    std::map<uint8_t, uint64_t> last_heartbeat;

    std::atomic<float> current_alt{0.0f};
    std::atomic<float> local_pos_x{0.0f}; // Позиція North (X)
    std::atomic<float> local_pos_y{0.0f}; // Позиція East (Y)
    std::atomic<float> current_roll{0.0f};
    std::atomic<float> current_pitch{0.0f};
    std::atomic<float> current_yaw{0.0f};
    std::atomic<int32_t> current_lat{0};
    std::atomic<int32_t> current_lon{0};
    std::atomic<bool> ekf_healthy{false};

    std::atomic<int32_t> dynamic_target_lat{0};
    std::atomic<int32_t> dynamic_target_lon{0};
    std::atomic<float> dynamic_target_alt{0.0f};
    std::atomic<bool> new_target_available{false};
    std::atomic<float> current_ground_speed{0.0f};
    

    // Внутрішні хелпери
    int openSerial(const std::string& port);
    void sendMavlinkMessage(mavlink_message_t* msg);
    void handleIncomingMessage(const mavlink_message_t* msg);
    bool waitForAck(uint16_t cmd, uint32_t timeout_ms = 2000);
    uint64_t getTimestampMs();

public:
    SITLMavlinkVehicle() = default;
    ~SITLMavlinkVehicle() override { disconnect(); }

    bool connect(const std::string& port) override;
    void disconnect() override;
    
    void startIOLoop() override;
    void startTelemetryLoop() override;

    bool setMode(uint32_t mode) override;
    bool armMotors() override;
    bool takeoff(float altitude) override;
    bool moveToLocalOffset(float north, float east, float down) override;

    float getCurrentAltitude() override { return current_alt.load(); }
    bool isEkfHealthy() override { return ekf_healthy.load(); }
    bool isTargetReached(float target_n, float target_e, float tolerance = 1.5f) override;
    bool flyTowardsGlobalFast(int32_t target_lat, int32_t target_lon, float alt_relative, float speed_mps) override;

    // --- НОВІ МЕТОДИ ДЛЯ GPS НАВІГАЦІЇ ---
    bool moveToGlobalWithAccel(int32_t lat, int32_t lon, float alt_relative, float max_speed, float accel) override;
    bool patrolRoute(const std::vector<MavlinkConfig::GPS_Point>& route, float alt_relative, float speed_mps, int lap_count) override;
    bool hasNewDynamicTarget() override {return new_target_available.load(); } // Поки що не реалізовано
    void getDynamicTarget(int32_t& out_lat, int32_t& out_lon, float& out_alt) override {
        out_lat = dynamic_target_lat.load();
        out_lon = dynamic_target_lon.load();
        out_alt = dynamic_target_alt.load();
    }
    void clearDynamicTargetFlag() override { new_target_available = false; }
   
    // bool patrolCircle(int32_t center_lat, int32_t center_lon, float alt_relative, float radius_m, float speed_mps) override;
    int32_t getCurrentLat() override { return current_lat.load(); }
    int32_t getCurrentLon() override { return current_lon.load(); }
    bool isGlobalTargetReached(int32_t target_lat, int32_t target_lon, float tolerance_m = 3.0f) override;
    float getCurrentGroundSpeed() override { return current_ground_speed.load(); }
    bool setParameter(const char* param_id, float value) override;
    
};