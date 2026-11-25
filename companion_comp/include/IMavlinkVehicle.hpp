#pragma once
#include <string>
#include <cstdint>
#include <vector>

namespace MavlinkConfig {
    struct GPS_Point;
}

class IMavlinkVehicle {
public:
    virtual ~IMavlinkVehicle() = default;

    // Життєвий цикл підключення
    virtual bool connect(const std::string& port) = 0;
    virtual void disconnect() = 0;

    // Потоки
    virtual void startIOLoop() = 0;        // Потік введення-виведення (читання/запис UART)
    virtual void startTelemetryLoop() = 0; // Потік аналізу телеметрії, EKF3 та Heartbeats

    // Високорівневі команди керування
    virtual bool setMode(uint32_t mode) = 0;
    virtual bool armMotors() = 0;
    virtual bool takeoff(float altitude) = 0;
    virtual bool moveToLocalOffset(float north, float east, float down) = 0;

    // --- МЕТОДИ ДЛЯ GPS НАВІГАЦІЇ ---
    // Політ у точку із заданим обмеженням швидкості та прискорення
    virtual bool moveToGlobalWithAccel(int32_t lat, int32_t lon, float alt_relative, float max_speed, float accel) = 0;
    virtual bool patrolRoute(const std::vector<MavlinkConfig::GPS_Point>& route, float alt_relative, float speed_mps, int lap_count) = 0;

    virtual bool hasNewDynamicTarget() = 0;
    virtual void getDynamicTarget(int32_t& out_lat, int32_t& out_lon, float& out_alt) = 0;
    virtual void clearDynamicTargetFlag() = 0;
    // Патрулювання навколо центру (по колу заданого радіусу)
    //virtual bool patrolCircle(int32_t center_lat, int32_t center_lon, float alt_rel, float radius_m, float speed_mps) = 0;
    virtual int32_t getCurrentLat() = 0;
    virtual int32_t getCurrentLon() = 0;
    virtual bool isGlobalTargetReached(int32_t target_lat, int32_t target_lon, float tolerance_m = 3.0f) = 0;
    virtual bool flyTowardsGlobalFast(int32_t target_lat, int32_t target_lon, float alt_relative, float speed_mps) = 0;


    // --- ДИНАМІЧНИЙ ПЕРЕХІД (зчитування поточної висоти та координат з повідомлення VFR_HUD / GLOBAL_POSITION_INT)  ---
    virtual float getCurrentAltitude() = 0;
    virtual bool isEkfHealthy() = 0;
    virtual bool isTargetReached(float target_n, float target_e, float tolerance = 1.5f) = 0;

    virtual float getCurrentGroundSpeed() = 0;
    virtual bool setParameter(const char* param_id, float value) = 0;
};