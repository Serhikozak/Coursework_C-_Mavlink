#pragma once
#include <cstdint>

namespace MavlinkConfig {
    // Налаштування зв'язку
    const char* const DEFAULT_SERIAL_PORT = "/dev/ttyAMA0";
    constexpr uint32_t BAUDRATE = 115200;

    // Параметри місії
    constexpr float TARGET_ALTITUDE_M = 100.0f;
    //constexpr float FLIGHT_DISTANCE_M = 200.0f;

    struct GPS_Point{
        int32_t lat; // В мікродегр. (1e-7)
        int32_t lon; // В мікродегр. (1e-7)
    };
    
    // Адресація MAVLink (Імітуємо Onboard Computer / GCS)
    constexpr uint8_t SYS_ID   = 1;
    constexpr uint8_t COMP_ID  = 191; // MAV_COMP_ID_ONBOARD_COMPUTER
    constexpr uint8_t TGT_SYS  = 1;
    constexpr uint8_t TGT_COMP = 1;

    // Кастомні режими ArduCopter
    constexpr uint32_t MODE_GUIDED = 4;
    constexpr uint32_t MODE_RTL = 6;
}