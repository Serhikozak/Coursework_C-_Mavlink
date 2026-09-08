






// Формат виводу — такий самий, як керування від пульта
struct RcChannels {
    int roll = RC_CENTER;
    int pitch = RC_CENTER;
    int throttle = RC_CENTER;
    int yaw = RC_CENTER;
    int aux_drop = 1000; // Канал 6: 1000 - закрито, 2000 - скид
};

// Спільні ресурси для потокобезпечного обміну даними між потоками IO та MissionProcessor
struct SharedData {
    Telemetry telemetry;
    RcChannels output_rc;
    Coord target_pos;
    std::mutex mtx; 
    
    std::atomic<bool> operator_switch{true};
    std::atomic<bool> link_lost{false};
    std::atomic<bool> is_running{true};
    std::atomic<bool> target_found{false};
};





#pragma once
#include <mutex>
#include <atomic>

// =====================================// КОНСТАНТИ СИСТЕМИ КЕРУВАННЯ (PWM мікросекунди)// ================================================================
constexpr int RC_CENTER      = 1500;
constexpr int RC_MAX_FORWARD = 1100; // Максимальне прискорення вперед
constexpr int RC_MIN_FORWARD = 1420; // Плавний рух
constexpr int RC_IGNORE      = 0;    // Передати контроль оператору

constexpr uint8_t  MAV_SYS_ID = 1;
constexpr uint8_t  MAV_COMP_ID      = 1;
constexpr uint16_t MAV_TARGET_PORT  = 14550; // UDP порт 14550
constexpr int ACK_TIMEOUT_MS = 400; // Таймаут очікування ACK пакета
constexpr int MAV_MAX_ATTEMPTS   = 5; // До 5 спроб скиду з очікуванням ACK

// Стан польотної місії (FSM)
// #region Стан польотної місії
enum class FlightState {
    MANUAL_OVERRIDE  = 0, // Керує оператор, автопілот шле 0
    PATROLLING       = 1, // Патрулювання за маршрутом
    TARGET_ATTACK    = 2, // Виявлено ціль: розгін та наведення (прискорення)
    PAYLOAD_DROP     = 3, // Точка досягнута: активація скиду
    MISSION_COMPLETE = 4 // Завдання виконано
};
// #endregion


                                                                                   
// ======================================// СТРУКТУРИ ДАНИХ ТА ГЕОМЕТРІЯ// ============================================================================

// Локальні метричні координати (NED)
struct Coord {
    float x = 0.0f; // Північ (North) в метрах
    float y = 0.0f; // Схід (East) в метрах
};
//Географічні координати для MAVLink пакетів
struct GPSPosition {
    int32_t lat = 0; // Широта (Latitude) -> градуси * 1e7
    int32_t lon = 0; // Довгота (Longitude) -> градуси * 1e7
};
// Телеметрія від польотника
struct Telemetry {
    Coord pos;
    float current_yaw = 0.0f; // Курс в радіанах
    float groundSpeed = 0.0f; // Швидкість дрона в м/с (для V0)
    float z           = 0.0f; // Висота в системі NED (напрямлена вниз!)
};
// Формат виводу — такий самий, як керування від пульта
struct RcChannels {
    int roll = RC_CENTER;
    int pitch = RC_CENTER;
    int throttle = RC_CENTER;
    int yaw = RC_CENTER;
    int aux_drop = 1000; // Канал 6: 1000 - закрито, 2000 - скид
};
// Спільні ресурси для потокобезпечного обміну даними між потоками IO та MissionProcessor
struct SharedData {
    Telemetry telemetry;
    RcChannels output_rc;
    Coord target_pos;
    std::mutex mtx; 
    
    std::atomic<bool> operator_switch{true};
    std::atomic<bool> link_lost{false};
    std::atomic<bool> is_running{true};
    std::atomic<bool> target_found{false};
};




