#pragma once
#include <mutex>
#include <atomic>

// Константи для сигналів пульта (PWM мікросекунди)
constexpr int RC_CENTER      = 1500;
constexpr int RC_MAX_FORWARD = 1100; // Максимальне прискорення вперед
constexpr int RC_MIN_FORWARD = 1420; // Плавний рух
constexpr int RC_IGNORE      = 0;    // Передати контроль оператору

// Стан польотної місії (FSM)
enum class FlightState {
    MANUAL_OVERRIDE  = 0, // Керує оператор, автопілот шле 0
    PATROLLING       = 1,      // Патрулювання за маршрутом
    TARGET_ATTACK    = 2,   // Виявлено ціль: розгін та наведення (прискорення)
    PAYLOAD_DROP     = 3,    // Точка досягнута: активація скиду
    MISSION_COMPLETE = 4 // Завдання виконано
};

// Переіменована структура для локальних метричних координат (NED)
struct Coord {
    float x = 0.0f; // Північ (North) в метрах
    float y = 0.0f; // Схід (East) в метрах
};

// Телеметрія від польотника
struct Telemetry {
    Coord pos;
    float current_yaw = 0.0f; // Курс в радіанах
    float groundSpeed = 0.0f; // НОВА ЗМІННА: Швидкість дрона в м/с (для V0)
    float z           = 0.0f; // НОВА ЗМІННА: Висота в системі NED (напрямлена вниз)
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