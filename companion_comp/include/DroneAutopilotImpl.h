#pragma once
#include "IDroneAutopilot.h"
#include "Types.h"          // ФІКС: Підключаємо ваші типи (SharedData, FlightState, Coord)
#include <thread>
#include <vector>
#include <netinet/in.h>     // ФІКС: Підключаємо структури сокетів Linux (sockaddr_in)

class DroneAutopilotImpl : public IDroneAutopilot {
private:
    std::thread m_io_thread;
    std::thread m_mission_processor_thread;
    
    std::vector<Coord> m_patrol_points;
    size_t m_current_wp_index;
    FlightState m_state;

    float currentPointToTarget(const Coord& p1, const Coord& p2) const;

    // Робочі методи паралельних потоків
    void runIO(SharedData& shared);
    void runMissionProcessor(SharedData& shared);

    // [ФІКС ПОМИЛКИ]: Додаємо оголошення методів для балістики та MAVLink 2
    GPSPosition convertLocalToGPS(float x, float y) const;
   

public:
    DroneAutopilotImpl();
    ~DroneAutopilotImpl() override; // override тут правильний для віртуального деструктора

    void start(SharedData& shared) override;
    void stop() override;

    bool sendDropCommandWithAck(int sock, const sockaddr_in& remote_addr, float drop_x, float drop_y, float drop_alt, SharedData& shared);
};