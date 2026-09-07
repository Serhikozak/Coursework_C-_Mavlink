#pragma once
#include "IDroneAutopilot.h"
#include <thread>
#include <vector>

class DroneAutopilotImpl : public IDroneAutopilot {
private:
    std::thread m_io_thread;                 // Потік 1: Читання/Запис MAVLink та робота з сокетом
    std::thread m_mission_processor_thread;  // Потік 2: Обробник місії та станів (FSM)
    
    std::vector<Coord> m_patrol_points;
    size_t m_current_wp_index;
    FlightState m_state;

    float currentPointToTarget(const Coord& p1, const Coord& p2) const;
    
    // Робочі методи паралельних потоків
    void runIO(SharedData& shared);
    void runMissionProcessor(SharedData& shared);

public:
    DroneAutopilotImpl();
    ~DroneAutopilotImpl() override;

    void start(SharedData& shared) override;
    void stop() override;
};