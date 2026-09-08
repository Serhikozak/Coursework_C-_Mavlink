#pragma once
#include "IDroneAutopilot.h"
#include "Types.h"
#include <thread>
#include <vector>
#include <netinet/in.h>

class DroneStateMachine; // Forward declaration for the state machine class

class DroneAutopilotImpl : public IDroneAutopilot {
private:
    std::thread m_io_thread;                 // Потік 1: Читання/Запис MAVLink та робота з сокетом
    std::thread m_mission_processor_thread;  // Потік 2: Обробник місії та станів (FSM)
    
    std::vector<Coord> m_patrol_points;
    size_t m_current_wp_index;
    FlightState m_state;

    // Робочі методи паралельних потоків
    void runIO(SharedData& shared);
    void runMissionProcessor(SharedData& shared);

    //Допоміжні методи для мережевої балістики MAVLink 2
    GPSPosition convertLocalToGPS(float x, float y) const;
    bool sendDropCommandWithAck(int sock, const sockaddr_in& remote_addr, float drop_x, float drop_y, float drop_alt, 
        SharedData& shared);

    float currentPointToTarget(const Coord& p1, const Coord& p2) const;
    
    

public:
    DroneAutopilotImpl();
    virtual ~DroneAutopilotImpl();

    void start(SharedData& shared) override;
    void stop() override;
};