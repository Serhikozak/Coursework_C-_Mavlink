#pragma once
#include "Types.h"
#include <vector>

class MissionProcessor {
private:
    int m_mission_id;
    float m_target_hit_radius;

public:
    MissionProcessor();
    ~MissionProcessor() = default;

    // Метод для красивого виводу балістичних логів у термінал бортового комп'ютера
    void logBallistics(float alt, float speed, float tof, float lead_dist, float remaining_dist);
    
    // Перевірка, чи безпечні поточні кути нахилу для виконання скиду
    bool isFlightStableForDrop(float yaw_error) const;
};