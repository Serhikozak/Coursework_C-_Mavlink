#include "../include/MissionProcessor.h"
#include <iostream>
#include <cmath>

MissionProcessor::MissionProcessor() 
    : m_mission_id(1), m_target_hit_radius(3.0f) {}

void MissionProcessor::logBallistics(float alt, float speed, float tof, float lead_dist, float remaining_dist) {
    std::cout << "[📊 MissionProcessor] АТАКА ЦІЛІ (Місія #" << m_mission_id << ")\n"
              << "                   Поточна висота H (Z0): " << alt << " м\n"
              << "                   Поточна швидкість V0 : " << speed << " м/с\n"
              << "                   Аналітичний ToF (Кардано): " << tof << " с\n"
              << "                   Балістичний винос (Тейлор): " << lead_dist << " м\n"
              << "                   Залишок дистанції до цілі: " << remaining_dist << " м\n"
              << "                   Прогноз промаху: " << std::abs(remaining_dist - lead_dist) << " м\n"
              << "-------------------------------------------------------\n";
}

bool MissionProcessor::isFlightStableForDrop(float yaw_error) const {
    // Якщо дрон сильно крутить носом (помилка курсу > 0.1 рад), скидати не можна, бо бомба полетить убік
    return std::abs(yaw_error) <= 0.1f;
}