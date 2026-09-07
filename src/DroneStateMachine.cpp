#include "../include/DroneAutopilotImpl.h"
#include "../include/AnaliticalSolver.h"
#include <cmath>
#include <iostream>
#include <chrono>
#include <algorithm>

// ПОТІК 2: Обробник місії та станів (MissionProcessor / FSM)
void DroneAutopilotImpl::runMissionProcessor(SharedData& shared) {
    std::cout << "[🧠 MissionProcessor] Потік обробки логіки FSM запущен (Частота: 10 Гц).\n";
    
    while (shared.is_running) {
        RcChannels local_rc;
        
        // ПРІОРИТЕТ БЕЗПЕКИ: Якщо спрацював Failsafe або оператор вимкнув тумблер — MANUAL_OVERRIDE
        if (shared.link_lost || !shared.operator_switch) {
            m_state = FlightState::MANUAL_OVERRIDE;
        }

        // Локальне копіювання спільних ресурсів під захистом м'ютексу
        Telemetry local_telemetry;
        Coord local_target;
        bool local_target_found;
        {
            std::lock_guard<std::mutex> lock(shared.mtx);
            local_telemetry = shared.telemetry;
            local_target = shared.target_pos;
            local_target_found = shared.target_found;
        }

        // Головний кінцевий автомат (FSM) тактичної місії
        switch (m_state) {
            case FlightState::MANUAL_OVERRIDE:
                local_rc.roll = local_rc.pitch = local_rc.throttle = local_rc.yaw = RC_IGNORE;
                local_rc.aux_drop = 1000; 

                if (shared.operator_switch && !shared.link_lost) {
                    m_state = FlightState::PATROLLING;
                    std::cout << "[MissionProcessor] Стан [" << static_cast<int>(m_state) << "]: Перехід в режим ПАТРУЛЮВАННЯ.\n";
                }
                break;

            case FlightState::PATROLLING: {
                Coord current_wp = m_patrol_points[m_current_wp_index];
                float distanceToWaypoint = currentPointToTarget(local_telemetry.pos, current_wp);

                std::cout << "[MissionProcessor] Патрулювання -> Маршрутна WP #" << m_current_wp_index 
                          << " | Дистанція: " << distanceToWaypoint << "м | Курс Yaw: " << local_telemetry.current_yaw << " рад\n";

                local_rc.pitch = RC_MIN_FORWARD; 
                local_rc.throttle = RC_CENTER;   

                // --- ТРИГОНОМЕТРИЧНИЙ РОЗРАХУНОК НАВЕДЕННЯ КУРСУ НА ТОЧКУ МАРШРУТУ ---
                float target_angle = std::atan2(current_wp.y - local_telemetry.pos.y, current_wp.x - local_telemetry.pos.x);
                float yaw_error = target_angle - local_telemetry.current_yaw;
                
                while (yaw_error > M_PI)  yaw_error -= 2.0f * M_PI;
                while (yaw_error < -M_PI) yaw_error += 2.0f * M_PI;

                if (std::abs(yaw_error) > 0.05f) {
                    local_rc.yaw = RC_CENTER + std::clamp(static_cast<int>(yaw_error * 150.0f), -150, 150);
                } else {
                    local_rc.yaw = RC_CENTER; 
                }

                if (distanceToWaypoint < 3.0f) {
                    m_current_wp_index = (m_current_wp_index + 1) % m_patrol_points.size();
                }

                if (local_target_found) {
                    m_state = FlightState::TARGET_ATTACK;
                    std::cout << "[MissionProcessor] [ALERT] Зміна стану -> [" << static_cast<int>(m_state) << "] ПЕРЕХВАТ ТА АТАКА ЦІЛІ!\n";
                }
                break;
            }

            case FlightState::TARGET_ATTACK: {
                float dist_to_target = currentPointToTarget(local_telemetry.pos, local_target);
                
                float Z0 = -local_telemetry.z; 
                if (Z0 < 1.0f) Z0 = 10.0f;          
                
                float V0 = local_telemetry.groundSpeed; 
                if (V0 < 0.1f) V0 = 0.1f;               
                
                constexpr float m = 1.5f;   
                constexpr float d = 0.15f;  
                constexpr float l = 0.05f;  

                // --- РОЗРАХУНОК БАЛІСТИЧНОГО ВИНОСУ (МЕТОДИ КАРДАНО ТА ТЕЙЛОРА) ---
                float tof = AnaliticalSolver::calcTimeOfFlight(Z0, V0, m, d, l);
                float drop_lead_distance = AnaliticalSolver::calcHDistance(tof, V0, m, d, l);
                
                std::cout << "[MissionProcessor] АТАКА. Висота: " << Z0 << "м | Швидкість V0: " << V0 << "м/с\n"
                          << "                   Час падіння (ToF): " << tof << "с | Упередження скиду: " << drop_lead_distance << "м\n"
                          << "                   Залишок дистанції до цілі: " << dist_to_target << "м\n";

                // --- НАВЕДЕННЯ НОСА (YAW) СТРОГО НА ЦІЛЬ ---
                float target_angle = std::atan2(local_target.y - local_telemetry.pos.y, local_target.x - local_telemetry.pos.x);
                float yaw_error = target_angle - local_telemetry.current_yaw;
                while (yaw_error > M_PI)  yaw_error -= 2.0f * M_PI;
                while (yaw_error < -M_PI) yaw_error += 2.0f * M_PI;
                
                if (std::abs(yaw_error) > 0.05f) {
                    local_rc.yaw = RC_CENTER + std::clamp(static_cast<int>(yaw_error * 150.0f), -150, 150);
                } else {
                    local_rc.yaw = RC_CENTER;
                }

                // --- КЕРУВАННЯ ШВИДКІСТЮ ТА ТРИГЕР СКIDУ (CCIP) ---
                if (dist_to_target > (drop_lead_distance + 10.0f)) {
                    local_rc.pitch = RC_MAX_FORWARD; 
                    local_rc.throttle = 1650;        
                } else {
                    local_rc.pitch = 1430;
                    local_rc.throttle = 1500;
                    
                    if (dist_to_target <= drop_lead_distance) {
                        m_state = FlightState::PAYLOAD_DROP; // ТРИГЕР СКIDУ ВАНТАЖУ!
                    }
                }
                break;
            }

            case FlightState::PAYLOAD_DROP:
                std::cout << "[MissionProcessor] Стан [" << static_cast<int>(m_state) 
                          << "]: 🎯 НАД БАЛІСТИЧНОЮ ТОЧКОЮ! АКТИВАЦІЯ СКIDУ (CH6 = 2000)\n";
                local_rc.pitch = RC_CENTER;
                local_rc.throttle = RC_CENTER;
                local_rc.aux_drop = 2000; 

                m_state = FlightState::MISSION_COMPLETE;
                break;

            case FlightState::MISSION_COMPLETE:
                local_rc.pitch = RC_CENTER;
                local_rc.throttle = RC_CENTER;
                local_rc.aux_drop = 1000; 
                break;
        }

        // Записуємо сформовані команди назад у спільну пам'ять для IO потоку
        {
            std::lock_guard<std::mutex> lock(shared.mtx);
            shared.output_rc = local_rc;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 10 Гц
    }
    std::cout << "[🧠 MissionProcessor] Потік обробки логіки зупинено.\n";
}