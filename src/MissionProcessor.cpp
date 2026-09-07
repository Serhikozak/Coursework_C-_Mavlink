#include "../include/DroneAutopilotImpl.h"
#include "../include/AnaliticalSolver.h"
#include <cmath>
#include <iostream>
#include <chrono>
#include <algorithm>
#include <mavlink.h>

// Мережеві системні заголовки Linux для UDP сокету
#include <sys/socket.h> 
#include <netinet/in.h>
#include <fcntl.h>
#include <unistd.h>

DroneAutopilotImpl::DroneAutopilotImpl() 
    : m_current_wp_index(0), m_state(FlightState::MANUAL_OVERRIDE) 
{
    // Задаємо маршрут патрулювання в метрах довкола умовної бази
    m_patrol_points = {{0.0f, 40.0f}, {40.0f, 40.0f}, {40.0f, 0.0f}, {0.0f, 0.0f}};
}

DroneAutopilotImpl::~DroneAutopilotImpl() {
    stop();
}

float DroneAutopilotImpl::currentPointToTarget(const Coord& p1, const Coord& p2) const {
    return std::hypot(p1.x - p2.x, p1.y - p2.y);
}

void DroneAutopilotImpl::start(SharedData& shared) {
    shared.is_running = true;
    // Активация двух независимых параллельных потоков бортового компьютера
    m_io_thread = std::thread(&DroneAutopilotImpl::runIO, this, std::ref(shared));
    m_mission_processor_thread = std::thread(&DroneAutopilotImpl::runMissionProcessor, this, std::ref(shared));
    std::cout << "[SYSTEM] Потоки IO та MissionProcessor успішно активовано.\n";
}

void DroneAutopilotImpl::stop() {
    if (m_io_thread.joinable()) m_io_thread.join();
    if (m_mission_processor_thread.joinable()) m_mission_processor_thread.join();
}

// ПОТІК 1: Реалізація асинхронного введення-виведення UDP та Failsafe
void DroneAutopilotImpl::runIO(SharedData& shared) {
    std::cout << "[⚙️ IO_THREAD] Потік мережевого введення-виведення запущено (100 Гц).\n";
    
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        std::cerr << "[⚙️ IO_THREAD] Помилка створення сокету.\n";
        shared.is_running = false;
        return;
    }
    // Робимо сокет неблокуючим, щоб таймаут Failsafe працював без зависань
    int flags = fcntl(sock, F_GETFL, 0);
    fcntl(sock, F_SETFL, flags | O_NONBLOCK);

    sockaddr_in local_addr{};
    local_addr.sin_family = AF_INET;
    local_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    local_addr.sin_port = htons(14551);
    
     if (bind(sock, (struct sockaddr*)&local_addr, sizeof(local_addr)) < 0) {
        std::cerr << "[⚙️ IO_THREAD] Помилка прив'язки до порту 14551.\n";
        close(sock);
        shared.is_running = false;
        return;
    }
    // Буфер для сетевых пакетов, структуры парсера и адресации ответов
    uint8_t rx_buffer[MAVLINK_MAX_PACKET_LEN];

    // Змінні для розшифровки MAVLink
    mavlink_message_t msg;
    mavlink_status_t status;

    sockaddr_in remote_addr{};
    socklen_t remote_addr_len = sizeof(remote_addr);

    auto last_packet_time = std::chrono::steady_clock::now();
    const auto timeout_duration = std::chrono::seconds(3); 

    while (shared.is_running) {
        int bytes_received = recvfrom(sock, rx_buffer, sizeof(rx_buffer), 0,
                                      (struct sockaddr*)&remote_addr, &remote_addr_len);
        auto now = std::chrono::steady_clock::now();

        if (bytes_received > 0) {
            last_packet_time = now;
            shared.link_lost = false;

            for (int i = 0; i < bytes_received; ++i) {
                if (mavlink_parse_char(MAVLINK_COMM_0, rx_buffer[i], &msg, &status)) {
                    switch (msg.msgid) {
                        case MAVLINK_MSG_ID_LOCAL_POSITION_NED: {
                            mavlink_local_position_ned_t local_pos;
                            mavlink_msg_local_position_ned_decode(&msg, &local_pos);
                            {
                                std::lock_guard<std::mutex> lock(shared.mtx);
                                shared.telemetry.pos.x = local_pos.x;
                                shared.telemetry.pos.y = local_pos.y;
                                shared.telemetry.z     = local_pos.z;
                            }
                            break;
                        }
                        case MAVLINK_MSG_ID_ATTITUDE: {
                            mavlink_attitude_t attitude;
                            mavlink_msg_attitude_decode(&msg, &attitude);
                            {
                                std::lock_guard<std::mutex> lock(shared.mtx);
                                shared.telemetry.current_yaw = attitude.yaw;
                            }
                            break;
                        }
                        case MAVLINK_MSG_ID_VFR_HUD: {
                            mavlink_vfr_hud_t vfr_hud;
                            mavlink_msg_vfr_hud_decode(&msg, &vfr_hud);
                            {
                                std::lock_guard<std::mutex> lock(shared.mtx);
                                shared.telemetry.groundSpeed = vfr_hud.groundspeed;
                            }
                            break;
                        }
                    }
                }
            }
        } else {
            if (now - last_packet_time > timeout_duration) {
                if (!shared.link_lost) {
                    std::cerr << "[⚙️ IO_THREAD] [ALERT] Потеря связи с симулятором! Активирован FAILSAFE.\n";
                }
                shared.link_lost = true;
            }
        }

        if (!shared.link_lost && remote_addr.sin_port != 0) {
            RcChannels current_rc;
            {
                std::lock_guard<std::mutex> lock(shared.mtx);
                current_rc = shared.output_rc;
            }

            mavlink_message_t tx_msg;
            uint8_t tx_buffer[MAVLINK_MAX_PACKET_LEN];

            mavlink_msg_rc_channels_override_pack(
                1, 1, &tx_msg, 1, 1,
                current_rc.roll, 
                current_rc.pitch, 
                current_rc.throttle, 
                current_rc.yaw,
                0, 
                current_rc.aux_drop,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 
            );

            uint16_t tx_len = mavlink_msg_to_send_buffer(tx_buffer, &tx_msg);
            sendto(sock, tx_buffer, tx_len, 0, (struct sockaddr*)&remote_addr, remote_addr_len);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    close(sock);
    std::cout << "[⚙️ IO_THREAD] UDP сокет закрыт. Поток успешно остановлен.\n";
}

            