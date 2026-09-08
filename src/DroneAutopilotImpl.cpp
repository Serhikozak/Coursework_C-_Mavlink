#include "../include/DroneAutopilotImpl.h"
#include "../include/AnaliticalSolver.h"
#include <cmath>
#include <iostream>
#include <chrono>
#include <algorithm>
#include <cstring>

// Системні мережеві заголовки Linux для UDP
#include <sys/socket.h> 
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <unistd.h>

// Підключаємо вашу окрему бібліотеку MAVLink
#include <mavlink.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

DroneAutopilotImpl::DroneAutopilotImpl() 
    : m_current_wp_index(0), m_state(FlightState::MANUAL_OVERRIDE) 
{
    // Маршрут патрулювання в метрах довкола умовної бази (Майдан Незалежності)
    m_patrol_points = {{0.0f, 40.0f}, {40.0f, 40.0f}, {40.0f, 0.0f}, {0.0f, 0.0f}};
}

DroneAutopilotImpl::~DroneAutopilotImpl() {
    stop();
}

float DroneAutopilotImpl::currentPointToTarget(const Coord& p1, const Coord& p2) const {
    return std::hypot(p1.x - p2.x, p1.y - p2.y);
}

// Формули перерахунку метричних NED координат у GPS (Згідно з ТЗ викладача)
GPSPosition DroneAutopilotImpl::convertLocalToGPS(float x, float y) const {
    const double lat0 = 50.4501;
    const double lon0 = 30.5234;
    const double meters_per_lat_degree = 111320.0;

    double lat = lat0 + (y / meters_per_lat_degree);
    double lon = lon0 + (x / (meters_per_lat_degree * std::cos(lat0 * M_PI / 180.0)));

    GPSPosition pos;
    pos.lat = static_cast<int32_t>(lat * 1e7);
    pos.lon = static_cast<int32_t>(lon * 1e7);
    return pos;
}

void DroneAutopilotImpl::start(SharedData& shared) {
    shared.is_running = true;
    m_io_thread = std::thread(&DroneAutopilotImpl::runIO, this, std::ref(shared));
    m_mission_processor_thread = std::thread(&DroneAutopilotImpl::runMissionProcessor, this, std::ref(shared));
    std::cout << "[SYSTEM] Потоки IO та MissionProcessor успішно активовано.\n";
}

void DroneAutopilotImpl::stop() {
    if (m_io_thread.joinable()) m_io_thread.join();
    if (m_mission_processor_thread.joinable()) m_mission_processor_thread.join();
}

// ПОТІК 1: Реалізація асинхронного введення-виведення UDP та Failsafe телеметрії
void DroneAutopilotImpl::runIO(SharedData& shared) {
    std::cout << "[⚙️ IO_THREAD] Потік мережевого введення-виведення запущено (100 Гц).\n";
    
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        std::cerr << "[⚙️ IO_THREAD] Помилка створення сокету.\n";
        shared.is_running = false;
        return;
    }

    // Неблокуючий режим для захисту від зависань recvfrom
    int flags = fcntl(sock, F_GETFL, 0);
    fcntl(sock, F_SETFL, flags | O_NONBLOCK);

    sockaddr_in local_addr{};
    local_addr.sin_family = AF_INET;
    local_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    local_addr.sin_port = htons(14551); // Слухаємо симулятор на 14551

    if (bind(sock, (struct sockaddr*)&local_addr, sizeof(local_addr)) < 0) {
        std::cerr << "[⚙️ IO_THREAD] Помилка прив'язки до порту 14551.\n";
        close(sock);
        shared.is_running = false;
        return;
    }

    uint8_t rx_buffer[MAVLINK_MAX_PACKET_LEN];
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
                                shared.telemetry.z     = local_pos.z; // Напрямлена вниз
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
                    std::cerr << "[⚙️ IO_THREAD] [ALERT] Втрата зв'язку з симулятором! Активовано FAILSAFE.\n";
                }
                shared.link_lost = true;
            }
        }

        // Якщо зв'язок є, надсилаємо зворотні команди керування
        if (!shared.link_lost && remote_addr.sin_port != 0) {
            FlightState current_state;
            RcChannels current_rc;
            Telemetry current_tel;
            {
                std::lock_guard<std::mutex> lock(shared.mtx);
                current_state = m_state;
                current_rc = shared.output_rc;
                current_tel = shared.telemetry;
            }

            //Якщо автомат перейшов у PAYLOAD_DROP — запускаємо бойовий контур MAVLink 2
            if (current_state == FlightState::PAYLOAD_DROP) {
                std::cout << "[⚙️ IO_THREAD] [ACTION] Активація процедури скиду через MAVLink 2 по UDP...\n";
                float drop_alt = -current_tel.z; // Інвертуємо NED Z у додатну висоту над землею

                // Запускаємо надійний метод з 5 спробами (передаємо shared для перевірки тумблера оператора)
                bool success = sendDropCommandWithAck(sock, remote_addr, current_tel.pos.x, current_tel.pos.y, drop_alt, shared);
                
                if (success) {
                    std::lock_guard<std::mutex> lock(shared.mtx);
                    m_state = FlightState::MISSION_COMPLETE;
                    shared.output_rc.aux_drop = 2000; // Фіксуємо відкриття засувки
                } else {
                    std::lock_guard<std::mutex> lock(shared.mtx);
                    m_state = FlightState::PATROLLING; // При збої або ручному перехопленні повертаємо патруль
                }
            }

            // Надсилання стандартного оверрайду радіоканалів пульта
            mavlink_message_t tx_msg;
            uint8_t tx_buffer[MAVLINK_MAX_PACKET_LEN];

            mavlink_msg_rc_channels_override_pack(
                MAV_SYS_ID, MAV_COMP_ID, &tx_msg, 1, 1,
                current_rc.roll, current_rc.pitch, current_rc.throttle, current_rc.yaw,
                0, current_rc.aux_drop, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 
            );

            uint16_t tx_len = mavlink_msg_to_send_buffer(tx_buffer, &tx_msg);
            sendto(sock, tx_buffer, tx_len, 0, (struct sockaddr*)&remote_addr, remote_addr_len);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Контур 100 Гц / 10 Гц обміну
    }

    close(sock);
    std::cout << "[⚙️ IO_THREAD] UDP сокет закрито. Поток успішно зупинено.\n";
}

// Реалізація бойового контуру з 5 спробами та перевіркою тумблера оператора в реальному часі
bool DroneAutopilotImpl::sendDropCommandWithAck(int sock, const sockaddr_in& remote_addr, float drop_x, float drop_y, float drop_alt, SharedData& shared) {
    GPSPosition gps_drop = convertLocalToGPS(drop_x, drop_y);
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    // Формуємо кадр COMMAND_LONG з командою 31010 (MAV_CMD_USER_1)
    mavlink_msg_command_long_pack(
        MAV_SYS_ID, MAV_COMP_ID, &msg, 1, 1, 
        31010, 0, 0, 0, 0, 0, 
        (float)gps_drop.lat, (float)gps_drop.lon, drop_alt
    );
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    socklen_t remote_len = sizeof(remote_addr);

    for (int attempt = 1; attempt <= MAV_MAX_ATTEMPTS; ++attempt) {
        // [БЕЗПЕКА]: Якщо оператор у будь-який момент часу забрав керування — миттєво припиняємо зліт/скид
        if (shared.link_lost || !shared.operator_switch) {
            std::cerr << "[MavLink] [ALERT] Оператор перехопив керування! Скасування бойової команди.\n";
            return false;
        }

         std::cout << "[MavLink] Надсилання кадру COMMAND_LONG (Спроба " << attempt << "/5)..." << std::endl;
        sendto(sock, buf, len, 0, (struct sockaddr*)&remote_addr, remote_len);

        auto start_wait = std::chrono::steady_clock::now();
        // Мікроцикл очікування ACK пакету (400 мс з Types.h)
        while (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start_wait).count() < ACK_TIMEOUT_MS) {
            if (shared.link_lost || !shared.operator_switch) {
                return false; 
            }

            uint8_t rx_buf[MAVLINK_MAX_PACKET_LEN];
            int rx_len = recvfrom(sock, rx_buf, sizeof(rx_buf), 0, nullptr, nullptr);
            if (rx_len > 0) {
                mavlink_message_t rx_msg;
                mavlink_status_t rx_status;
                for (int i = 0; i < rx_len; ++i) {
                    if (mavlink_parse_char(MAVLINK_COMM_0, rx_buf[i], &rx_msg, &rx_status)) {
                        if (rx_msg.msgid == MAVLINK_MSG_ID_COMMAND_ACK) {
                            mavlink_command_ack_t ack;
                            mavlink_msg_command_ack_decode(&rx_msg, &ack);
                            if (ack.command == 31010 && ack.result == MAV_RESULT_ACCEPTED) {
                                return true; // Скид успішно підтверджено чекером!
                            }
                        }
                        
                    }
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }
    return false; // Після 5 спроб без успіху — повертаємо false
}

