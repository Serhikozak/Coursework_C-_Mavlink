#include "SITLMavlinkVehicle.hpp"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <chrono>
#include <cmath>
#include <vector>

uint64_t SITLMavlinkVehicle::getTimestampMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
}

int SITLMavlinkVehicle::openSerial(const std::string& port) {
    int fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) return -1;

    struct termios tio;
    if (tcgetattr(fd, &tio) != 0) { close(fd); return -1; }
    cfmakeraw(&tio);
    tio.c_cflag &= ~CRTSCTS;
    tio.c_cflag &= ~(IXON | IXOFF | IXANY);
    cfsetispeed(&tio, B115200);
    cfsetospeed(&tio, B115200);
    tio.c_cflag |= (CLOCAL | CREAD);
    
    tcflush(fd, TCIOFLUSH);
    if (tcsetattr(fd, TCSANOW, &tio) != 0) { close(fd); return -1; }
    fcntl(fd, F_SETFL, 0);
    return fd;
}

bool SITLMavlinkVehicle::connect(const std::string& port) {
    link_fd = openSerial(port);
    if (link_fd == -1) return false;
    is_running = true;
    return true;
}

void SITLMavlinkVehicle::disconnect() {
    is_running = false;
    if (io_thread.joinable()) io_thread.join();
    if (telemetry_thread.joinable()) telemetry_thread.join();
    if (link_fd != -1) { close(link_fd); link_fd = -1; }
}

void SITLMavlinkVehicle::sendMavlinkMessage(mavlink_message_t* msg) {
    std::lock_guard<std::mutex> lock(write_mutex);
    if (link_fd == -1) return;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, msg);
    write(link_fd, buf, len);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
}

// ПОРЦІЯ АРХІТЕКТУРИ: Потік №1 — Pure IO Loop (Низькорівневе затягування байтів з шини)
void SITLMavlinkVehicle::startIOLoop() {
    io_thread = std::thread([this]() {
        mavlink_message_t msg;
        mavlink_status_t st;
        uint8_t buffer[256];
        
        while (is_running) {
            int n = read(link_fd, buffer, sizeof(buffer));
            if (n > 0) {
                for (int i = 0; i < n; ++i) {
                    if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &msg, &st)) {
                        handleIncomingMessage(&msg); // Передача парсеного пакета на аналіз телеметрії
                    }
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    });
}

// ПОРЦІЯ АРХІТЕКТУРИ: Потік №2 — Telemetry & Heartbeat Loop (Високорівнева логіка синхронізації)
void SITLMavlinkVehicle::startTelemetryLoop() {
    telemetry_thread = std::thread([this]() {
        // Активуємо потік ESTIMATOR_STATUS через Message Interval command
        mavlink_message_t msg;
        mavlink_msg_command_long_pack(MavlinkConfig::SYS_ID, MavlinkConfig::COMP_ID, &msg, 
            MavlinkConfig::TGT_SYS, MavlinkConfig::TGT_COMP, 
            511, // MAV_CMD_SET_MESSAGE_INTERVAL
            0, 
            33.0f, // ID 33 = GLOBAL_POSITION_INT
            200000.0f, // 200 000 мкс = 5 Гц
            0, 0, 0, 0, 0);
        sendMavlinkMessage(&msg);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

         mavlink_msg_command_long_pack(MavlinkConfig::SYS_ID, MavlinkConfig::COMP_ID, &msg, 
            MavlinkConfig::TGT_SYS, MavlinkConfig::TGT_COMP, 
            511, // MAV_CMD_SET_MESSAGE_INTERVAL
            0, 
            32.0f, // ID 32 = SYS_STATUS
            200000.0f, // 200 000 мкс = 5 Гц
            0, 0, 0, 0, 0);
        sendMavlinkMessage(&msg);

       mavlink_msg_command_long_pack(MavlinkConfig::SYS_ID, MavlinkConfig::COMP_ID, &msg, 
            MavlinkConfig::TGT_SYS, MavlinkConfig::TGT_COMP, 
            511,       // MAV_CMD_SET_MESSAGE_INTERVAL
            0,
            87.0f,     // ID 87 = POSITION_TARGET_GLOBAL_INT
            200000.0f, // 200 000 мкс = 5 Гц
            0, 0, 0, 0, 0
        );
        sendMavlinkMessage(&msg);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        while (is_running) {
            // Генерація Heartbeat (Обов'язкова умова трансляції для SITL)
            mavlink_msg_heartbeat_pack(MavlinkConfig::SYS_ID, MavlinkConfig::COMP_ID, &msg, 
                MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, 0, 0, MAV_STATE_ACTIVE);
            sendMavlinkMessage(&msg);
            
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    });
}

void SITLMavlinkVehicle::handleIncomingMessage(const mavlink_message_t* m) {
     if (m->msgid == MAVLINK_MSG_ID_HEARTBEAT) {
        last_heartbeat[m->compid] = getTimestampMs();
        if (m->compid == MavlinkConfig::TGT_COMP) {
            mavlink_heartbeat_t hb;
            mavlink_msg_heartbeat_decode(m, &hb);
            current_mode = hb.custom_mode;

            //ekf_healthy = true; // Простий індикатор здоров'я EKF3
        }
    }

    // Временный отладчик: выводим все пакеты НЕ от автопілота (sysid != 1) или команды навигации
    else if (m->msgid == MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT) {
        mavlink_position_target_global_int_t target;
        mavlink_msg_position_target_global_int_decode(m, &target);

        if (std::abs(target.lat_int) > 10000000 && std::abs(target.lon_int) > 10000000) {
            dynamic_target_lat = target.lat_int;
            dynamic_target_lon = target.lon_int;
            dynamic_target_alt = target.alt;
            new_target_available = true;
        }
    }

    else if (m->msgid == MAVLINK_MSG_ID_ATTITUDE) {
        mavlink_attitude_t att;
        mavlink_msg_attitude_decode(m, &att);
        current_roll  = att.roll;
        current_pitch = att.pitch;
        current_yaw   = att.yaw;
    }
    else if (m->msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT) {
        mavlink_global_position_int_t pos;
        mavlink_msg_global_position_int_decode(m, &pos);
        current_lat = pos.lat; // В мікродегр. (1e-7)
        current_lon = pos.lon; // В мікродегр. (1e-7)
        current_alt = pos.relative_alt /1000.0f; // В мм

        // Обчислюємо реальну швидкість відносно землі в м/с
        float vx_ms = pos.vx / 100.0f;
        float vy_ms = pos.vy / 100.0f;
        current_ground_speed = std::sqrt(vx_ms * vx_ms + vy_ms * vy_ms);
    }
    // Миттєве перехоплення точки кліку "Go here" від QGroundControl (SYSID == 255)
    else if (m->sysid == 255 && m->msgid == MAVLINK_MSG_ID_MISSION_ITEM_INT) {
        mavlink_mission_item_int_t item;
        mavlink_msg_mission_item_int_decode(m, &item);
        if (item.x != 0 && item.y != 0) {
            dynamic_target_lat = item.x;
            dynamic_target_lon = item.y;
            dynamic_target_alt = item.z;
            new_target_available = true;
            std::cout << "\n\033[1;36m[QGC_INTERCEPT] Перехоплено точку Go Here: Lat=" 
                      << (item.x / 10000000.0) << " Lon=" << (item.y / 10000000.0) << "\033[0m" << std::endl;
        }
    }
    else if (m->sysid == 255 && m->msgid == MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT) {
        mavlink_set_position_target_global_int_t target;
        mavlink_msg_set_position_target_global_int_decode(m, &target);
        if (target.lat_int != 0 && target.lon_int != 0) {
            dynamic_target_lat = target.lat_int;
            dynamic_target_lon = target.lon_int;
            dynamic_target_alt = target.alt;
            new_target_available = true;
            std::cout << "\n\033[1;36m[QGC_INTERCEPT] Перехоплено ціль SetPosition: Lat=" 
                      << (target.lat_int / 10000000.0) << " Lon=" << (target.lon_int / 10000000.0) << "\033[0m" << std::endl;
        }
    }

    //else if (m->msgid == MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT) {
    //    mavlink_position_target_global_int_t target;
    //    mavlink_msg_position_target_global_int_decode(m, &target);
        
    //    // Ігноруємо нульові координати (коли ціль ще не задана)
    //    if (target.lat_int != 0 && target.lon_int != 0) {
    //        // Перевіряємо, чи змінилася ціль порівняно з попередньою
    //        if (target.lat_int != dynamic_target_lat.load() || target.lon_int != dynamic_target_lon.load()) {
    //            dynamic_target_lat = target.lat_int;
    //            dynamic_target_lon = target.lon_int;
    //            dynamic_target_alt = target.alt;
    //            new_target_available = true;
    //            std::cout << "[TELEMETRY] Отримано нову ціль: Lat=" << (target.lat_int / 10000000.0)
    //                      << " Lon=" << (target.lon_int / 10000000.0) << std::endl;
     //       }
    //    }
    //}
    else if (m->msgid == MAVLINK_MSG_ID_COMMAND_ACK) {
        mavlink_command_ack_t ack;
        mavlink_msg_command_ack_decode(m, &ack);
        if (ack.command == awaiting_cmd) {
            if (ack.result == MAV_RESULT_ACCEPTED) cmd_accepted = true;
        }
    }
    else if (m->msgid == MAVLINK_MSG_ID_VFR_HUD) {
        mavlink_vfr_hud_t hud;
        mavlink_msg_vfr_hud_decode(m, &hud);
        current_alt = hud.alt; // Оновлюємо атомарну змінну висоти
        current_ground_speed = hud.groundspeed; // Оновлюємо атомарну змінну швидкості
    }
    else if (m->msgid == MAVLINK_MSG_ID_LOCAL_POSITION_NED) {
        mavlink_local_position_ned_t pos;
        mavlink_msg_local_position_ned_decode(m, &pos);
        local_pos_x = pos.x; // Відстань на Північ (North)
        local_pos_y = pos.y; // Відстань на Схід (East)

        current_alt = -pos.z; // Висота (Down) в протилежному напрямку
    }
    else if (m->msgid == MAVLINK_MSG_ID_ESTIMATOR_STATUS) {
        mavlink_estimator_status_t ekf;
        mavlink_msg_estimator_status_decode(m, &ekf);

        uint16_t f = ekf.flags;
        bool attitude      = (f & 1);   // Біт 1: Орієнтація
        bool horiz_vel     = (f & 2);   // Біт 2: Горизонтальна швидкість
        bool horiz_pos_abs = (f & 16);  // Біт 16: GPS фіксація (Критично для GUIDED)

        if (attitude && horiz_vel && horiz_pos_abs) {
            ekf_healthy = true;
        } else {
            ekf_healthy = false;
        }
    }
    else if (m->msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT) {
        mavlink_global_position_int_t gps_pos;
        mavlink_msg_global_position_int_decode(m, &gps_pos);
        current_lat = gps_pos.lat; // В мікродегр. (1e-7)
        current_lon = gps_pos.lon; // В мікродегр. (1e-7)
    }
    else if (m->msgid == MAVLINK_MSG_ID_STATUSTEXT) {
        mavlink_statustext_t status;
        mavlink_msg_statustext_decode(m, &status);
        std::string log_msg(status.text);
        std::cout << "[SITL_LOG] " << status.text << std::endl;

        if (log_msg.find("using GPS") != std::string::npos ||
            log_msg.find("yaw alignment complete") != std::string::npos ||
            log_msg.find("EKF3 IMU0 ready") != std::string::npos) {
            std::cout << "\n\033[1;32m[SYSTEM_ARCH] ТЕКСТОВИЙ СИГНАЛ ГОТОВНОСТІ EKF3 ОТРИМАНО!\033[0m" << std::endl;   
            ekf_healthy = true;
        }
    }
}

bool SITLMavlinkVehicle::setParameter(const char* param_id, float value) {
    mavlink_message_t msg;
    char id_buf[16] = {0};
    strncpy(id_buf, param_id, 16);

    mavlink_msg_param_set_pack(
        MavlinkConfig::SYS_ID, MavlinkConfig::COMP_ID, &msg,
        MavlinkConfig::TGT_SYS, MavlinkConfig::TGT_COMP,
        id_buf, value, MAV_PARAM_TYPE_REAL32
    );
    sendMavlinkMessage(&msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    return true;
}

bool SITLMavlinkVehicle::waitForAck(uint16_t cmd, uint32_t timeout_ms) {
    awaiting_cmd = cmd;
    cmd_accepted = false;
    uint64_t t0 = getTimestampMs();
    while (getTimestampMs() - t0 < timeout_ms) {
        if (cmd_accepted) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return false;
}

bool SITLMavlinkVehicle::setMode(uint32_t mode) {
    mavlink_message_t msg;
    mavlink_msg_set_mode_pack(MavlinkConfig::SYS_ID, MavlinkConfig::COMP_ID, &msg, MavlinkConfig::TGT_SYS, 1, mode);
    
    for(int i=0; i<3; ++i) { // Захист лінії SITL від пропуску кадрів
        sendMavlinkMessage(&msg);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    uint64_t t0 = getTimestampMs();
    while (getTimestampMs() - t0 < 3000) {
        if (current_mode == mode) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return false;
}

bool SITLMavlinkVehicle::armMotors() {
    mavlink_message_t msg;
    mavlink_msg_command_long_pack(MavlinkConfig::SYS_ID, MavlinkConfig::COMP_ID, &msg, 
        MavlinkConfig::TGT_SYS, MavlinkConfig::TGT_COMP, 400, 0, 1.0f, 0, 0, 0, 0, 0, 0);
    sendMavlinkMessage(&msg);
    return waitForAck(400);
}

bool SITLMavlinkVehicle::takeoff(float altitude) {
    mavlink_message_t msg;
    mavlink_msg_command_long_pack(MavlinkConfig::SYS_ID, MavlinkConfig::COMP_ID, &msg, 
        MavlinkConfig::TGT_SYS, MavlinkConfig::TGT_COMP, 22, 0, 0, 0, 0, 0, 0, 0, altitude);
    sendMavlinkMessage(&msg);
    return waitForAck(22);
}

bool SITLMavlinkVehicle::isTargetReached(float target_n, float target_e, float tolerance) {
    // Рахуємо дельту між ціллю та поточною координатою дрона
    float delta_n = target_n - local_pos_x.load();
    float delta_e = target_e - local_pos_y.load();
    // Гіпотенуза (відстань за прямою лінією)
    float distance = std::sqrt(delta_n * delta_n + delta_e * delta_e);
    return (distance <= tolerance);
}

bool SITLMavlinkVehicle::moveToLocalOffset(float north, float east, float down) {
    mavlink_message_t msg;
    mavlink_msg_set_position_target_local_ned_pack(MavlinkConfig::SYS_ID, MavlinkConfig::COMP_ID, &msg,
        getTimestampMs(), MavlinkConfig::TGT_SYS, MavlinkConfig::TGT_COMP,
        MAV_FRAME_LOCAL_NED,
        0b0000111111111000,
        north, east, down,
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    sendMavlinkMessage(&msg);
    return true;
}

// НАДСИЛАННЯ КОМАНДИ РУХУ ЗА GPS КООРДИНАТАМИ
bool SITLMavlinkVehicle::moveToGlobalWithAccel(int32_t lat, int32_t lon, float alt_relative, float max_speed, float accel) {
    mavlink_message_t msg;

    // Зміна динаміки швидкості автопілота
     mavlink_msg_set_position_target_global_int_pack(
        MavlinkConfig::SYS_ID, MavlinkConfig::COMP_ID, &msg,
        getTimestampMs(),
        MavlinkConfig::TGT_SYS, MavlinkConfig::TGT_COMP,
        MAV_FRAME_GLOBAL_RELATIVE_ALT,       // Робота в глобальній системі
        0b0000111111111000,         // Маска: керуємо тільки Lat/Lon/Alt
        lat, lon, alt_relative,
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f
    );
    sendMavlinkMessage(&msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    mavlink_msg_command_long_pack(MavlinkConfig::SYS_ID, MavlinkConfig::COMP_ID, &msg,
        MavlinkConfig::TGT_SYS, MavlinkConfig::TGT_COMP,
        178, 0, 1.0f, max_speed, accel, 0, 0, 0, 0);
    sendMavlinkMessage(&msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
        
    return true;
}

bool SITLMavlinkVehicle::patrolRoute(const std::vector<MavlinkConfig::GPS_Point>& route, float alt_relative, float speed_mps, int lap_count) {
    if (route.empty()) return false;

    std::cout << "\n[ROUTE_ENGINE] Запуск патрулювання маршруту. Точок: " << route.size() 
              << " | Кількість обходів: " << lap_count << std::endl;

    // Зовнішній цикл відповідає за кількість повних обходів (кіл маршруту)
    for (int lap = 1; lap <= lap_count; ++lap) {
        std::cout << "\n\033[1;35m--- ЗАПУСК КОЛА МАРШРУТУ № " << lap << " з " << lap_count << " ---\033[0m" << std::endl;

        // Внутрішній цикл іде строго по черзі по кожній точці масиву
        for (size_t i = 0; i < route.size(); ++i) {
            int32_t t_lat = route[i].lat;
            int32_t t_lon = route[i].lon;

            std::cout << "[ROUTE] Політ до точки " << (i + 1) << "/" << route.size() 
                      << " (Lat: " << (t_lat / 10000000.0) << " | Lon: " << (t_lon / 10000000.0) << ") зі швидкістю " << speed_mps << " м/с" << std::endl;

            // Відправляємо дрон на поточну точку маршруту (прискорення ставимо стандартне 3.5)
            moveToGlobalWithAccel(t_lat, t_lon, alt_relative, speed_mps, 5.0f);

            // 1. Чекаємо, поки залишиться 100 метрів до цієї точки
            while (!isGlobalTargetReached(t_lat, t_lon, 100.0f)) {
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
            std::cout << "[ROUTE] Залишилося 100м до точки " << (i + 1) << ". Контролер ArduPilot починає гальмування..." << std::endl;

            // 2. Чекаємо фінального прильоту в точку (допуск 4 метри)
            while (!isGlobalTargetReached(t_lat, t_lon, 4.0f)) {
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            }
            std::cout << "\033[1;32m[SUCCESS] Точка " << (i + 1) << " успішно пройдена!\033[0m Фіксація 1 сек." << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }

    std::cout << "\n[ROUTE_ENGINE] Патрулювання всіх " << lap_count << " кіл маршруту повністю завершено!" << std::endl;
    return true;
}

//bool SITLMavlinkVehicle::patrolCircle(int32_t center_lat, int32_t center_lon, float alt_rel, float radius_m, float speed_mps) {
    // Рахуємо кутову швидкість (рад/сек): w = v / R
//    double angular_velocity = speed_mps / radius_m;
    
    // Вираховуємо поточний кут на основі системного часу Малини
//    double time_sec = getTimestampMs() / 1000.0;
//    double angle = angular_velocity * time_sec;

    // Константи переведення метрів у градуси на сфері Землі
//    constexpr double R_EARTH = 6371000.0;
//    double lat_center_deg = center_lat / 10000000.0;

    // Зміщення в метрах по осях North та East
//    double offset_north = radius_m * std::cos(angle);
//    double offset_east  = radius_m * std::sin(angle);

    // Перераховуємо зміщення метрів у GPS координати (широту/долготу)
//    int32_t target_lat = center_lat + static_cast<int32_t>((offset_north / R_EARTH) * (180.0 / M_PI) * 10000000.0);
//    int32_t target_lon = center_lon + static_cast<int32_t>((offset_east / (R_EARTH * std::cos(lat_center_deg * M_PI / 180.0))) * (180.0 / M_PI) * 10000000.0);

    // Відправляємо дрон на поточну точку орбіти кола
//    mavlink_message_t msg;
//    mavlink_msg_set_position_target_global_int_pack(
//        MavlinkConfig::SYS_ID, MavlinkConfig::COMP_ID, &msg,
//        getTimestampMs(), MavlinkConfig::TGT_SYS, MavlinkConfig::TGT_COMP,
//        MAV_FRAME_GLOBAL_RELATIVE_ALT,
//        0b0000111111111000,
//        target_lat, target_lon, alt_rel,
//        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f
//    );
//    sendMavlinkMessage(&msg);
//    return true;
//}

// РОЗРАХУНОК ВІДСТАНІ ДО GPS ЦІЛІ НА СФЕРІ (МЕТОД СЕРЕДНЬОЇ ШИРОТИ)
bool SITLMavlinkVehicle::isGlobalTargetReached(int32_t target_lat, int32_t target_lon, float tolerance_m) {
    int32_t c_lat = current_lat.load();
    int32_t c_lon = current_lon.load();
    // ЗАХИСТ ВІД НУЛЬОВИХ ДАНИХ: Якщо телеметрія ще не прийшла, ми точно НЕ досягли цілі
    if (c_lat == 0 || c_lon == 0) return false; // Якщо координати ще не отримані

    double lat1 = (c_lat / 10000000.0) * M_PI / 180.0;
    double lon1 = (c_lon / 10000000.0) * M_PI / 180.0;
    double lat2 = (target_lat / 10000000.0) * M_PI / 180.0;
    double lon2 = (target_lon / 10000000.0) * M_PI / 180.0;

    constexpr double R = 6371000.0; // Радіус Землі в метрах
    double dlat = lat2 - lat1;
    double dlon = lon2 - lon1;

    double x = dlon * std::cos((lat1 + lat2) / 2.0);
    double y = dlat;
    double distance_meters = std::sqrt(x * x + y * y) * R;

    std::cout << "[DEBUG_GPS] Відстань до цілі: " << distance_meters << " м" << std::endl;

    return (distance_meters <= tolerance_m);
}

bool SITLMavlinkVehicle::flyTowardsGlobalFast(int32_t target_lat, int32_t target_lon, float alt_relative, float speed_mps) {
    double c_lat = (current_lat.load() / 10000000.0) * M_PI / 180.0;
    double c_lon = (current_lon.load() / 10000000.0) * M_PI / 180.0;
    double t_lat = (target_lat / 10000000.0) * M_PI / 180.0;
    double t_lon = (target_lon / 10000000.0) * M_PI / 180.0;

    // Вектор напрямку на Північ (North) та Схід (East)
    double dn = (t_lat - c_lat);
    double de = (t_lon - c_lon) * std::cos((c_lat + t_lat) / 2.0);
    double norm = std::sqrt(dn * dn + de * de);
    if (norm < 1e-9) return true;

    float vx = static_cast<float>((dn / norm) * speed_mps); // Швидкість на Північ (м/с)
    float vy = static_cast<float>((de / norm) * speed_mps); // Швидкість на Схід (м/с)

    mavlink_message_t msg;
    mavlink_msg_set_position_target_global_int_pack(
        MavlinkConfig::SYS_ID, MavlinkConfig::COMP_ID, &msg,
        getTimestampMs(),
        MavlinkConfig::TGT_SYS, MavlinkConfig::TGT_COMP,
        MAV_FRAME_GLOBAL_RELATIVE_ALT,
        0b0000111111000111, // Маска: керуємо швидкістю (vx, vy) + висотою (alt)
        0, 0, alt_relative,
        vx, vy, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f
    );
    sendMavlinkMessage(&msg);
    return true;
}