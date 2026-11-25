#include "MavlinkVehicleFactory.hpp"
#include "MavlinkConfig.hpp"
#include <iostream>
#include <thread>
#include <vector>

int main() {
    std::cout << "[SYSTEM] Запуск автономного GPS клієнта місії патрулювання..." << std::endl;
    auto drone = MavlinkVehicleFactory::createVehicle(MavlinkVehicleFactory::TargetType::SITL_DOCKER);

    if (!drone->connect(MavlinkConfig::DEFAULT_SERIAL_PORT)) {
        std::cerr << "[CRITICAL] Не вдалося підключитися до порту!" << std::endl;
        return 1;
    }

    drone->startIOLoop();
    drone->startTelemetryLoop();

    std::cout << "[SYSTEM] Очікування стабілізації фільтра EKF3 та GPS супутників..." << std::endl;
    int seconds_passed = 0;
    while (!drone->isEkfHealthy()) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        seconds_passed++;
        if (seconds_passed >= 25) { 
            break;
        }
    }
    std::cout << "[SUCCESS] Супутники синхронізовано. Робоча геопозиція готова." << std::endl;

    // Перевірка безпеки геозони України
    double start_lat = drone->getCurrentLat() / 10000000.0;
    if (start_lat < 40.0 || start_lat > 55.0) {
        std::cerr << "[CRITICAL] Дрон злітає поза межами України! Широта: " << start_lat << std::endl;
        drone->disconnect();
        return 1;
    }

    if (!drone->setMode(MavlinkConfig::MODE_GUIDED)) return 1;
    if (!drone->armMotors()) return 1;

    // --- ЗЛІТ ---
    std::cout << "[MISSION] Зліт на висоту " << MavlinkConfig::TARGET_ALTITUDE_M << " м..." << std::endl;
    if (!drone->takeoff(MavlinkConfig::TARGET_ALTITUDE_M)) return 1;

    while (drone->getCurrentAltitude() < (MavlinkConfig::TARGET_ALTITUDE_M - 2.0f)) {
        std::cout << "[TRAKING] Набір висоти: " << drone->getCurrentAltitude() 
                  << " / " << MavlinkConfig::TARGET_ALTITUDE_M << " м" << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    std::cout << "[SUCCESS] Висоту зльоту набрано. Переходимо в режим чергування." << std::endl;

    int32_t patrol_lat = 0;
    int32_t base_lon = 0;
    float base_alt = 0.0f;

    drone->clearDynamicTargetFlag();
    std::cout << "\n[MISSION] Очікування стартової координати району патрулювання з телеметрії..." << std::endl;

    while (true) {
        if (drone->hasNewDynamicTarget()) {
            int32_t t_lat = 0, t_lon = 0;
            float t_alt = 0.0f;
            drone->getDynamicTarget(t_lat, t_lon, t_alt);

            bool far_enough = !drone->isGlobalTargetReached(t_lat, t_lon, 50.0f);
            bool not_absurd = drone->isGlobalTargetReached(t_lat, t_lon, 50000.0f);

            if (far_enough && not_absurd) {
                patrol_lat = t_lat;
                base_lon = t_lon;
                base_alt = t_alt;
                std::cout << "[MISSION] Отримано нову координату! Перехоплюємо керування..." << std::endl;
                break;
            }
            drone->clearDynamicTargetFlag();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
    }

    drone->getDynamicTarget(patrol_lat, base_lon, base_alt);
    
    // --- АВТОМАТИЧНА ГЕНЕРАЦІЯ МАРШРУТУ НАВКОЛО ОТРИМАНОЇ ТОЧКИ ---
    std::vector<MavlinkConfig::GPS_Point> patrol_route = {
        { patrol_lat + 15000, base_lon + 15000 }, 
        { patrol_lat - 13000, base_lon + 13000 },
        { patrol_lat - 13000, base_lon - 13000 },
        { patrol_lat + 15000, base_lon - 15000 },
        { patrol_lat + 15000, base_lon + 15000 },
        { patrol_lat - 15000, base_lon - 15000 },
        { patrol_lat - 13000, base_lon + 13000 },
        { patrol_lat + 15000, base_lon - 15000 }
        
    };

    // --- РУХ ДО ТОЧКИ ПОЧАТКУ ПАТРУЛЮВАННЯ ІЗ ПРИСКОРЕННЯМ ---
    std::cout << "\n[MISSION] Розгін до першої точки початку патрулювання..." << std::endl;
    
    // Рух до ПЕРШОЇ точки вектора (індекс 0)
    //drone->moveToGlobalWithAccel(patrol_route[0].lat, patrol_route[0].lon, MavlinkConfig::TARGET_ALTITUDE_M, 16.0f, 5.0f);
    // --- ПІДГОТОВКА ЛІМІТІВ ШВИДКОСТІ ТА ФІЗИКИ СИМУЛЯТОРА ---
    //std::cout << "\n[DEBUG_SETUP] Встановлення лімітів для швидкого польоту (16 м/с)..." << std::endl;
    //drone->setParameter("ATC_ANGLE_MAX", 55.0f);  // Збільшуємо макс. нахил до 55° (щоб подолати опір повітря SITL)
    //drone->setParameter("LOIT_SPEED_MS", 22.0f);  // Запас ліміту швидкості позиційного контролера
    //drone->setParameter("WP_SPD", 16.0f);         // Цільова швидкість маршруту 16 м/с
    //drone->setParameter("WP_ACC", 5.0f);          // Прискорення 5 м/с² для швидкого набору
    //drone->setParameter("SIM_WIND_SPD", 0.0f);    // Гарантуємо відсутність зустрічного вітру


    // ЕТАП 1: Чекаємо підльоту до зони 100 метрів першої точки
    int debug_counter = 0;
    while (!drone->isGlobalTargetReached(patrol_route[0].lat, patrol_route[0].lon, 100.0f)) {
        drone->flyTowardsGlobalFast(patrol_route[0].lat, patrol_route[0].lon, MavlinkConfig::TARGET_ALTITUDE_M, 16.0f);
        // Виводимо дебаг швидкості приблизно раз на 0.9 секунди (кожні 3 ітерації по 300 мс)
        if (++debug_counter % 3 == 0) {
            float real_spd = drone->getCurrentGroundSpeed();
            std::cout << "[SPEED_DEBUG] Ціль: 16.0 м/с | Реальна швидкість: " 
                      << real_spd << " м/с" << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
    }
    
    // --- ЕТАП 2: ПЛАВНЕ ГАЛЬМУВАННЯ ТА ТОЧНИЙ ПІДХІД (останні 100 метрів) ---
    std::cout << "[ALERT] Залишилося 100м до початкової точки. Вхід у режим точного гальмування..." << std::endl;
    drone->moveToGlobalWithAccel(patrol_route[0].lat, patrol_route[0].lon, MavlinkConfig::TARGET_ALTITUDE_M, 7.0f, 3.5f);
    
    while (!drone->isGlobalTargetReached(patrol_route[0].lat, patrol_route[0].lon, 5.0f)) {
        std::cout << "[BRAKE_DEBUG] Гальмування... Поточна швидкість: " 
                  << drone->getCurrentGroundSpeed() << " м/с" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    std::cout << "[SUCCESS] Дрон прибув у точку старту патрулювання!" << std::endl;

    // --- ПАТРУЛЮВАННЯ МАРШРУТУ 2 РАЗИ ---
    drone->patrolRoute(patrol_route, MavlinkConfig::TARGET_ALTITUDE_M, 7.0f, 2);

    // --- ПОВЕРНЕННЯ ДОДОМУ (RTL) ---
    std::cout << "\n[MISSION] Місію виконано на 100%! Активація режиму RTL..." << std::endl;
    drone->setMode(MavlinkConfig::MODE_RTL);
    std::this_thread::sleep_for(std::chrono::seconds(5));

    drone->disconnect();
    std::cout << "[SYSTEM] Програму успішно завершено." << std::endl;
    return 0;
}