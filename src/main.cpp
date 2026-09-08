#include "../include/AutopilotFactory.h"
#include <iostream>
#include <chrono>
#include <thread>

int main() {
    // 1. Створюємо структуру спільної пам'яті для обміну між потоками
    SharedData shared_resources;

    // 2. Використовуємо паттерн Фабрика для створення об'єкта автопілота
    std::unique_ptr<IDroneAutopilot> autopilot = 
        AutopilotFactory::createAutopilot(AutopilotFactory::AutopilotType::SITL_VIRTUAL);

    if (!autopilot) {
        std::cerr << "[SYSTEM] [ERROR] Не вдалося створити об'єкт автопілота через фабрику.\n";
        return 1;
    }

    // 3. Запускаємо багатопотокову систему (потоки IO_THREAD та MissionProcessor)
    autopilot->start(shared_resources);

    std::cout << "\n=======================================================\n";
    std::cout << "=== СИСТЕМУ АВТОПІЛОТА УСПІШНО ЗАПУЩЕНО В DEVCONTAINER ===\n";
    std::cout << "=======================================================\n\n";

    // Головний керуючий цикл місії (імітація зовнішніх подій у часі)
    for (int second = 1; second <= 15; ++second) {
        std::this_thread::sleep_for(std::chrono::seconds(1));

        // На 4-й секунді польоту система комп'ютерного зору (ШІ-камера) фіксує ціль
        if (second == 4) {
            std::lock_guard<std::mutex> lock(shared_resources.mtx);
            shared_resources.target_pos = {15.0f, 18.0f}; // Задаємо координати цілі Coord
            shared_resources.target_found = true;
            std::cout << "\n[🎯 MAIN_EVENT] ШІ-камера виявила об'єкт! Координати передано в MissionProcessor.\n\n";
        }

        // На 11-й секунді імітуємо обрив зв'язку з QGroundControl для перевірки FAILSAFE
        if (second == 11) {
            std::cout << "\n[⚠️ MAIN_EVENT] ІМІТАЦІЯ ВТРАТИ СИГНАЛУ З НАЗЕМНОЇ СТАНЦІЇ КЕРУВАННЯ!\n\n";
            // Потік runIO зазвичай робить це автоматично по таймауту, 
            // але ми виставляємо прапорець примусово для ізольованого тесту логіки
            shared_resources.link_lost = true;
        }
    }

     // 4. Коли цикл тесту завершено, зупиняємо роботу всієї системи
    std::cout << "\n[SYSTEM] Завершення тесту місії. Зупинка паралельних потоків...\n";
    shared_resources.is_running = false;
    
    // Блокуємо головний потік, поки IO та MissionProcessor безпечно завершують свої цикли
    autopilot->stop();

    std::cout << "[SYSTEM] Програму успішно та безпечно завершено.\n";
    return 0;
}
// Фінальна версія для заліку