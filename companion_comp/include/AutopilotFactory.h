#pragma once
#include "DroneAutopilotImpl.h"
#include <memory>
#include <iostream>

class AutopilotFactory {
public:
    enum class AutopilotType {
        SITL_VIRTUAL,
        HARDWARE_UART
    };

    static std::unique_ptr<IDroneAutopilot> createAutopilot(AutopilotType type) {
        switch (type) {
            case AutopilotType::SITL_VIRTUAL:
                std::cout << "[FACTORY] Створено віртуальний емулятор автопілота для Devcontainer.\n";
                return std::make_unique<DroneAutopilotImpl>();
            case AutopilotType::HARDWARE_UART:
                std::cout << "[FACTORY] Створено апаратний автопілот для Raspberry Pi 5.\n";
                return std::make_unique<DroneAutopilotImpl>();
        }
        return nullptr;
    }
};