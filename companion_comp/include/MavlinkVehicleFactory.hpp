#pragma once
#include "IMavlinkVehicle.hpp"
#include "SITLMavlinkVehicle.hpp"
#include <memory>


class MavlinkVehicleFactory {
public:
    enum class TargetType {
        SITL_DOCKER,
        REAL_HARDWARE // На майбутнє для розширення архітектури
    };

    static std::unique_ptr<IMavlinkVehicle> createVehicle(TargetType type) {
        switch (type) {
            case TargetType::SITL_DOCKER:
                return std::make_unique<SITLMavlinkVehicle>();
            default:
                return nullptr;
        }
    }
};