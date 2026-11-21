#pragma once
#include "Types.h"

class IDroneAutopilot {
public:
    virtual ~IDroneAutopilot() = default;
    
    // Запуск та зупинка потоків
    virtual void start(SharedData& shared) = 0;
    virtual void stop() = 0;
};