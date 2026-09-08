cmake_minimum_required(VERSION 3.15)
project(MavLink_Autopilot LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ПРИБИРАЄМО СМІТТЯ З ТЕРМІНАЛУ: Кажемо компілятору ігнорувати варнінги упакованих структур MAVLink
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-address-of-packed-member")

find_package(Threads REQUIRED)

set(SOURCES
    src/main.cpp
    src/MissionProcessor.cpp
    src/DroneStateMachine.cpp
    src/DroneAutopilotImpl.cpp
)

add_executable(autopilot_node ${SOURCES})
target_link_libraries(autopilot_node PRIVATE Threads::Threads)
target_include_directories(autopilot_node PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/mavlink/include
        
)    
 