cmake_minimum_required(VERSION 3.16)

project(takeoff_sitl_project LANGUAGES CXX)

# MAVSDK v4 вимагає стандарт C++17 або новіший
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Пошук встановленої бібліотеки MAVSDK
find_package(Threads  REQUIRED)

set(SOURCES
    src/main.cpp
    src/SITLMavlinkVehicle.cpp
    )

# Додавання нашого виконуваного файлу
add_executable(takeoff_sitl ${SOURCES})

# Лінкування з таргетом MAVSDK v4
target_link_libraries(takeoff_sitl PRIVATE 
    Threads::Threads
)

target_compile_definitions(takeoff_sitl PRIVATE MAVSDK_ENABLE_MAVLINK_C_API)

# Вказуємо CMake заглянути всередину локальної папки MAVSDK
target_include_directories(takeoff_sitl PRIVATE 
    "/home/mult/T1/MAVSDK/cpp/build/third_party/install/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
)
