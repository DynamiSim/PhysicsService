cmake_minimum_required(VERSION 3.10)

project(PhysicsService LANGUAGES CXX)

find_package(pinocchio REQUIRED)
find_package(websocketpp REQUIRED)
find_package(jsoncpp REQUIRED)

add_executable(physics_service main.cpp WebsocketServer.cpp)

target_include_directories(physics_service PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(physics_service PRIVATE pinocchio::pinocchio websocketpp::websocketpp pthread jsoncpp_lib)
