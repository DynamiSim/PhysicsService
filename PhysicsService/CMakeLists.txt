cmake_minimum_required(VERSION 3.16)
project(PhysicsService LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wno-deprecated-declarations)
endif()

find_package(pinocchio REQUIRED)
find_package(websocketpp REQUIRED)
find_package(jsoncpp REQUIRED)

include(FindProtobuf)
find_package(Protobuf REQUIRED)
include_directories(${PROTOBUF_INCLUDE_DIR})


find_path(ZeroMQ_INCLUDE_DIR
        NAMES zmq.hpp
        PATHS ${PC_ZeroMQ_INCLUDE_DIRS}
        )

find_library(ZeroMQ_LIBRARY
        NAMES zmq
        PATHS ${PC_ZeroMQ_LIBRARY_DIRS}
        )

# Generate the protobuf stub files during compilation
add_library(proto-objects OBJECT "${CMAKE_CURRENT_LIST_DIR}/protos/motion_control.proto")
target_link_libraries(proto-objects PUBLIC protobuf::libprotobuf)
set(PROTO_BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}")

target_include_directories(proto-objects PUBLIC "$<BUILD_INTERFACE:${PROTO_BINARY_DIR}>")

protobuf_generate(
    TARGET proto-objects
    IMPORT_DIRS "${CMAKE_CURRENT_LIST_DIR}/protos"
    PROTOC_OUT_DIR "${PROTO_BINARY_DIR}")

add_executable(physics_service main.cpp WebsocketServer.cpp ${PROTO_BINARY_DIR}/protos/motion_control.pb.cc)
# Add dependency on proto-objects
add_dependencies(physics_service proto-objects)

target_include_directories(physics_service PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${PROTO_BINARY_DIR})

target_link_libraries(physics_service PRIVATE pinocchio::pinocchio websocketpp::websocketpp pthread jsoncpp_lib protobuf::libprotobuf zmq)
