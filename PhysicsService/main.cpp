#include "WebsocketServer.h"

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/rnea.hpp"

#include <iostream>
#include <thread>
#include <asio/io_service.hpp>
#include <chrono>
#include <csignal>
#include <fstream>
#include <json/json.h>
#include <sstream>

// PINOCCHIO_MODEL_DIR is defined by the CMake but you can define your own directory here.
#ifndef PINOCCHIO_MODEL_DIR
  #define PINOCCHIO_MODEL_DIR "/workspaces/PhysicsService/robots"
#endif

// The port number the WebSocket server listens on
#define PORT_NUMBER 1234

enum SimulationState { STOPPED, RUNNING, RESET };

// Create the event loop for the main thread, and the WebSocket server
asio::io_service mainEventLoop;
WebsocketServer server;
SimulationState simState = STOPPED;

using namespace pinocchio;

int main(int argc, char* argv[])
{	
	#pragma region WebSocket callbacks

	// Register our network callbacks, ensuring the logic is run on the main thread's event loop
	server.connect([&mainEventLoop, &server](ClientConnection conn)
	{
		mainEventLoop.post([conn, &server]()
		{
			std::clog << "Connection opened." << std::endl;
			std::clog << "There are now " << server.numConnections() << " open connections." << std::endl;
			
			//Send a hello message to the client
			server.sendMessage(conn, "hello", Json::Value());
		});
	});
	server.disconnect([&mainEventLoop, &server](ClientConnection conn)
	{
		mainEventLoop.post([conn, &server]()
		{
			std::clog << "Connection closed." << std::endl;
			std::clog << "There are now " << server.numConnections() << " open connections." << std::endl;
		});
	});
	server.message("message", [&mainEventLoop, &server](ClientConnection conn, const Json::Value& args)
	{
		mainEventLoop.post([conn, args, &server]()
		{
			std::clog << "Message payload:" << std::endl;
			for (auto key : args.getMemberNames()) {
				std::clog << "\t" << key << ": " << args[key].asString() << std::endl;
			}
			
			// Echo the message pack to the client
			server.sendMessage(conn, "message", args);
		});
	});

	// Message handler for starting the simulation
	server.message("start", [&mainEventLoop, &server, &simState](ClientConnection conn, const Json::Value& args)
	{
		mainEventLoop.post([&server, &simState]()
		{
			// Start the simulation if it's not already running
			if (simState != RUNNING)
			{
				simState = RUNNING;
				std::cout << "Simulation started." << std::endl;
			}
		});
	});

	// Message handler for stopping the simulation
	server.message("stop", [&mainEventLoop, &server, &simState](ClientConnection conn, const Json::Value& args)
	{
		mainEventLoop.post([&server, &simState]()
		{
			// Stop the simulation if it's running
			if (simState != STOPPED)
			{
				simState = STOPPED;
				std::cout << "Simulation stopped." << std::endl;
			}
		});
	});

	// Message handler for resetting the simulation
	server.message("reset", [&mainEventLoop, &server, &simState](ClientConnection conn, const Json::Value& args)
	{
		mainEventLoop.post([&server, &simState]()
		{
			// Reset the simulation to initial state
			simState = RESET;
			std::cout << "Simulation reset." << std::endl;
		});
	});

	#pragma endregion
	
	// Start the networking thread
	std::thread serverThread([&server]() {
        std::cout << "Server is running on: " << asio::ip::host_name() << std::endl;
		server.run(PORT_NUMBER);
	});
	
	// Start the event loop for the main thread
	asio::io_service::work work(mainEventLoop);

	std::cout << "Starting main event loop..." << std::endl;
	std::thread mainEventLoopThread([&]()
		{
			mainEventLoop.run();
		}
	);
	std::cout << "Main event loop started" << std::endl;

	// Set up simulation //

	// Read config file
    Json::Value config;
    std::ifstream configFile("../config.json");
    if (!configFile.is_open()) {
        std::cerr << "Failed to open config file." << std::endl;
        return -1;
    }
    configFile >> config;

	// Extract damping factor, dt, and URDF filename
    double damping_factor = config["damping_factor"].asDouble();
	std::cout << "Damping factor: " << damping_factor << std::endl;
    double dt = config["dt"].asDouble();
	std::cout << "dt: " << dt << std::endl;
    std::string urdfFilename = config["urdf_filename"].asString();
	std::cout << "urdf: " << urdfFilename << std::endl;
    std::string urdfFilePath = std::string(PINOCCHIO_MODEL_DIR) + "/" + urdfFilename;

	// Initialize simulation state
	simState = STOPPED;
	Model model;
    pinocchio::urdf::buildModel(urdfFilePath,model);
    std::cout << "model name: " << model.name << std::endl;
    
    // Create data required by the algorithms
    Data data(model);

	Eigen::VectorXd tau = Eigen::VectorXd::Zero(model.nv);
    Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nv);
    Eigen::VectorXd v = Eigen::VectorXd::Zero(model.nv);
    Eigen::VectorXd a = Eigen::VectorXd::Zero(model.nv);
           
    auto startTime = std::chrono::high_resolution_clock::now();

	// Main simulation loop
	while (1)
	{	
		switch (simState) {
			case STOPPED:
				break;
			case RESET:
				tau = Eigen::VectorXd::Zero(model.nv);
				q = Eigen::VectorXd::Zero(model.nv);
				v = Eigen::VectorXd::Zero(model.nv);
				a = Eigen::VectorXd::Zero(model.nv);
				simState = STOPPED;
				break;
			case RUNNING:
				tau = damping_factor * v;
				a = pinocchio::aba(model, data, q, v, tau);
				v += a * dt;
				q = pinocchio::integrate(model, q, v*dt);
				break;
			default:
				std::cout << "Simulation state invalid." << std::endl;
				return -1;
		}

		// Measure iteration time
        auto endTime = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime).count();

		// Calculate remaining time to sleep
        auto remaining_time = static_cast<int>(dt * 1000000) - elapsed;
        if (remaining_time > 0)
        {
            std::this_thread::sleep_for(std::chrono::microseconds(remaining_time));
        }
        //std::clog<< "q: " << q.transpose() << "\tdt: " << dt <<std::endl;

        startTime = std::chrono::high_resolution_clock::now();

		// Broadcast the input to all connected clients (is sent on the network thread)
        Json::Value payload;

		// Convert Eigen::VectorXd to Json::Value array
        Json::Value positionsArray(Json::arrayValue);
        for (int i = 0; i < q.size(); ++i) {
             positionsArray.append(q[i]);
        }

        payload["positions"] = positionsArray;
		server.broadcastMessage("simulation", payload);
			
		//Debug output on the main thread
        /*
		mainEventLoop.post([]() {
			//std::clog << "User input debug output on the main thread" << std::endl;
			});
        */
	}
	
	return 0;
}
