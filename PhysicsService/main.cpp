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
#include <zmq.hpp>
#include "build/protos/motion_control.pb.h"

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

Eigen::VectorXd tau;
Eigen::VectorXd q;
Eigen::VectorXd v;
Eigen::VectorXd a;

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
	
	// Start the Websocket thread
	std::thread serverThread([&server]() {
        std::cout << "Server is running on: " << asio::ip::host_name() << std::endl;
		server.run(PORT_NUMBER);
	});

	// ZeroMQ server setup //
	
	// Create a ZeroMQ context
    zmq::context_t context(1);
    // Create a socket to listen for incoming connections
    zmq::socket_t socket(context, ZMQ_REP);
    // Bind the socket to a specific port
    socket.bind("tcp://*:5555"); // Adjust the port number as needed
	
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

	tau = Eigen::VectorXd::Zero(model.nv);
    q = Eigen::VectorXd::Zero(model.nv);
    v = Eigen::VectorXd::Zero(model.nv);
    a = Eigen::VectorXd::Zero(model.nv);
           
    auto startTime = std::chrono::high_resolution_clock::now();

	// Main simulation loop
	while (1)
	{	
		// Receive a control message
        zmq::message_t message;
		try
		{
			socket.recv(&message);
		}
		catch(const std::exception& e)
		{
			std::cout << e.what() << std::endl;
			continue;
		}

        // Parse the received message
        motion_control::Request request;
        request.ParseFromArray(message.data(), message.size());

        // Handle the received message
        auto handleRequest = [&](const motion_control::Request& request) {
            // Define the handling logic for different message types here
            switch (request.type()) {
                case motion_control::Request::SET_JOINT_POSITION:
                    // Handle SET_JOINT_POSITION message type
                    break;
                case motion_control::Request::SET_JOINT_VELOCITY:
                    // Handle SET_JOINT_VELOCITY message type
                    break;
				case motion_control::Request::SET_JOINT_TORQUE:
                    // Handle SET_JOINT_TORQUE message type
					if(request.has_set_joint_torque()){
						auto input_size = request.set_joint_torque().torque_size();
						tau = Eigen::VectorXd(input_size);
						// Populate the Eigen::VectorXd with torque values
						for (int i = 0; i < input_size; ++i) {
							tau(i) = (double)request.set_joint_torque().torque(i);
						}
					}
					 
                    break;
                // Add cases for other message types as needed
                default:
                    // Handle unknown message type
                    break;
            }
        };

        // Call the handleMessage lambda function with the received message
        handleRequest(request);

		// Prepare a response message
		// TODO: Delete
		std::string response = "Dummy response";
		zmq::message_t reply(response.size());
		memcpy(reply.data(), response.data(), response.size());
		socket.send(reply);
		// ENDTODO

        // Prepare a response message
        // Example: create an acknowledgment message
    	//motion_control::Acknowledge acknowledge;
        //acknowledge.set_acknowledge("Message received successfully");

        // Serialize the response message
        //std::string response = "Ok";
        //acknowledge.SerializeToString(&response);

        // Send the response
        //zmq::message_t reply(response.size());
        //memcpy(reply.data(), response.data(), response.size());
        //socket.send(reply);
		

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
				tau += damping_factor * v;
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
