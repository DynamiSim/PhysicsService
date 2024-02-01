#include "WebsocketServer.h"

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/aba.hpp"

#include <iostream>
#include <thread>
#include <asio/io_service.hpp>
#include <chrono>

// PINOCCHIO_MODEL_DIR is defined by the CMake but you can define your own directory here.
#ifndef PINOCCHIO_MODEL_DIR
  #define PINOCCHIO_MODEL_DIR "/workspaces/PhysicsService/robots"
#endif

//The port number the WebSocket server listens on
#define PORT_NUMBER 1234

using namespace pinocchio;

int main(int argc, char* argv[])
{
	//Create the event loop for the main thread, and the WebSocket server
	asio::io_service mainEventLoop;
	WebsocketServer server;
	
	//Register our network callbacks, ensuring the logic is run on the main thread's event loop
	server.connect([&mainEventLoop, &server](ClientConnection conn)
	{
		mainEventLoop.post([conn, &server]()
		{
			//std::clog << "Connection opened." << std::endl;
			//std::clog << "There are now " << server.numConnections() << " open connections." << std::endl;
			
			//Send a hello message to the client
			server.sendMessage(conn, "hello", Json::Value());
		});
	});
	server.disconnect([&mainEventLoop, &server](ClientConnection conn)
	{
		mainEventLoop.post([conn, &server]()
		{
			//std::clog << "Connection closed." << std::endl;
			//std::clog << "There are now " << server.numConnections() << " open connections." << std::endl;
		});
	});
	server.message("message", [&mainEventLoop, &server](ClientConnection conn, const Json::Value& args)
	{
		mainEventLoop.post([conn, args, &server]()
		{
			//std::clog << "message handler on the main thread" << std::endl;
			//std::clog << "Message payload:" << std::endl;
			for (auto key : args.getMemberNames()) {
				//std::clog << "\t" << key << ": " << args[key].asString() << std::endl;
			}
			
			//Echo the message pack to the client
			server.sendMessage(conn, "message", args);
		});
	});
	
	//Start the networking thread
	std::thread serverThread([&server]() {
        std::cout << "Server is running on: " << asio::ip::host_name() << std::endl;
		server.run(PORT_NUMBER);
	});
	
	//Start an input thread that reads from the simulation
	std::thread inputThread([&server, &mainEventLoop]()
	{
        // You should change here to set up your own URDF file or just pass it as an argument of this example.
        const std::string urdf_filename = PINOCCHIO_MODEL_DIR + std::string("/ur10e/robot.urdf");
        
        // Load the urdf model
        Model model;
        pinocchio::urdf::buildModel(urdf_filename,model);
        std::cout << "model name: " << model.name << std::endl;
        
        // Create data required by the algorithms
        Data data(model);

        // Set initial robot data
        Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nv);
        Eigen::VectorXd v = Eigen::VectorXd::Zero(model.nv);
        Eigen::VectorXd a = Eigen::VectorXd::Zero(model.nv);
        Eigen::VectorXd tau = Eigen::VectorXd::Zero(model.nv);
        
        
        auto startTime = std::chrono::high_resolution_clock::now();

        // TODO: Add exit criteria
		while (1)
		{	

            // Generate joint positions
            a = pinocchio::aba(model, data, q, v, tau);

            // Measure iteration time
            auto endTime = std::chrono::high_resolution_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);
            auto dt = elapsed.count() / 1000000.0; // Adjust dt dynamically based on iteration time
            dt = 0.0005;
            v += a * dt;
            q += v * dt;
            //std::cout << "q: " << q.transpose() << "\tdt: " << dt <<std::endl;

            startTime = std::chrono::high_resolution_clock::now();

			//Broadcast the input to all connected clients (is sent on the network thread)
            Json::Value payload;

			// Convert Eigen::VectorXd to Json::Value array
            Json::Value positionsArray(Json::arrayValue);
            for (int i = 0; i < q.size(); ++i) {
                positionsArray.append(q[i] * 0.0174532925);
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
	});
	
	//Start the event loop for the main thread
	asio::io_service::work work(mainEventLoop);
	mainEventLoop.run();
	
	return 0;
}
