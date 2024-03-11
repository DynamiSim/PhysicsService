import zmq
import time
from google.protobuf import json_format
from motion_control_pb2 import Request, JointTorqueGoal

def main():
    while True:
        # Create a ZeroMQ context
        context = zmq.Context()

        # Create a socket to connect to the ZeroMQ server
        socket = context.socket(zmq.REQ)
        socket.connect("tcp://localhost:5555")  # Adjust the address and port as needed

        # Create a SET_JOINT_TORQUE message
        request = Request()
        request.type = Request.SET_JOINT_TORQUE

        # Populate the JointTorqueGoal message
        joint_torque_goal = JointTorqueGoal()
        joint_torque_goal.torque.extend([0.2, 0.2, 0.0, 0.0, 0.0, 0.0])  # Adjust the torque values as needed

        # Set the JointTorqueGoal message in the Request
        request.set_joint_torque.CopyFrom(joint_torque_goal)

        # Serialize the message to send it over the wire
        message_bytes = request.SerializeToString()

        # Send the message
        socket.send(message_bytes)

        # Wait for a response (optional)
        response_bytes = socket.recv()

        # Optionally deserialize the response
        #response = Response()
        #response.ParseFromString(response_bytes)

        # Print the response (optional)
        #print(response)

    # Close the socket and context
    socket.close()
    context.term()

if __name__ == "__main__":
    main()
