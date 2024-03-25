# Use the official VS Code Remote - Containers image
FROM mcr.microsoft.com/devcontainers/base:1.0-bullseye

# Install required dependencies
RUN apt-get update && export DEBIAN_FRONTEND=noninteractive \
    && apt-get -y install --no-install-recommends lsb-release curl cmake \
        libwebsocketpp-dev libasio-dev libjsoncpp-dev libprotobuf-dev \ 
        protobuf-compiler libzmq5-dev \
        python3 python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Install additional Python packages for testing
RUN pip3 install websocket-client protobuf==3.20.* zmq

# Fix json.h path
RUN ln -s /usr/include/jsoncpp/json/ /usr/include/json

# Copy repository to the container
COPY . /workspaces/PhysicsService
RUN chmod +x /workspaces/PhysicsService/scripts/install-pinocchio.sh
RUN /workspaces/PhysicsService/scripts/install-pinocchio.sh

RUN ls -R /workspaces/PhysicsService/PhysicsService

# Expose the environment variables
ENV PATH=/opt/openrobots/bin:$PATH
ENV PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH
ENV LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH
ENV PYTHONPATH=/opt/openrobots/lib/python3.10/site-packages:$PYTHONPATH
ENV CMAKE_PREFIX_PATH=/opt/openrobots:$CMAKE_PREFIX_PATH

# Build the physics service
WORKDIR /workspaces/PhysicsService/PhysicsService/build
RUN rm -r /workspaces/PhysicsService/PhysicsService/build/*
RUN cmake ..
RUN make

EXPOSE 1234
EXPOSE 5555

ENTRYPOINT [ "./physics_service" ]