# Use the official VS Code Remote - Containers image
FROM mcr.microsoft.com/devcontainers/base:1.0-bullseye

# Install required dependencies
RUN apt-get update && export DEBIAN_FRONTEND=noninteractive \
    && apt-get -y install --no-install-recommends lsb-release curl cmake libwebsocketpp-dev libasio-dev libjsoncpp-dev \
    && rm -rf /var/lib/apt/lists/*

# Fix json.h path
RUN ln -s /usr/include/jsoncpp/json/ /usr/include/json

# Copy repository to the container
COPY . /workspaces
RUN chmod +x /workspaces/scripts/install-pinocchio.sh
RUN /workspaces/scripts/install-pinocchio.sh

RUN chown -R vscode /workspaces/PhysicsService

# Switch back to non-root to install packages
USER vscode

# Expose the environment variables
ENV PATH=/opt/openrobots/bin:$PATH
ENV PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH
ENV LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH
ENV PYTHONPATH=/opt/openrobots/lib/python3.10/site-packages:$PYTHONPATH
ENV CMAKE_PREFIX_PATH=/opt/openrobots:$CMAKE_PREFIX_PATH

EXPOSE 1234
