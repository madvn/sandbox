# syntax = docker/dockerfile:1.0-experimental
FROM ubuntu:18.04

# Install apt dependencies first
RUN apt update \
 && apt install -y --no-install-recommends \
    ca-certificates \
    curl \
    build-essential \
    libgtest-dev
#   list your project build dependencies here
#   only one per line for easy reading

# Install other non-apt dependencies
# Install cmake >= 3.14
ENV CMAKE_VERSION=3.19.5
WORKDIR /tmp
RUN curl -OL https://github.com/Kitware/CMake/releases/download/v${CMAKE_VERSION}/cmake-${CMAKE_VERSION}-Linux-x86_64.sh \
 && chmod +x cmake-${CMAKE_VERSION}-Linux-x86_64.sh \
 && ./cmake-${CMAKE_VERSION}-Linux-x86_64.sh --skip-license --prefix=/usr \
 && rm -f cmake-${CMAKE_VERSION}-Linux-x86_64.sh

# Build GTest
RUN cd /usr/src/googletest \
 && cmake . \
 && cmake --build . -j --target install

# Copy only what's needed to build and package the application
WORKDIR /app
COPY deb /app/deb
COPY modules /app/modules
COPY apps /app/apps
COPY CMakeLists.txt /app
COPY cmake_templateConfig.cmake.in /app

# Build the installer
WORKDIR /app/build
RUN cmake -DCMAKE_BUILD_TYPE=Release .. \
 && make -j package \
 && ls *.deb > .deb

# Copy the tests
COPY test /app/test

# Build the tests
WORKDIR /app/build
RUN cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=ON .. \
 && make -j

# Copy in the script used to run the tests
COPY test.sh .
RUN chmod +x test.sh

# Set the command which runs when a container is started
ENTRYPOINT ["./test.sh"]
