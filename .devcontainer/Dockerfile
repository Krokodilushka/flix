FROM ubuntu:22.04

# Установка зависимостей
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    ca-certificates \
    tar \
    gzip \
    git \
    make \
    python3 \
    gnupg \
    build-essential \
    libsdl2-dev \
    cmake

# Установка arduino-cli
RUN curl -sL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | \
    env BINDIR=/usr/local/bin sh

# Установка gazebo classic
RUN curl -sL http://get.gazebosim.org | sh