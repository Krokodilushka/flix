FROM debian:bullseye-slim

# Установка зависимостей
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    ca-certificates \
    tar \
    gzip \
    git \
    make \
    python3 \
    && rm -rf /var/lib/apt/lists/*

# Установка arduino-cli
RUN curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | \
    env BINDIR=/usr/local/bin sh
