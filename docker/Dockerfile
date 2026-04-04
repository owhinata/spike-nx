FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    gcc-arm-none-eabi \
    libnewlib-arm-none-eabi \
    git \
    python3 \
    python3-pip \
    python3-venv \
    pipx \
    unzip \
    zip \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --break-system-packages pycryptodomex

ENV PATH="/root/.local/bin:${PATH}"
RUN pipx install poetry
