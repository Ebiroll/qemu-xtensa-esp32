# Use Ubuntu 20.04 as base — includes legacy Python2 in apt
FROM ubuntu:20.04

# Avoid interactive prompts during package installs
ENV DEBIAN_FRONTEND=noninteractive

# Basic dev tools + Python2 + QEMU build dependencies placeholder
RUN apt update && apt upgrade -y && \
    apt install -y \
      build-essential \
      git \
      curl \
      wget \
      pkg-config \
      automake \
      autoconf \
      libtool \
      make \
      gcc \
      g++ \
      python2 \
      python2-dev \
      python-is-python2 \
      libssl-dev \
      libglib2.0-dev \
      zlib1g-dev \
      sudo \
      # Add any other dependencies needed by your fork here
      && apt clean && rm -rf /var/lib/apt/lists/*

# Symlink python -> python2 explicitly if not already
#RUN ln -s /usr/bin/python2 /usr/bin/python

# Create a non-root user (optional)
ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt install -y sudo \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

# Set default user and working directory
USER $USERNAME

# Default CMD (shell)
CMD [ "/bin/bash" ]

