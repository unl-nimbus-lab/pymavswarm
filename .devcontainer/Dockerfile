FROM python:3

ENV DEBIAN_FRONTEND=noninteractive

ARG USERNAME=developer
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Add non-root user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && rm -rf /var/lib/apt/lists/* \
    && echo "source /usr/share/bash-completion/completions/git" >> /home/$USERNAME/.bashrc \
    && echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/$USERNAME/.bashrc

# Install apt packages
RUN apt-get update \
    && apt-get -y install --no-install-recommends \
    software-properties-common \
    apt-utils \
    python3-dev \
    gcc \
    git \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*

# Update pip to latest version
RUN pip3 install --upgrade pip

# Install required Python packages
RUN pip3 install \
    monotonic \
    black \
    pylint \
    pymavlink \
    isort

ENV PATH "${PATH}:/home/developer/.local/bin"

ENV DEBIAN_FRONTEND=dialog