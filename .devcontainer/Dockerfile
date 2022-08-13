###################################################################
# Base Stage: Includes only the packages required for production
###################################################################
FROM python:3 as base

ENV DEBIAN_FRONTEND=noninteractive

# Update pip to latest version
RUN pip3 install --upgrade pip

# Install all required Python packages
RUN pip3 install \
    pandas \
    numpy \
    monotonic \
    pymavlink \
    pyserial \
    PyYAML


###################################################################
# Development Stage: Creates a development environment
###################################################################
FROM base as development

# Install dev apt packages
RUN apt-get update \
    && apt-get -y install --no-install-recommends \
    apt-utils \
    python3-dev \
    nano \
    vim \
    git \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*

# Install debugging/linting Python packages
RUN pip3 install \
    pipenv \
    black \
    flake8 \
    isort \
    pydocstyle \
    mypy \
    bandit \
    coverage \
    pre-commit \
    tox

# Install required packaging and deployment tools
RUN pip3 install \
    build \
    twine \
    check-wheel-contents

# Install visualization tools
RUN pip3 install \
    matplotlib \
    plotly \
    kaleido

# Configure a new non-root user
ARG USERNAME=developer
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && rm -rf /var/lib/apt/lists/* \
    && echo "source /usr/share/bash-completion/completions/git" >> /home/$USERNAME/.bashrc \
    && echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/$USERNAME/.bashrc

# Install packages needed for enabling OpenGL and X11
RUN apt-get update && apt-get install -y -qq --no-install-recommends \
    libglvnd0 \
    libgl1 \
    libglx0 \
    libegl1 \
    libxext6 \
    libx11-6

# Configure environment variables needed
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute
ENV QT_X11_NO_MITSHM 1

# Add the .local/bin directory to the path
ENV PATH "${PATH}:/home/$USERNAME/.local/bin"
