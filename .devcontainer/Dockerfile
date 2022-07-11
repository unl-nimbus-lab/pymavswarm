###################################################################
# Base Stage: Includes only the packages required for production
###################################################################
FROM python:3 as base

ENV DEBIAN_FRONTEND=noninteractive

# Update pip to latest version
RUN pip3 install --upgrade pip

# Install all required Python packages
RUN pip3 install \
    monotonic \
    pymavlink \
    pyserial


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
    && rm -rf /var/lib/apt/lists/*

# Add the .local/bin directory to the path
ENV PATH "${PATH}:/home/$USERNAME/.local/bin"
