# Install Speaker Utils
RUN apt-get update && apt-get install -y \
    python3-pip \
    portaudio19-dev \
    alsa-utils \
    espeak \
    libespeak-dev \
    && rm -rf /var/lib/apt/lists/*

# Add user to audio group
RUN usermod -a -G audio ros

# Install Python packages as the ros user
USER ros
RUN pip3 install --user sounddevice
USER root