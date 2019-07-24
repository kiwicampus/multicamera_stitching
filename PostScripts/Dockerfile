FROM davidnet/kiwibot3:tensorflow
MAINTAINER david@kiwicampus.com

# ENV DEVICE_TYPE=jetson-tx2
WORKDIR /usr/src/app

# # Add script to turn on the fan of Jetson
RUN wget -q https://gist.githubusercontent.com/Davidnet/344751590f4c72c24a3b6fcc20d7e792/raw/9bc06e2c0ed167088fd0243a6fe3a0b1a8e2ecd1/jetson_cores.sh && \
     chmod +x /usr/src/app/jetson_cores.sh

# Set up new repositories
RUN export CLOUD_SDK_REPO="cloud-sdk-xenial" \
    && echo "deb http://packages.cloud.google.com/apt $CLOUD_SDK_REPO main" | tee -a /etc/apt/sources.list.d/google-cloud-sdk.list \
    && curl https://packages.cloud.google.com/apt/doc/apt-key.gpg | sudo apt-key add -

# Install Node
RUN curl -sL https://deb.nodesource.com/setup_8.x | sudo -E bash - && \
    apt-get install -y nodejs && \
    npm config set unsafe-perm true -g --unsafe-perm

# keyboard minimal installation
RUN printf 'XKBMODEL="pc105"\nXKBLAYOUT="us"\nXKBVARIANT=""\nXKBOPTIONS=""\nBACKSPACE="guess"' >> /etc/default/keyboard

RUN DEBIAN_FRONTEND=noninteractive apt-get update && apt-get install -y --no-install-recommends \
  xserver-xorg-core \
  xserver-xorg-video-fbdev \
  xorg \
  libdbus-1-dev \
  libcap-dev \
  libxtst-dev \
  libnss3-dev \
  fluxbox \
  libsmbclient \
  libssh-4 \
  fbset \
  xine-ui \
  libexpat-dev \
  curl \
  dnsmasq \
  iproute2 \
  iw \
  udev \
  rfkill \
  ca-certificates \
  git \
  htop \
  wget \
  ffmpeg \
  vim \
  nano \
  libssl-dev \
  net-tools \
  iputils-ping \
  libi2c-dev \
  i2c-tools \
  mpg123 \
  google-cloud-sdk \
  network-manager \
  chromium-browser \
  lm-sensors \
  && apt-get -y clean all \
  && rm -rf /var/lib/apt/lists/*

# Add ROS sources and install ros-base
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu xenial main" > /etc/apt/sources.list.d/ros-latest.list' && \
    apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
    apt-get update && \
    apt-get install -y ros-kinetic-ros-base \
    python-rosdep \
    python-rosinstall \
    python-rosinstall-generator \
    python-wstool \
    geographiclib-tools \
    wireless-tools \
    ros-kinetic-angles \
    ros-kinetic-geographic-msgs \
    ros-kinetic-geometry \
    ros-kinetic-mavros \
    ros-kinetic-mavros-extras \
    ros-kinetic-mavros-msgs \
    && apt-get -y clean all \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    apt-get install -y  libncurses5-dev libncursesw5-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /usr/src/app
RUN wget -q https://storage.googleapis.com/vision-198622-kiwibot-packages/tegrastats && chmod +x tegrastats
RUN wget -q https://storage.googleapis.com/vision-198622-kiwibot-packages/gtop && chmod +x gtop

# Clone socketio server (for local interface)
WORKDIR /catkin_ws/src/skynet/src/
RUN echo "StrictHostKeyChecking no" >> /etc/ssh/ssh_config
COPY resin_key /catkin_ws/src/skynet/src/
RUN chmod 400 resin_key && \
    chown $USER:$USER resin_key && \
    ssh-agent bash -c 'ssh-add resin_key; git clone git@bitbucket.org:kiwicampus/socketio-server.git'

# Socketio Server install
WORKDIR /catkin_ws/src/skynet/src/socketio-server
RUN npm install

# Install mongodb_logs package and zbar for QR-detection
RUN apt-get update && \
    apt-get install -y ros-kinetic-mongodb-log libzbar-dev \
    && rm -rf /var/lib/apt/lists/*


# Downgrade to chromium 70.0 that is the version that works with out webrtc pipeline
WORKDIR /opt/debian-packages
RUN wget -q https://storage.googleapis.com/vision-198622-kiwibot-packages/debian-packages/chromium-browser_70.0.3538.77-0ubuntu0.16.04.1_arm64.deb && \
    wget -q https://storage.googleapis.com/vision-198622-kiwibot-packages/debian-packages/chromium-codecs-ffmpeg-extra_70.0.3538.77-0ubuntu0.16.04.1_arm64.deb && \
    apt-get install -y --allow-downgrades ./*.deb && rm -rf /opt/debian-packages


# Add python dependencies
RUN python3 -m pip install spidev git+https://github.com/kiwicampus/python-websocket-server git+https://github.com/kiwicampus/easy_memmap && \
    pip install git+https://github.com/kiwicampus/tfinterface.git@develop && \
    pip install speedtest-cli dicto supervisor python_path && \
    pip install selenium && \
    pip install git+https://github.com/kiwicampus/easy_memmap git+https://github.com/charlielito/extended_rospylogs && \
    pip install sh scandir google-cloud-storage ipython pyserial spidev ntplib zbar pyzbar

# Install dependencies for ros robot localization
RUN apt-get update && \
    apt-get install -y ros-kinetic-roslint \
    ros-kinetic-tf2-geometry-msgs \
    libyaml-cpp-dev \
    && apt-get -y clean all \
    && rm -rf /var/lib/apt/lists/*

# Set Xorg and FLUXBOX preferences
RUN mkdir ~/.fluxbox
RUN echo "xset s off" > ~/.fluxbox/startup && echo "xserver-command=X -s 0 dpms" >> ~/.fluxbox/startup
RUN echo "#!/bin/bash" > /etc/X11/xinit/xserverrc \
  && echo "" >> /etc/X11/xinit/xserverrc \
  && echo 'exec /usr/bin/X -s 0 dpms -nocursor -nolisten tcp "$@"' >> /etc/X11/xinit/xserverrc

# Config file for videoplayer
RUN mkdir /root/.xine
COPY configs/config /root/.xine/config

# Adding nvpmodel.conf to etc
COPY configs/nvpmodel.conf /etc/

# Move app to filesystem
WORKDIR /usr/src/app
COPY ./video_app ./
RUN cp Xwrapper.config /etc/X11/

# Copy memes video folder
COPY ./video_memes /usr/src/app/video_memes/

# WebRTC Stuff
COPY webrtc/ /usr/src/app/webrtc
COPY webrtc/package-lock.json webrtc/package.json /usr/src/app/webrtc/
WORKDIR /usr/src/app/webrtc
RUN PUPPETEER_SKIP_CHROMIUM_DOWNLOAD=1 npm install

# Google cloud AUTH
WORKDIR /usr/src/app
COPY configs/vision_key.json /usr/src/app/
COPY configs/calibration_gcp.json /usr/src/app/
RUN gcloud auth activate-service-account --key-file vision_key.json \
    && gcloud config set project vision-198622

# Move to app dir
WORKDIR /usr/src/app

# Install geographiclib stuff
#RUN /opt/ros/kinetic/lib/mavros/install_geographiclib_datasets.sh
RUN mkdir -p /usr/share/GeographicLib
COPY configs/GeographicLib /usr/share/GeographicLib/


# Add mount details to fstab
RUN echo "/dev/sda1 /media ext4 rw,relatime,discard,data=ordered 0 2" >> /etc/fstab


# **** ENV Variables *****
ENV INITSYSTEM on
ENV ROS_HOSTNAME=localhost
# Add path to chrome driver for scrapping
ENV PATH $PATH:/catkin_ws/src/link_4g/src 
ENV DBUS_SYSTEM_BUS_ADDRESS unix:path=/host/run/dbus/system_bus_socket
# # Copy Video Folder
ENV VIDEO_URL https://storage.googleapis.com/vision-198622-kiwibot-packages/black-eyes.mp4
ENV VIDEO_PATH /data/black-eyes.mp4

# **** Freedom Robotics Integration ***** #
RUN curl -s "https://api.futurerobot.co/proto/accounts/INSTALL_ONLY/devices/GENERIC_DEVICE/installscript?mc_token=INSTALL_ONLY_DEVICE_TOKEN&install_elements=no_ssh,no_credentials,service_none" | python

# Initialize rosdep
RUN rosdep init && rosdep update

# Creating workspace for catkin
RUN mkdir -p /catkin_ws && \
    mkdir -p /catkin_ws/src

# Aux Neural Nets
WORKDIR /catkin_ws/src/skynet/src/
# Clone Objectdetection Repo to run locally
# RUN echo update models
RUN ssh-agent bash -c 'ssh-add resin_key; git clone git@bitbucket.org:kiwicampus/models.git'
RUN cd models && pip install -e .
RUN git clone https://github.com/Davidnet/real_time_object_detection


# Copy scripts
COPY configs/freedom_robotics/ /usr/src/app/freedom_robotics/
COPY ./kiwi_wifi_enforcement /usr/src/app/wifi_enforcement
COPY configs/camera_paremeters_640_480.yaml /usr/src/app/
COPY configs/camera_paremeters_640_360.yaml /usr/src/app/
COPY configs/mono_vision_default_params_C.npz /usr/src/app/
COPY configs/stitcher_default_config.npz /usr/src/app/
COPY configs/Launch/kiwiBot.launch /usr/src/app/
COPY configs/patch3.sh /usr/src/app/
COPY configs/doorTest.sh /usr/src/app/
COPY resin_files/ /usr/src/app/
COPY lights/ /usr/src/app/
COPY px4/mavros/* /opt/ros/kinetic/share/mavros/launch/
COPY websocket-server/ /usr/src/app/websocket-server

# Copy and execute ROS code
WORKDIR /catkin_ws
COPY ros/src/ /catkin_ws/src/
# Init websocket lib
RUN git clone https://github.com/kiwicampus/websocketpp.git /catkin_ws/src/web_client/src/websocketpp
RUN /bin/bash -c "source /opt/ros/kinetic/setup.bash ; catkin_make"

# Start the application
WORKDIR /usr/src/app

CMD ["bash", "/usr/src/app/resin-start.sh"]
