#
# Install ROS packages used by Autoware.
#
FROM gitlab.cs.ut.ee:5050/autonomous-driving-lab/autoware.ai/local/autoware_misc/autoware:latest
ARG CI_REPOSITORY_URL
ARG ROS_DISTRO

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Install tools and libraries required by Autoware
RUN apt-get -qq update && \
apt-get install -qq --no-install-recommends -y python-pip python-setuptools python-rosinstall python-catkin-tools libglfw3 libglfw3-dev && \
python2 -m pip install --no-cache-dir -U pip && \
pip2 --no-cache-dir install -U setuptools wheel && \
pip2  install --no-cache-dir setuptools keras==2.2.4 pandacan paho-mqtt readchar && \
rm -rf /var/lib/apt/lists/*

# AS Repos
RUN apt-get -qq update && \
apt-get -qq install --no-install-recommends software-properties-common && \
apt-add-repository -y ppa:astuff/kvaser-linux && \
apt-get -qq install --no-install-recommends apt-transport-https && \
sh -c 'echo "deb [trusted=yes] https://s3.amazonaws.com/autonomoustuff-repo/ $(lsb_release -sc) main" > /etc/apt/sources.list.d/autonomoustuff-public.list' && \
apt-get -qq update && \
sh -c 'echo "yaml https://s3.amazonaws.com/autonomoustuff-repo/autonomoustuff-public-'$ROS_DISTRO'.yaml" > /etc/ros/rosdep/sources.list.d/40-autonomoustuff-public-'$ROS_DISTRO'.list' && \
rm -rf /var/lib/apt/lists/*

# Configure environmet
RUN update-locale LANG=en_US.UTF-8 LC_MESSAGES=POSIX

# Add user
ENV USERNAME autoware
ARG USER_ID=1000
ARG GROUP_ID=15214
ENV PULSE_SERVER /run/pulse/native

USER autoware

# Configure terminal colors
RUN gconftool-2 --set "/apps/gnome-terminal/profiles/Default/use_theme_background" --type bool false && \
    gconftool-2 --set "/apps/gnome-terminal/profiles/Default/use_theme_colors" --type bool false && \
    gconftool-2 --set "/apps/gnome-terminal/profiles/Default/background_color" --type string "#000000"

RUN echo $CI_REPOSITORY_URL

# Installing ut_aw_pkgs from src
RUN bash -c 'mkdir -p /home/$USERNAME/autoware_build/src &&  \
    cd /home/$USERNAME/autoware_build/src && \
    git clone $CI_REPOSITORY_URL && \
    cd /home/$USERNAME/autoware_build && \
    source $HOME/autoware.ut/install/setup.bash && \
    sudo apt-get -qq update && \
    rosdep update && \
    rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO && \
    MAKEFLAGS="-j3 -l3" colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --executor sequential && \
    source ./install/setup.bash && \
    sudo rm -rf /var/lib/apt/lists/*'

USER root
