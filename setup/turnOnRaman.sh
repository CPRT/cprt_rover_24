#!/usr/bin/env bash
# turnOnRaman.sh – run the Raman node as root

# ----- figure out the *real* home of the user who invoked sudo -----
ME=${SUDO_USER:-$USER}                        # chris
USER_HOME=$(getent passwd "$ME" | cut -d: -f6)
WS="$USER_HOME/cprt_rover_24"                # /home/chris/cprt_rover_24

# ----- bring ROS + your workspace into root's environment -----
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

# (optional) expose your user-site packages to root as well
export PYTHONPATH="$USER_HOME/.local/lib/python3.10/site-packages:$PYTHONPATH"

exec ros2 run science_sensors raman "$@"


#This is how to properly call the service since it runs as sudo
: '

sudo -E bash -c '
  source /opt/ros/humble/setup.bash
  source ~/cprt_rover_24/install/setup.bash   # if the client needs your workspace
  ros2 service call /get_raman_spectrum interfaces/srv/Raman \
    "{inittime: 500, scansavg: 1, smoothing: 1}"
'


'