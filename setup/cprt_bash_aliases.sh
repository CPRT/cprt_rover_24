# CPRT Bash Aliases

if [ "$0" = "$BASH_SOURCE" ]; then
    echo "Error: cprt_bash_aliases.sh must be sourced"
    exit 1
fi

echo "Added CPRT bash aliases"

debug=0

create_alias_with_echo() {
  local alias_name="$1"
  local command_string="$2"

  if [ "$debug" != 0 ]; then
    echo "Adding alias with name: $alias_name and command: $command_string"
  fi

  alias "$alias_name"="echo Running: '$command_string'; eval \"$command_string\""
}

alias ea='nano $ROVER_DIR/setup/cprt_bash_aliases.sh'
alias eaNano='nano $ROVER_DIR/setup/cprt_bash_aliases.sh'
alias eaVim='vim $ROVER_DIR/setup/cprt_bash_aliases.sh'

alias home='cd $ROVER_DIR'
alias gohome='cd $ROVER_DIR'
alias goRoverHome='cd $ROVER_DIR'
alias goWebHome='cd $WEB_DIR'
alias homeWeb='cd $WEB_DIR'

alias roverSource='source $ROVER_DIR/install/setup.bash'
rover_source_name=roverSource # Used below

create_alias_with_echo goLatLonDir 'cd $ROVER_DIR/src/nav_commanders/known_gps_coords'

create_alias_with_echo pingBaseAntenna 'ping 192.168.0.2'
create_alias_with_echo pingRoverAntenna 'ping 192.168.0.3'
create_alias_with_echo pingJetson 'ping 192.168.0.55'
create_alias_with_echo pingJetsonWifi 'ping 192.168.1.111'
create_alias_with_echo pingBattleStation 'ping 192.168.0.20'
create_alias_with_echo pingBattleStationWifi 'ping 192.168.1.100'

create_alias_with_echo sshJetson 'ssh cprt@192.168.0.55'
create_alias_with_echo sshJetsonWifi 'ssh cprt@192.168.1.111'
create_alias_with_echo sshBattleStation 'ssh cprt@192.168.0.20'
create_alias_with_echo sshBattleStationWifi 'ssh cprt@192.168.1.100'

create_alias_with_echo lsCams 'ls /dev/v4l/by-id'

create_alias_with_echo openWebUI 'chromium http://localhost:3000'
create_alias_with_echo openRoverCode 'code $ROVER_DIR'
create_alias_with_echo openWebCode 'code $WEB_DIR'

create_alias_with_echo rvizMoveit 'rviz2 -d $ROVER_DIR/src/arm_srdf/config/moveit.rviz'
create_alias_with_echo rvizArmMoveIt 'rviz2 -d $ROVER_DIR/src/arm_srdf/config/moveit.rviz'
create_alias_with_echo rvizNav 'rviz2 -d $ROVER_DIR/src/navigation/rviz/nav2_cprt.rviz'


# Launch helpers

create_alias_with_echo launchBasestation "$rover_source_name && ros2 launch bringup basestation.launch.py" 
create_alias_with_echo launchEquipementServicing "$rover_source_name && ros2 launch bringup equipment_servicing.launch.py"
create_alias_with_echo launchNav "$rover_source_name && ros2 launch bringup nav.launch.py"
create_alias_with_echo launchScience "$rover_source_name && ros2 launch bringup science.launch.py"
create_alias_with_echo launchTraversal "$rover_source_name && ros2 launch bringup transversal.launch.py"


 



