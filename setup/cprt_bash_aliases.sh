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

alias pingBaseAntenna='echo "Pinging 192.168.0.2" && ping 192.168.0.2'
alias pingRoverAntenna='echo "Pinging 192.168.0.3" && ping 192.168.0.3'
alias pingJetson='echo "Pinging 192.168.0.55" && ping 192.168.0.55'

if command -v sshpass &> /dev/null; then
    create_alias_with_echo sshJetson 'sshpass -p cprt ssh cprt@192.168.0.55'
else
    create_alias_with_echo sshJetson 'ssh cprt@192.168.0.55'
fi


create_alias_with_echo lsCams 'ls /dev/v4l/by-id'

create_alias_with_echo openWebUI 'chromium http://localhost:3000'
create_alias_with_echo openRoverCode 'code $ROVER_DIR'
create_alias_with_echo openWebCode 'code $WEB_DIR'

create_alias_with_echo rvizMoveit 'rviz2 -d $ROVER_DIR/src/arm_srdf/config/moveit.rviz'
create_alias_with_echo rvizArm 'rviz2 -d $ROVER_DIR/src/arm_srdf/config/moveit.rviz'



# Launch helpers

create_alias_with_echo launchBasestation "$rover_source_name && ros2 launch bringup basestation.launch.py"
create_alias_with_echo launchEquipementServicing "$rover_source_name && ros2 launch bringup equipment_servicing.launch.py"
create_alias_with_echo launchNav "$rover_source_name && ros2 launch bringup nav.launch.py"
create_alias_with_echo launchScience "$rover_source_name && ros2 launch bringup science.launch.py"
create_alias_with_echo launchTraversal "$rover_source_name && ros2 launch bringup transversal.launch.py"


 



