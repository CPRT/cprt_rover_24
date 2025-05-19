# CPRT Bash rc

if [ "$0" = "$BASH_SOURCE" ]; then
    echo "Error: cprt_bash_rc.sh must be sourced"
    exit 1
fi

if [[ -z "${OVERRIDE_ROVER_DIR}" ]]; then
    export ROVER_DIR=$(cd ~/cprt_rover_24 && pwd)
    # echo "Set ROVER_DIR env variable to $ROVER_DIR"
else
    export ROVER_DIR=$(cd $OVERRIDE_ROVER_DIR && pwd)
    # echo "Setting ROVER_DIR to OVERRIDE_ROVER_DIR, so ROVER_DIR set to $ROVER_DIR"
fi

if [[ -z "${OVERRIDE_WEB_DIR}" ]]; then
    export WEB_DIR=$(cd ~/cprt_web_ui_24 && pwd)
    # echo "Set WEB_DIR env variable to $WEB_DIR"
else 
    export WEB_DIR=$(cd $OVERRIDE_WEB_DIR && pwd)
    # echo "Setting WEB_DIR to OVERRIDE_WEB_DIR, so WEB_DIR set to $WEB_DIR"
fi


source "$ROVER_DIR/setup/cprt_bash_aliases.sh"

