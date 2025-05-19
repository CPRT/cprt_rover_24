#!/bin/bash

PRINT=0
CAMERA=
while [ "$#" -gt 0 ]; do
  case "$1" in
    -p | --print)
      PRINT=1
      shift
      ;;
    *)
      echo "Unknown option: $1"
      exit 1
      ;;
  esac
done

if [ "$PRINT" = "1" ]; then
  echo "Printing ports and not doing anything to them"
fi

restart_all_usbs() {
  search_dir=/sys/bus/usb/devices/
  echo "Searching ports at $search_dir"
  for entry in "$search_dir"*
  do
    base_name=$(basename $entry)
    if [[ $base_name != *usb* ]]; then
      echo "Skipping $base_name"
      continue
    fi

    if [ "$PRINT" = "0" ]; then
      echo "Disabling port at $entry/bConfigurationValue"
      echo 0 | sudo tee $entry/bConfigurationValue
    else 
      echo "Would disable port at $entry/bConfigurationValue"
    fi
  done

  if [ "$PRINT" = "1" ]; then
    exit 0
  fi

  echo "Waiting for 5 seconds..."
  sleep 5

  for entry in "$search_dir"*
  do
    base_name=$(basename $entry)
    if [[ $base_name != *usb* ]]; then
      echo "Skipping $base_name"
      continue
    fi

    echo "Enabling port at $entry/bConfigurationValue"
    echo 1 | sudo tee $entry/bConfigurationValue
  done
}

restart_all_usbs

