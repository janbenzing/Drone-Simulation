#!/bin/bash
# Supervisor

# start simulation supervisor
if [ ! -f Tools/simsupervisor/build/simsupervisor ]; then
    echo "Error: could not find simulation supervisor" >&2
    exit 1
fi

# read stdin line by line
while IFS= read -r line
do
    echo "PX4: $line" >>"$1"
    if [ "$line" == "WARN  [offboard_controller] [daemon] starting" ]
    then # our controller is started
        echo "dronecourse mission" # start mission
        cout=$(mktemp)
        echo "simsuperviser started" >&2
        (Tools/simsupervisor/build/simsupervisor >$cout; cat $cout | sed 's/^/SUP: /' >>"$1"; rm $cout) & >&2
        echo "simsuperviser stopped" >&2
    fi
done
