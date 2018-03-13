#!/bin/bash

# The first argument is the number of trials (5 by default)
if [ -z "$1" ]
then
    # No arguent supplied, use default
    NB_TRIALS=5
else
    NB_TRIALS=$1
fi

# The second argument is whether to use the dev build
if [ -z "$2" ]
then
    # No arguent supplied, use default
    DEV=0
else
    DEV=$2
fi

# The third argument is whether to run inside docker or not
if [ -z "$3" ]
then
    # No arguent supplied, use default
    DOCKER=1
else
    DOCKER=$3
fi

# Current directory
SCRIPT=`realpath $0`
SCRIPTPATH=`dirname $SCRIPT`

# Start
echo "Starting evaluation with" $NB_TRIALS "trials"
if [ $DOCKER == "1" ]; then echo "Using docker"; fi;
if [ $DEV == "1" ]; then echo "Using dev build"; fi;

for (( TRIAL=1;  TRIAL<=$NB_TRIALS; TRIAL++))
do
    echo "#########################"
    echo "Trial number" $TRIAL "..."
    echo "#########################"
    
    # Set truck and platform initial positions
    $SCRIPTPATH/set_platform_truck_position.py $TRIAL
    
    # Run simulation
    if [ $DOCKER == "1" ] 
    then
        if [ $DEV == "1" ] 
        then
            make -j 1 dronecourse_dev_gazebo_eval
        else
            make -j 1 dronecourse_gazebo_eval
        fi
    else
        if [ $DEV == "1" ] 
        then
            make -j 1 _dronecourse_dev_gazebo_eval
        else
            make -j 1 _dronecourse_gazebo_eval
        fi
    fi
    
    # Run evaluation
    if [ $DEV == "1" ] 
    then
        $SCRIPTPATH/evaluate.py -e -d
    else
        $SCRIPTPATH/evaluate.py -e
    fi
done

# Reset truck and platform positions
$SCRIPTPATH/set_platform_truck_position.py 0