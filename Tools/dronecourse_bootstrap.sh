#!/usr/bin/env bash
# script to compile and run the autopilot and the gazebo simulation
# Created for the EPFL LIS Dronecourse

# Use with "-a" or "-all" to also recompile the gazebo simulation

set -eu

# Generate waypoints code
Tools/show-waypoints.sh -c > src/include/waypoints.hpp

MAIN_DIR="$PWD"
DEV_MODE=false
DRONECOURSE_DEV_DIR="./src/dronecourse_dev"
DRONECOURSE_BKP_DIR="./src/dronecourse_bkp"
# the randomly named file ensure that we don't overwrite student files
DRONECOURSE_ACTUAL_DEV="$DRONECOURSE_DEV_DIR/f8t0jrzgb5968yoylwqthhky6m1s5vhr"
DRONECOURSE_DIR="./src/dronecourse"
DRONECOURSE_NOT_ACTUAL_DEV="$DRONECOURSE_DIR/f8t0jrzgb5968yoylwqthhky6m1s5vhr"
WARNING="AUTO-GENERATED FILE - DO NOT EDIT!!"
WARNING_HEADER=true
TPNUMBER=
TPCOUNT=10
STRIP_TAGS=false
STRIP_TAGS_LINE=false
KEEP_DIR=false
NOBACKUP=false

while getopts ":adshgGWCw:t:T:xXkB" opt; do
  case $opt in
    h)
      cat << EOF
Usage: run_dronecourse.sh [OPTIONS]
Options:
    -s bootstrap in student mode (default)
    -d bootstrap in developer/teacher mode, i.e. keep <DRONECOURSE DEV>
    -h show this message
    -B do note create backup
    -W do not add warning header to generated student files
    -w set warning header text
    -t tp# from 0 to 9 specifies which tag to strip.
       Example: -t 2 will strip <DRONECOURSE TP3>, <... TP4>, <... TP5>, ...
    -T set the number of TP (10 by default)
    -x strip all tags, keep content, i.e. \`<...>foo<...>\` becomes \`foo\`
    -X same as -x, but delete the whole line
    -k skip stripping step and keep dronecourse directory
EOF
exit 0
;;
    s)
      DEV_MODE=false
      ;;
    d)
      DEV_MODE=true
      ;;
    k)
      KEEP_DIR=true
      ;;
    t)
      TPNUMBER=$OPTARG
      ;;
    T)
      TPCOUNT=$OPTARG
      ;;
    B)
      NOBACKUP=true
      ;;
    W)
      WARNING_HEADER=false
      ;;
    w)
      WARNING=$OPTARG
      ;;
    x)
      STRIP_TAGS=true
      ;;
    X)
      STRIP_TAGS_LINE=true
      ;;
    \?)
      echo "Invalid option: -$OPTARG" >&2
      exit 1
      ;;
    :)
      echo "Option -$OPTARG requires an argument." >&2
      exit 1
      ;;
  esac
done

# file ACTUAL_DEV must be present to prevent accidentally overwiting students' files
if ( [ -d $DRONECOURSE_DEV_DIR ] && [ -f $DRONECOURSE_ACTUAL_DEV ] && ! $KEEP_DIR); then
  if [ "$NOBACKUP" = false ] && [ -d src/dronecourse  ]; then
    if [ ! -d $DRONECOURSE_BKP_DIR ];then
      mkdir $DRONECOURSE_BKP_DIR
      echo "*" > $DRONECOURSE_BKP_DIR/.gitignore
    fi
    tar -zcf $DRONECOURSE_BKP_DIR/dronecourse-$(date +%s).tar.gz $DRONECOURSE_DIR
  fi
  
  # Remove existing student code
  rm -rf $DRONECOURSE_DIR
  # copy dev files to student folder
  cp -ar $DRONECOURSE_DEV_DIR $DRONECOURSE_DIR
  # fix module names in CMakelists.txt
  find $DRONECOURSE_DIR -type f -exec sed -i -e 's/dronecourse_dev__/dronecourse__/g' {} \;
  # cleanup
  rm $DRONECOURSE_NOT_ACTUAL_DEV

  # remove tags
  if ! $DEV_MODE; then
    find $DRONECOURSE_DIR -type f | xargs -n 1 -P 8 -I % sed -i '/<DRONECOURSE\s*TP[[:alnum:]]\s*>/I,/<\/DRONECOURSE\s*TP[[:alnum:]]\s*>/Id' %
  fi
  if [ ! -z "$TPNUMBER" ]; then
    for (( TPNB=$(($TPNUMBER+1)); TPNB<=$TPCOUNT; TPNB++ )); do
      find $DRONECOURSE_DIR -type f | xargs -n 1 -P 8 -I % sed -i "/<DRONECOURSE\s*TP$TPNB\s*>/I,/<\/DRONECOURSE\s*TP$TPNB\s*>/Id" %
    done
  fi
  if $STRIP_TAGS; then
    find $DRONECOURSE_DIR -type f | xargs -n 1 -P 8 -I % sed -i "s/<\/\{0,1\}DRONECOURSE\s*[[:alnum:]]*\s*>//I" %
  fi
  if $STRIP_TAGS_LINE; then
    find $DRONECOURSE_DIR -type f | xargs -n 1 -P 8 -I % sed -i "/<\/\{0,1\}DRONECOURSE\s*[[:alnum:]]*\s*>/Id" %
  fi
  if $WARNING_HEADER; then
    find $DRONECOURSE_DIR -type f -name '*.c' -o -name '*.cpp' -o -name '*.h' -o -name '*.hpp' | xargs -n 1 -P 8 -I % sed -i "1i /* $WARNING */" %
    find $DRONECOURSE_DIR -type f -name '*.txt' | xargs -n 1 -P 8 -I % sed -i "1i # $WARNING" %
  fi
fi
