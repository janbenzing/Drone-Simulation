#!/bin/bash

# Colors
if test -t 1; then
    ncolors=$(tput colors)
    if test -n "$ncolors" && test $ncolors -ge 8; then
        bold="$(tput bold)"
        underline="$(tput smul)"
        standout="$(tput smso)"
        normal="$(tput sgr0)"
        black="$(tput setaf 0)"
        red="$(tput setaf 1)"
        green="$(tput setaf 2)"
        yellow="$(tput setaf 3)"
        blue="$(tput setaf 4)"
        magenta="$(tput setaf 5)"
        cyan="$(tput setaf 6)"
        white="$(tput setaf 7)"
    fi
fi

if [ -e "reports/latest" ];then
rm "reports/latest"
fi

DIRNAME=$(date +"%Y_%m_%d_%H_%M_%S")
REPDIR="reports/$DIRNAME"
if [ ! -d $REPDIR ]; then
    mkdir -p $REPDIR
fi
ln -s $DIRNAME reports/latest
students="$REPDIR/students"
TMPDIR=$(mktemp -d "${TMPDIR:-/tmp/}$(basename 0).XXXXXXXXXXXX")
fifo="$TMPDIR/fifo"
mkfifo "$fifo"
for file in $(ls dist); do
    file="dist/$file"
    if [ -d src/dronecourse ]; then
        echo "${bold}Info:${normal} cleaning src/dronecourse"
        rm -rf src/dronecourse
    fi
    echo "${bold}Info:${normal} Unpacking '$file'"
    tar -C src -xzf $file dronecourse
    tar -C "$TMPDIR" -xzf $file id.ini
    IDFILE="$TMPDIR/id.ini"
    SCIPER="$(sed -n /^sciper=/s/sciper=//p $IDFILE)"
    MAIL="$(sed -n /^mail=/s/mail=//p $IDFILE)"
    UUID="$(sed -n /^uuid=/s/uuid=//p $IDFILE)"
    REPFILE="$REPDIR/$UUID.txt"
    echo "${bold}Student:${normal}"
    echo "${bold}    SCIPER:${normal} $SCIPER"
    echo "${bold}    MAIL: ${normal}  $MAIL"
    echo "${bold}    UUID: ${normal}  $UUID"
    echo "${bold}Info:${normal} Running simulation"
    echo "$UUID $SCIPER" >>"$students"
    Tools/dronecourse_supervisor.sh "$REPFILE" <"$fifo" \
        | ./run_dronecourse.sh \
        | tee "$fifo"
    if [ "$?" -eq "2" ]; then
        echo "${yellow}Warning:${normal} make encountered an error"
        echo "Error: make encountered an error" >> $REPFILE
    else
        echo "${green}Success:${normal} done"
        echo "Success: make encountered no error" >> $REPFILE
    fi
    echo "${green}Success:${normal} Report written to $REPFILE"
done
echo "${green}Success:${normal} All reports written to $REPDIR"
