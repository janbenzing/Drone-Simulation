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

IDFILE="$HOME/.dronecourse_id"

create_identity() {
    yn="n"
    while [ "$yn" == "n" ]; do
        read -p "Please enter your sciper number: " SCIPER
        if [ "$SCIPER" -eq "$SCIPER" ] 2>/dev/null && [ "$SCIPER" -ge 100000 ] && [ "$SCIPER" -le 999999 ]; then
            read -p "Confirm sciper '$SCIPER' (Y/n)? " yn
        else
            echo "${red}Error:${normal} sciper '$SCIPER' invalid"
        fi
    done

    echo "${bold}Info:${normal} querying server for sciper '$SCIPER'"

    QUERY=$(curl -sSI  https://search.epfl.ch/psearch.action?q=$SCIPER)
    CODE=$(echo "$QUERY" | head -n 1 | tr -d '\n\r')

    if [ "$CODE" = "HTTP/1.1 302 Found" ]; then
        echo "${green}Success:${normal} sciper '$SCIPER' found"
    else
        echo "${red}Error:${normal} sciper '$SCIPER' not found. Aborting."
        exit 1;
    fi

    echo "${bold}Info:${normal} querying server for email for sciper '$SCIPER'"

    LOCATION=$(echo "$QUERY" | sed -n 4p | cut -c 11- | tr -d '\n\r')
    EMAIL=$(curl -sSL $LOCATION | sed -n "s/.*msgto('\(.*\)','\(.*\)').*/\1@\2/p")

    if [ -z "$EMAIL" ]; then
        echo "${red}Error:${normal} no email found."
        exit 1;
    else
        echo "${green}Success:${normal} email '$EMAIL' found for sciper '$SCIPER'"
    fi

    echo "${bold}Identity:${normal}"
    echo "${bold}    SCIPER:${normal} $SCIPER"
    echo "${bold}    EMAIL: ${normal} $EMAIL"

    read -p "Confirm identity (Y/n)? " yn
    if [ "$yn" == "n" ]; then
        echo "Aborting."
        exit 1
    fi

    echo "$SCIPER" > "$IDFILE"
    echo "$EMAIL" >> "$IDFILE"
    echo "${green}Success:${normal} identity file successfully written to '$IDFILE'"
}

if [ -f "$IDFILE" ]; then
    echo "Identity found in '$IDFILE'."
    echo "${bold}Identity:${normal}"
    echo "${bold}    SCIPER:${normal} $(head -n 1 $IDFILE)"
    echo "${bold}    EMAIL: ${normal} $(tail -n 1 $IDFILE)"
    read -p "Use this identity (Y/n)? " yn
    if [ "$yn" == "n" ]; then
        create_identity
    fi
else
    create_identity
fi

SCIPER=$(head -n 1 $IDFILE)
EMAIL=$(tail -n 1 $IDFILE)
PACKAGENAME="dronecourse.$SCIPER.$(date +"%Y_%m_%d_%H_%M_%S").tar"

echo "${bold}Info:${normal} preparing id file"
TMPDIR=$(mktemp -d "${TMPDIR:-/tmp/}$(basename 0).XXXXXXXXXXXX")
TMPIDF="$TMPDIR/id.ini"
echo "[SYSTEM]" > $TMPIDF
echo "date=$(date --rfc-3339=second)" >> $TMPIDF
echo "timestamp=$(date +%s)" >> $TMPIDF
echo "kernelname=$(uname -s)" >> $TMPIDF
echo "nodename=$(uname -n)" >> $TMPIDF
echo "kernelrelease=$(uname -r)" >> $TMPIDF
echo "kernelversion=$(uname -v)" >> $TMPIDF
echo "machine=$(uname -m)" >> $TMPIDF
echo "processor=$(uname -p)" >> $TMPIDF
echo "hardwareplatform=$(uname -i)" >> $TMPIDF
echo "operatingsystem=$(uname -o)" >> $TMPIDF
echo "dcu=$DRONECOURSEU" >> $TMPIDF

echo "[STUDENT]" >> $TMPIDF
echo "sciper=$SCIPER" >> $TMPIDF
echo "mail=$EMAIL" >> $TMPIDF

echo "[MISC]" >> $TMPIDF
echo "uuid=$(uuidgen)" >> $TMPIDF

echo "${bold}Info:${normal} creating archive"
mkdir dist 2>/dev/null
tar -cf dist/$PACKAGENAME -C src dronecourse
tar -rf dist/$PACKAGENAME -C $(dirname $TMPIDF) $(basename $TMPIDF)
gzip dist/$PACKAGENAME
echo "${green}Success:${normal} archive written to $PACKAGENAME"
echo "${yellow}${bold}IMPORTANT:${normal} do not rename or edit the generated archives."
