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
OUTPUT_CSV=1

# Script path
SCRIPTPATH=$( cd $(dirname $0) ; pwd -P )
WORLDFILE="$SCRIPTPATH/sitl_gazebo/worlds/dronecourse.world"

while getopts ":w:sch" opt; do
  case $opt in
    h)
      cat << EOF
Usage: show-waypoints.sh [OPTIONS]
Print waypoints from dronecourse.world file in comma separated value
format or WaypointNavigation class format.

Options:
    -h show this message
    -w path to dronecourse world file, default to $WORLDFILE
    -s output waypoints in csv form (default)
    -c output waypoints as a WaypointNavigator class file
EOF
    exit 0
   ;;
   s)
    OUTPUT_CSV=1
    ;;
   c)
    OUTPUT_CSV=0
    ;;
   w)
    NO_WORLD_FILE_WARNING=1
    WORLDFILE=$OPTARG
    ;;
   \?)
      echo "${red}Error:${normal} invalid option -$OPTARG" >&2
      echo >&2 "${yellow}Hint:${normal} use option -h to show help"
      exit 1
    ;;
    :)
      echo "${red}Error:${normal} option -$OPTARG requires an argument." >&2
      echo >&2 "${yellow}Hint:${normal} use option -h to show help"
      exit 1
    ;;
  esac
done

if [ "$NO_WORLD_FILE_WARNING" != "1" ]; then
	echo >&2 "${yellow}Warning:${normal} Consider using the -w option."
fi

# Find world file
if [ ! -f "$WORLDFILE" ]; then
    echo >&2 "${red}Error:${normal} world file $WORLDFILE does not exist"
    exit 1
fi

# Check that xsltproc is installed
hash xsltproc 2>/dev/null || {
    echo >&2 "${red}Error:${normal} xsltproc must be installed to run this command."
    exit 1
}

# Create stylesheet
# XSL is used here for validation but it would be better to use XSD
STYLESHEET=$(mktemp)
tee $STYLESHEET > /dev/null <<'EOF'
<?xml version="1.0"?>
<xsl:stylesheet xmlns:xsl="http://www.w3.org/1999/XSL/Transform" version="1.0"
                xmlns:str="http://exslt.org/strings"
                extension-element-prefixes="str">
  <xsl:output method="text"/>
  <xsl:template match="/sdf/world">
	<xsl:for-each select="include[plugin/@filename='libgazebo_waypoint_plugin.so']">
	<xsl:sort select="number(substring(plugin/@name, string-length('Waypoint')+1))" data-type="number"/>
	<xsl:variable name="waypoint" select="substring(plugin/@name, string-length('Waypoint')+1)"/>
	<xsl:variable name="waypointn" select="number($waypoint)"/>
    <xsl:if test="$waypointn!=$waypointn">
		<xsl:message terminate="yes">
Error parsing waypoints: <xsl:value-of select="$waypoint"/> is not a valid waypoint name!
It should match pattern 'WaypointN' where N is a number.
		</xsl:message>
    </xsl:if>
    <xsl:variable name="coord" select="str:tokenize(pose/text(), ' ')"/>
    <xsl:variable name="coord1" select="number($coord[1])"/>
    <xsl:variable name="coord2" select="number($coord[2])"/>
    <xsl:variable name="coord3" select="number($coord[3])"/>
    <xsl:if test="$coord1!=$coord1">
		<xsl:message terminate="yes">
Error parsing waypoint #<xsl:value-of select="$waypointn"/>:
<xsl:value-of select="$coord[1]"/> is not a number!
		</xsl:message>
    </xsl:if>
    <xsl:if test="$coord2!=$coord2">
		<xsl:message terminate="yes">
Error parsing waypoint #<xsl:value-of select="$waypointn"/>:
<xsl:value-of select="$coord[2]"/> is not a number!
		</xsl:message>
    </xsl:if>
    <xsl:if test="$coord3!=$coord3">
		<xsl:message terminate="yes">
Error parsing waypoint #<xsl:value-of select="$waypointn"/>:
<xsl:value-of select="$coord[3]"/> is not a number!
		</xsl:message>
    </xsl:if>
    <xsl:value-of select="$waypointn"/>
    <xsl:text>,</xsl:text>
    <xsl:value-of select="$coord2"/>
    <xsl:text>,</xsl:text>
    <xsl:value-of select="$coord1"/>
    <xsl:text>,</xsl:text>
    <xsl:text>-</xsl:text>
    <xsl:value-of select="$coord3"/>
    <xsl:text>&#10;</xsl:text>
	</xsl:for-each>
  </xsl:template>
  <xsl:template match="text()"/>
</xsl:stylesheet>
EOF

# Use stylesheet to extract waypoint from world file
WAYPOINTS=$(xsltproc $STYLESHEET $WORLDFILE)
ERROR_CODE=$?
if [ "$ERROR_CODE" -ne "0" ]; then
    echo >&2 "${red}Error:${normal} xsltproc exited with error code ${ERROR_CODE}."
    HINT=$(man xsltproc | sed -e "/^DIAGNOSTICS/,/SEE ALSO/! d;s/^ *//g;/$ERROR_CODE/,+1! d" | tail -n 1)
    echo >&2 "${yellow}Hint:${normal} Open man page for more information about this error:"
    echo >&2 "${yellow}Hint:${normal} ${bold}${HINT}${normal}."
    exit 1;
fi
if [ "$WAYPOINTS" == "" ]; then
	WAYPOINT_COUNT=0
else
	WAYPOINT_COUNT=$(wc -l<<<"$WAYPOINTS")
fi
# Delete stylesheet
rm $STYLESHEET

# Show warnings
if [ "$WAYPOINT_COUNT" -gt 0 ]; then
	FIRST_WAYPOINT_INDEX=$(echo "$WAYPOINTS" | cut -f 1 -d ',' | head -n 1)
	if [ "$FIRST_WAYPOINT_INDEX" != "0" ]; then
		echo >&2 "${yellow}Warning:${normal} Waypoint numerotation should start at 0, got '$FIRST_WAYPOINT_INDEX'."
	fi
	DUPLICATES=$(echo "$WAYPOINTS" | cut -f 1 -d ',' | uniq -d | wc -l)
	if [ "$DUPLICATES" -gt 0 ]; then
		echo >&2 "${yellow}Warning:${normal} Waypoints should have unique names 'WaypointN' where N is a number."
	fi
	while read n ; do
		if (( n != 0 && n != previous + 1 )) ; then
			echo >&2 "${yellow}Warning:${normal} Missing waypoints from index $(( $previous + 1 )) to index $(( $n - 1 ))."
		fi
		previous=$n
	done<<<"$(echo "$WAYPOINTS" | cut -f 1 -d ',')"
fi

# Output CSV
if [ "$OUTPUT_CSV" -eq "1" ]; then echo "$WAYPOINTS"; else

# Output HPP
cat<<EOF
/* AUTO-GENERATED FILE - DO NOT EDIT!! */

#pragma once

class WaypointNavigator
{
public:
	const int INVALID_WAYPOINT = -1;
	/**
	 * Fetch waypoint at the given index.
	 *
	 * @param index    the waypoint's index.
	 * @param waypoint pointer to the vector receiving the data.
	 * @return index on success or INVALID_WAYPOINT on failure.
	 */
	virtual int waypoint_copy(int index, matrix::Vector3f *waypoint)
	{
		switch (index % $WAYPOINT_COUNT) {
EOF

while read -r wp; do
cat<<EOF
		case $(echo $wp | cut -f 1 -d ','):
			*waypoint = matrix::Vector3f($(echo $wp | cut -f 2,3,4 -d ',' | sed "s/,/, /g"));
			break;

EOF
done<<<"$WAYPOINTS"

cat<<EOF
		default:
			return INVALID_WAYPOINT;
		}

		return index;
	};
	/**
	 * Fetch the number of waypoints that are available.
	 *
	 * @return number of available waypoints.
	 */
	virtual int waypoint_count()
	{
		return WAYPOINT_COUNT;
	};
private:
	const int WAYPOINT_COUNT = $WAYPOINT_COUNT;
};
EOF
fi
