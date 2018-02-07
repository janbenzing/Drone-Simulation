#!/usr/bin/env roundup

describe "Show Waypoints"

it_displays_usage() {
  usage=$(./show-waypoints.sh -h | head -n 1)
  test "$usage" = "Usage: show-waypoints.sh [OPTIONS]"
}

it_shows_header_file() {
	tmpd=$(mktemp -d)
	tmp=$(mktemp)
	tmphpp=$tmpd/waypoints.hpp
	tee $tmp > /dev/null <<'EOF'
<?xml version="1.0" ?>
<sdf version="1.5">
  <world name="default">
    <include>
      <uri>model://waypoint</uri>
      <pose>1 2 3 0 0 0</pose>
      <plugin name='Waypoint0' filename='libgazebo_waypoint_plugin.so'>
      </plugin>
    </include>
    <include>
      <uri>model://waypoint</uri>
      <pose>4 5 6 0 0 0</pose>
      <plugin name='Waypoint1' filename='libgazebo_waypoint_plugin.so'>
      </plugin>
    </include>
    <include>
      <uri>model://waypoint</uri>
      <pose>7 8 9 0 0 0</pose>
      <plugin name='Waypoint2' filename='libgazebo_waypoint_plugin.so'>
      </plugin>
    </include>
  </world>
</sdf>
EOF
	status=$(set +e; ./show-waypoints.sh -w $tmp -c > $tmphpp; echo $?)
	test 0 -eq $status

	# do some grep search
	cat $tmphpp
	grep "const int WAYPOINT_COUNT = 3;" $tmphpp
	test $? -eq 0
	grep "*waypoint = matrix::Vector3f(2, 1, -3);" $tmphpp
	test $? -eq 0
	grep "*waypoint = matrix::Vector3f(5, 4, -6);" $tmphpp
	test $? -eq 0
	grep "*waypoint = matrix::Vector3f(8, 7, -9);" $tmphpp
	test $? -eq 0
}

it_shows_csv() {
	tmp=$(mktemp)
	tee $tmp > /dev/null <<'EOF'
<?xml version="1.0" ?>
<sdf version="1.5">
  <world name="default">
    <include>
      <uri>model://platform</uri>
    </include>
    <include>
      <uri>model://waypoint</uri>
      <pose>1 2 3 0 0 0</pose>
      <plugin name='Waypoint0' filename='libgazebo_waypoint_plugin.so'>
      </plugin>
    </include>
    <include>
      <uri>model://waypoint</uri>
      <pose>4 5 6 0 0 0</pose>
      <plugin name='Waypoint1' filename='libgazebo_waypoint_plugin.so'>
      </plugin>
    </include>
    <include>
      <uri>model://waypoint</uri>
      <pose>7 8 9 0 0 0</pose>
      <plugin name='Waypoint2' filename='libgazebo_waypoint_plugin.so'>
      </plugin>
    </include>
    <physics name='default_physics' default='0' type='ode'/>
  </world>
</sdf>
EOF
	csv=$(./show-waypoints.sh -w $tmp 2>&1 | tr '\n' ' ')
	test "$csv" = "0,2,1,-3 1,5,4,-6 2,8,7,-9 "
}

it_shows_no_csv_if_no_waypoints() {
	tmp=$(mktemp)
	tee $tmp > /dev/null <<'EOF'
<?xml version="1.0" ?>
<sdf version="1.5">
  <world name="default">
    <include>
      <uri>model://platform</uri>
    </include>
    <physics name='default_physics' default='0' type='ode'/>
  </world>
</sdf>
EOF
	csv=$(./show-waypoints.sh -w $tmp | tr '\n' ' ')
	test "$csv" = " "
}

it_fails_if_invalid_xml() {
	tmp=$(mktemp)
	tee $tmp > /dev/null <<'EOF'
<?xml version="1.0" ?>
<sdfx version="1.5">
  lt_physics' default='0' type='ode'/>
  </world>
</sdf>
EOF
	status=$(set +e ; ./show-waypoints.sh -w $tmp ; echo $?)
	test 0 -ne $status
}

it_fails_if_malformed_waypoint_name() {
	tmp=$(mktemp)
	tee $tmp > /dev/null <<'EOF'
<?xml version="1.0" ?>
<sdf version="1.5">
  <world name="default">
    <include>
      <uri>model://waypoint</uri>
      <pose>7 8 9 0 0 0</pose>
      <plugin name='my-waypoint' filename='libgazebo_waypoint_plugin.so'>
      </plugin>
    </include>
  </world>
</sdf>
EOF
    status=$(set +e ; ./show-waypoints.sh -w $tmp ; echo $?)
	test 0 -ne $status
}

it_fails_if_malformed_pose() {
	tmp=$(mktemp)
	tee $tmp > /dev/null <<'EOF'
<?xml version="1.0" ?>
<sdf version="1.5">
  <world name="default">
    <include>
      <uri>model://waypoint</uri>
      <pose>x7 8aa z8d5 0 0 0</pose>
      <plugin name='Waypoint0' filename='libgazebo_waypoint_plugin.so'>
      </plugin>
    </include>
  </world>
</sdf>
EOF
    status=$(set +e ; ./show-waypoints.sh -w $tmp ; echo $?)
	test 0 -ne $status
}

it_fails_if_missing_waypoint_name() {
	tmp=$(mktemp)
	tee $tmp > /dev/null <<'EOF'
<?xml version="1.0" ?>
<sdf version="1.5">
  <world name="default">
    <include>
      <uri>model://waypoint</uri>
      <pose>7 8 9 0 0 0</pose>
      <plugin name='' filename='libgazebo_waypoint_plugin.so'>
      </plugin>
    </include>
  </world>
</sdf>
EOF
    status=$(set +e ; ./show-waypoints.sh -w $tmp ; echo $?)
	test 0 -ne $status
}

it_fails_if_missing_pose() {
	tmp=$(mktemp)
	tee $tmp > /dev/null <<'EOF'
<?xml version="1.0" ?>
<sdf version="1.5">
  <world name="default">
    <include>
      <uri>model://waypoint</uri>
      <pose></pose>
      <plugin name='Waypoint0' filename='libgazebo_waypoint_plugin.so'>
      </plugin>
    </include>
  </world>
</sdf>
EOF
    status=$(set +e ; ./show-waypoints.sh -w $tmp ; echo $?)
	test 0 -ne $status
}

it_warns_if_starting_index_is_not_0() {
	tmp=$(mktemp)
	tee $tmp > /dev/null <<'EOF'
<?xml version="1.0" ?>
<sdf version="1.5">
  <world name="default">
    <include>
      <uri>model://waypoint</uri>
      <pose>0 0 0 0 0 0</pose>
      <plugin name='Waypoint1' filename='libgazebo_waypoint_plugin.so'>
      </plugin>
    </include>
  </world>
</sdf>
EOF
    csv=$(./show-waypoints.sh -w $tmp 2>&1 | head -n 1)
	test "$csv" = "Warning: Waypoint numerotation should start at 0, got '1'."
}

it_warns_if_not_uniq() {
	tmp=$(mktemp)
	tee $tmp > /dev/null <<'EOF'
<?xml version="1.0" ?>
<sdf version="1.5">
  <world name="default">
    <include>
      <uri>model://waypoint</uri>
      <pose>0 0 0 0 0 0</pose>
      <plugin name='Waypoint0' filename='libgazebo_waypoint_plugin.so'>
      </plugin>
    </include>
    <include>
      <uri>model://waypoint</uri>
      <pose>0 0 0 0 0 0</pose>
      <plugin name='Waypoint0' filename='libgazebo_waypoint_plugin.so'>
      </plugin>
    </include>
  </world>
</sdf>
EOF
    csv=$(./show-waypoints.sh -w $tmp 2>&1 | head -n 1)
	test "$csv" = "Warning: Waypoints should have unique names 'WaypointN' where N is a number."
}

it_warns_if_not_in_sequence() {
	tmp=$(mktemp)
	tee $tmp > /dev/null <<'EOF'
<?xml version="1.0" ?>
<sdf version="1.5">
  <world name="default">
    <include>
      <uri>model://waypoint</uri>
      <pose>0 0 0 0 0 0</pose>
      <plugin name='Waypoint0' filename='libgazebo_waypoint_plugin.so'>
      </plugin>
    </include>
    <include>
      <uri>model://waypoint</uri>
      <pose>0 0 0 0 0 0</pose>
      <plugin name='Waypoint3' filename='libgazebo_waypoint_plugin.so'>
      </plugin>
    </include>
  </world>
</sdf>
EOF
    csv=$(./show-waypoints.sh -w $tmp 2>&1 | head -n 1)
	test "$csv" = "Warning: Missing waypoints from index 1 to index 2."
}

it_fails_if_file_is_not_found() {
	tmp=$(mktemp)
	rm $tmp
    status=$(set +e ; ./show-waypoints.sh -w $tmp ; echo $?)
	test 0 -ne $status
}

it_warns_if_not_using_w() {
    warning=$(./show-waypoints.sh 2>&1 | head -n 1)
	test "$warning" = "Warning: Consider using the -w option."
}
