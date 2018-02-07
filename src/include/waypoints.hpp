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
		switch (index % 3) {
		case 0:
			*waypoint = matrix::Vector3f(35, 60, -8);
			break;

		case 1:
			*waypoint = matrix::Vector3f(0, 80, -15);
			break;

		case 2:
			*waypoint = matrix::Vector3f(-35, 55, -13);
			break;

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
	const int WAYPOINT_COUNT = 3;
};
