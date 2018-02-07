/****************************************************************************
*
*   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* 1. Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in
*    the documentation and/or other materials provided with the
*    distribution.
* 3. Neither the name PX4 nor the names of its contributors may be
*    used to endorse or promote products derived from this software
*    without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
* OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
* AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
****************************************************************************/

/**
 * @file offboard_controller.cpp
 * daemon application to relay velocity commands to mc_pos_control
 *
 * @author Julien Lecoeur <julien.lecoeur@epfl.ch>
 * @author Basil Huber <basil.huber@gmail.com
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <px4_config.h>
#include <px4_tasks.h>

#include <systemlib/systemlib.h>
#include <systemlib/err.h>

#include <drivers/drv_hrt.h>

#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/dronecourse_velocity_setpoint.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/position_setpoint.h>

static bool thread_should_exit = false;		/**< daemon exit flag */
static bool thread_running = false;		/**< daemon status flag */
static int daemon_task;				/**< Handle of daemon task / thread */

/**
 * daemon management function.
 */
extern "C" __EXPORT int offboard_controller_main(int argc, char *argv[]);

/**
 * Mainloop of daemon.
 */
int offboard_controller_tread(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

static void
usage(const char *reason)
{
	if (reason) {
		warnx("%s\n", reason);
	}

	warnx("usage: offboard_controller {start|stop|status} [-p <additional params>]\n\n");
}

/**
 * The daemon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to task_create().
 */
int offboard_controller_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage("missing command");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			warnx("daemon already running\n");
			/* this is not an error */
			return 0;
		}

		thread_should_exit = false;
		daemon_task = px4_task_spawn_cmd("offboard_controller",
						 SCHED_DEFAULT,
						 SCHED_PRIORITY_DEFAULT,
						 2000,
						 offboard_controller_tread,
						 (argv) ? (char *const *)&argv[2] : (char *const *)NULL);
		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			warnx("\trunning\n");

		} else {
			warnx("\tnot started\n");
		}

		return 0;
	}

	usage("unrecognized command");
	return 1;
}

int offboard_controller_tread(int argc, char *argv[])
{

	warnx("[daemon] starting\n");


	// ----------------
	// Subscriptions
	// ----------------
	// control mode
	int control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	int velocity_setpoint_sub = orb_subscribe(ORB_ID(dronecourse_velocity_setpoint));
	struct vehicle_control_mode_s control_mode;
	struct dronecourse_velocity_setpoint_s vel_sp;


	// ----------------
	// Adverts
	// ----------------
	// Offboard control
	orb_advert_t _offboard_control_mode_pub = nullptr;
	// advertise position setpoint triplet topic
	orb_advert_t pos_sp_triplet_pub = nullptr;

	// ----------------
	// Main loop
	// ----------------
	thread_running = true;

	while (!thread_should_exit) {
		//warnx("Hello daemon!\n");
		usleep(10000);

		// --------------
		// Off board mode
		// --------------
		struct offboard_control_mode_s offboard_control_mode = {};
		offboard_control_mode.ignore_position = true;
		offboard_control_mode.ignore_alt_hold = false;
		offboard_control_mode.ignore_velocity = false;
		offboard_control_mode.ignore_acceleration_force = true;
		offboard_control_mode.ignore_attitude = false;
		offboard_control_mode.ignore_bodyrate = true;
		offboard_control_mode.timestamp = hrt_absolute_time();

		if (_offboard_control_mode_pub == nullptr) {
			_offboard_control_mode_pub = orb_advertise(ORB_ID(offboard_control_mode), &offboard_control_mode);

		} else {
			orb_publish(ORB_ID(offboard_control_mode), _offboard_control_mode_pub, &offboard_control_mode);
		}


		// --------------------------------------
		// Check if current mode is offboard mode
		// --------------------------------------
		bool updated;
		orb_check(control_mode_sub, &updated);

		if (updated) {
			orb_copy(ORB_ID(vehicle_control_mode), control_mode_sub, &control_mode);
		}

		bool setpoint_received;
		orb_check(velocity_setpoint_sub, &setpoint_received);


		if (control_mode.flag_control_offboard_enabled && setpoint_received) {

			orb_copy(ORB_ID(dronecourse_velocity_setpoint), velocity_setpoint_sub, &vel_sp);

			// -----------------
			// Position setpoint
			// -----------------
			struct position_setpoint_triplet_s pos_sp_triplet = {};
			pos_sp_triplet.timestamp = hrt_absolute_time();

			// deactivate previous and next waypoint to avoid interference
			pos_sp_triplet.previous.valid = false;
			pos_sp_triplet.next.valid = false; // the next setpoint is never valid
			pos_sp_triplet.current.valid = true;

			pos_sp_triplet.current.type = position_setpoint_s::SETPOINT_TYPE_FOLLOW_TARGET;

			// set velocity and yaw
			pos_sp_triplet.current.velocity_valid = true;
			pos_sp_triplet.current.velocity_frame = position_setpoint_s::VELOCITY_FRAME_LOCAL_NED;
			pos_sp_triplet.current.vx = vel_sp.vx;
			pos_sp_triplet.current.vy = vel_sp.vy;
			pos_sp_triplet.current.vz = vel_sp.vz;
			pos_sp_triplet.current.yaw_valid = vel_sp.yaw_valid;
			pos_sp_triplet.current.yaw = vel_sp.yaw;

			// set position (even if not used, for mavlink)
			pos_sp_triplet.current.position_valid = false;

			// deactivate all other controls
			pos_sp_triplet.current.acceleration_valid = false;
			pos_sp_triplet.current.yawspeed_valid = false;
			pos_sp_triplet.current.alt_valid = false;

			// ----------------------------------
			// publish position_setpoint_triplet
			// ----------------------------------
			if (pos_sp_triplet_pub == nullptr) {
				pos_sp_triplet_pub = orb_advertise(ORB_ID(position_setpoint_triplet),
								   &pos_sp_triplet);

			} else {
				orb_publish(ORB_ID(position_setpoint_triplet), pos_sp_triplet_pub,
					    &pos_sp_triplet);
			}
		}
	}

	warnx("[daemon] exiting.\n");

	thread_running = false;

	return 0;
}
