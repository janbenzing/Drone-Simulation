/* AUTO-GENERATED FILE - DO NOT EDIT!! */
/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
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
 * @file dronecourse_main.cpp
 * Designed for the LIS-EPFL DroneCourse
 *
 * @author Basil Huber LIS <basil.huber@gmail.com>
 * @author Arthur Gay <arthur.gay@epfl.ch>
 */

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <systemlib/systemlib.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <drivers/drv_hrt.h>
#include <math.h>
#include <fcntl.h>
#include <px4_posix.h>

#include "DronecourseHandler.hpp"

// using std::string;

/** Deamon exit flag. Set this flag to true to interrupt thread. */
static volatile bool thread_should_exit = false;
/** Deamon status flag. */
static volatile bool thread_running = false;
/** Handle of daemon task / thread */
static int deamon_task;

/* Set to `true` is the platform should be initialized at a random position */
static bool random_platform_position = false;

static DronecourseHandler::DcMode dc_mode = DronecourseHandler::DcMode::IDLE;
static bool mode_change = false;
static bool dc_mode_auto = false;
/** Set to `true` when position has been updated */
static bool new_pos = false;
/** Desired drone x position */
static float pos_x = 0;
/** Desired drone y position */
static float pos_y = 0;
/** Desired drone z position */
static float pos_z = 5;
/** Desired drone x position */
static float platform_x = 0;
/** Desired drone y position */
static float platform_y = 0;
/** Desired drone z position */
static float platform_z = 0;
/** Set to `true` when gimbal rotation has been updated */
static bool new_gimbal_rot = false;
/** Set to `true` when gimbal should switch to auto mode */
static bool new_gimbal_auto = false;
/** Desired gimbal pitch */
static float pitch = 0;
/** Desired gimbal yaw */
static float yaw = 0;

/** @cond */
#define DT_US 50000

#define DRONECOURSE_COMMAND(cmd, c, v) \
  if (!strcmp(argv[1], #cmd)) { \
    return dronecourse_command_##cmd(c, v); \
  }
/** @endcond */

/**
 * Deamon management function.
 */
extern "C" __EXPORT int dronecourse_main(int argc, char *argv[]);

/**
 * Mainloop of deamon.
 */
static int dronecourse_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static int usage(const char *reason);

static int usage(const char *reason)
{
  FILE* output = stdout;

  if (reason) {
    output = stderr;
    fprintf(output, "%s\n", reason);
  }

  fprintf(output,
      "usage: dronecourse command [parameters] \n\n"
      "Commands:\n"
      "    start                    start the dronecourse application\n"
      "    stop                     stop the dronecourse application\n"
      "    status                   print whether the dronecourse application is running\n"
      "    pos                      set desired position\n"
      "    platform                 set platform position\n"
      "    waypoint_navigation      start waypoint_navigation\n"
      "    sonar_landing            start landing with sonar\n"
      "    target_following         start target_following\n"
      "    mission                  start complete mission, from waypoint navigation, to sonar landing, to target following\n"
      "    auto                     set mode to auto-continue\n"
      "    gimbal                   set gimbal parameters\n"
      "    arm                      arm the vehicle (starts motors)\n"
      "    disarm                   disarm the vehicle (stops motors)\n"
      "    wait_for_return          waits for return (used in eval mode)\n"
      "    random_platform_position platform will be initialized at a random position\n"
      "    help                     show this message or more information about one command\n"
      "\n"
      "Type `dronecourse help command` to print more information about a "
      "specific command and its parameters.\n");

  // If reason is given, return with error code 1
  if (reason) {
    return 1;
  }

  return 0;
}

/**
 * Print help message.
 */
static int dronecourse_command_help(int argc, char *argv[]);

static int dronecourse_command_help(int argc, char *argv[])
{
  if (argc > 2) {
    if (!strcmp(argv[2], "start")) {
      printf("usage: dronecourse start\n");
      printf("Start the dronecourse application.\n");
      return 0;

    } else if (!strcmp(argv[2], "stop")) {
      printf("usage: dronecourse stop\n");
      printf("Stop the dronecourse application.\n");
      return 0;

    } else if (!strcmp(argv[2], "status")) {
      printf("usage: dronecourse status\n");
      printf("Print whether the dronecourse application is running.\n");
      return 0;

    } else if (!strcmp(argv[2], "pos")) {
      printf("usage: dronecourse pos x y z\n");
      printf("Set desired position.\n"
             "Parameters:\n"
             "    x position along x axis\n"
             "    y position along y axis\n"
             "    z position along z axis\n");
      return 0;

    } else if (!strcmp(argv[2], "platform")) {
      printf("usage: dronecourse platform x y z\n");
      printf("Set platform position.\n"
             "Parameters:\n"
             "    x position along x axis\n"
             "    y position along y axis\n"
             "    z height\n");
      return 0;

    } else if (!strcmp(argv[2], "auto")) {
      printf("usage: dronecourse auto\n");
      printf("Set mode to auto.\n");
      return 0;

    } else if (!strcmp(argv[2], "target_following")) {
      printf("usage: dronecourse target_following\n");
      printf("Set mode to target_following.\n");
      return 0;

    } else if (!strcmp(argv[2], "gimbal")) {
      printf("usage: dronecourse gimbal auto\n"
             "   or: dronecourse gimbal pitch yaw\n");
      printf("Set gimbal mode to automatic or set its desired orientation.\n"
             "Parameters:\n"
             "    pitch desired gimbal pitch\n"
             "    yaw   desired gimbal yaw\n");
      return 0;

    } else if (!strcmp(argv[2], "help")) {
      printf("usage: dronecourse help\n"
             "   or: dronecourse [command]\n");
      printf("Show a list of available commands with a short description.\n"
             "Parameters:\n"
             "    command gives detailed help on the given command\n");
      return 0;
    }
  }

  usage("");

  return 0;
}



/**
 * Start command.
 * usage: dronecourse start
 */
static int dronecourse_command_start(int argc, char *argv[]);

static int dronecourse_command_start(int argc, char *argv[])
{
  if (thread_running) {
    PX4_INFO("already running");
    /* this is not an error */
    return 0;
  }

  thread_should_exit = false;

  deamon_task = px4_task_spawn_cmd("dronecourse",
           SCHED_DEFAULT,
           SCHED_PRIORITY_MAX - 1,
           13000, dronecourse_thread_main, NULL);

  return 0;
}

/**
 * Stop command.
 * usage: dronecourse stop
 */
static int dronecourse_command_stop(int argc, char *argv[]);

static int dronecourse_command_stop(int argc, char *argv[])
{
  if (thread_running) {
    PX4_DEBUG("stop");
    thread_should_exit = true;

  } else {
    PX4_WARN("not started");
  }

  return 0;
}

/**
 * Status command.
 * usage: dronecourse status
 */
static int dronecourse_command_status(int argc, char *argv[]);

static int dronecourse_command_status(int argc, char *argv[])
{
  if (thread_running) {
    PX4_INFO("is running");

    switch (dc_mode) {
    case DronecourseHandler::DcMode::IDLE:
      PX4_INFO("mode : IDLE");
      break;
    
    case DronecourseHandler::DcMode::POS_CTRL:
      PX4_INFO("mode : POS_CTRL");
      break;

    case DronecourseHandler::DcMode::SONAR_LANDING:
      PX4_INFO("mode : SONAR_LANDING");
      break;

    case DronecourseHandler::DcMode::TARGET_FOLLOWING:
      PX4_INFO("mode : TARGET_FOLLOWING");
      break;

    case DronecourseHandler::DcMode::WAYPOINT_NAVIGATION:
      PX4_INFO("mode : WAYPOINT_NAVIGATION");
      break;

    case DronecourseHandler::DcMode::END:
      PX4_INFO("mode : END");
      break;
    }

    if (dc_mode_auto) {
      PX4_INFO("auto : true");
    } else {
      PX4_INFO("auto : false");      
    }

  } else {
    PX4_INFO("not started");
  }

  return 0;
}

/**
 * Position command.
 * usage: dronecourse pos x y z
 */
static int dronecourse_command_pos(int argc, char *argv[]);

static int dronecourse_command_pos(int argc, char *argv[])
{
  if (argc < 5) {
    usage("coordinates missing");
    return 1;
  }

  // setting new position setpoint
  char *end;
  pos_x = strtod(argv[2], &end);
  pos_y = strtod(argv[3], &end);
  pos_z = strtod(argv[4], &end);

  new_pos = true;
  dc_mode = DronecourseHandler::DcMode::POS_CTRL;
  dc_mode_auto = false;
  mode_change = true;
  PX4_INFO("Setting position command to ( %f | %f | %f )",
     (double) pos_x, (double) pos_y, (double) pos_z);
  return 0;
}

/**
 * Platform command.
 * usage: dronecourse platform x y z
 */
static int dronecourse_command_platform(int argc, char *argv[]);

static int dronecourse_command_platform(int argc, char *argv[])
{
  if (argc < 5) {
    usage("coordinates missing");
    return 1;
  }

  const int MAX_CHAR = 10;

  char str1[40];
  char str2[10];
  char str3[10];
  strcpy (str1,"platform -x ");
  strcpy (str2," -y ");
  strcpy (str3," -H ");
  // setting new position setpoint
  char *end;
  platform_x = strtod(argv[2], &end);
  platform_y = strtod(argv[3], &end);
  platform_z = strtod(argv[4], &end);

  strncat (str1, argv[2], MAX_CHAR);
  strncat (str1, str2, MAX_CHAR);
  strncat (str1, argv[3], MAX_CHAR);
  strncat (str1, str3, MAX_CHAR);
  strncat (str1, argv[4], MAX_CHAR);

  if (system(str1)) {
    PX4_WARN("platform error");
  }

  PX4_INFO("Moving platform to ( %f | %f | %f )",
     (double) platform_x, (double) platform_y, (double) platform_z);
  return 0;
}

/**
 * Waypoint Navigation command.
 * usage: dronecourse waypoint_navigation
 */
static int dronecourse_command_waypoint_navigation(int argc, char *argv[]);

static int dronecourse_command_waypoint_navigation(int argc, char *argv[])
{
  dc_mode = DronecourseHandler::DcMode::WAYPOINT_NAVIGATION;
  dc_mode_auto = false;
  mode_change = true;
  PX4_INFO("Switching to WAYPOINT_NAVIGATION mode");
  return 0;

}

/**
 * Sonar landing command.
 * usage: dronecourse sonar_landing
 */
static int dronecourse_command_sonar_landing(int argc, char *argv[]);

static int dronecourse_command_sonar_landing(int argc, char *argv[])
{
  dc_mode = DronecourseHandler::DcMode::SONAR_LANDING;
  dc_mode_auto = false;
  mode_change = true;
  PX4_INFO("Switching to SONAR_LANDING mode");
  return 0;
}

/**
 * Target Following command.
 * usage: dronecourse target_following
 */
static int dronecourse_command_target_following(int argc, char *argv[]);

static int dronecourse_command_target_following(int argc, char *argv[])
{
  dc_mode = DronecourseHandler::DcMode::TARGET_FOLLOWING;
  dc_mode_auto = false;
  mode_change = true;
  PX4_INFO("Switching to TARGET_FOLLOWING mode");
  return 0;

}

/**
 * Wait for return command.
 * usage: dronecourse wait_for_return
 */
static int dronecourse_command_wait_for_return(int argc, char *argv[]); 

 static int dronecourse_command_wait_for_return(int argc, char *argv[])
 {
  random_platform_position = true; 

  PX4_INFO("Waiting for return"); 

  while (dc_mode != DronecourseHandler::DcMode::END){
     usleep(1000000);
  } 

   return 0;
   
} 

/* *
 * Initialise platform at random position
 * usage: dronecourse random_platform_position
*/ 
static int dronecourse_command_random_platform_position(int argc, char *argv[]);

static int dronecourse_command_random_platform_position(int argc, char *argv[])
{
  random_platform_position = true;

  PX4_INFO("Platform will be initialized at a random position");

  return 0;
}

/**
 * Auto command.
 * usage: dronecourse auto
 */
static int dronecourse_command_auto(int argc, char *argv[]);

static int dronecourse_command_auto(int argc, char *argv[])
{
  dc_mode_auto = true;
  PX4_INFO("Switching to auto_mode");
  return 0;

}

/**
 * Mission command.
 * usage: dronecourse mission
 */
static int dronecourse_command_mission(int argc, char *argv[]);

static int dronecourse_command_mission(int argc, char *argv[])
{
  dc_mode = DronecourseHandler::DcMode::WAYPOINT_NAVIGATION;  
  dc_mode_auto = true;
  mode_change = true;
  PX4_INFO("Switching to WAYPOINT_NAVIGATION (automatically)");
  return 0;

}

/**
 * Gimbal command.
 * usage: dronecourse gimbal
 */
static int dronecourse_command_gimbal(int argc, char *argv[]);

static int dronecourse_command_gimbal(int argc, char *argv[])
{
  if (argc < 3) {
    usage("gimbal: arguments missing");
    return 1;
  }

  if (!strcmp(argv[2], "auto")) {
    PX4_INFO("Setting gimbal to automatic");
    new_gimbal_auto = true;
    new_gimbal_rot = false;
    return 0;

  } else if (argc >= 4) {
    char *end;
    pitch = strtod(argv[2], &end);
    yaw = strtod(argv[3], &end);

    new_gimbal_rot = true;
    new_gimbal_auto = false;
    PX4_INFO("Setting gimbal command to pitch: %f    yaw: %f )",
       (double) pitch, (double) yaw);
    return 0;

  } else {
    usage("gimbal: coordinates missing");
    return 0;
  }
}

/**
 * Arm command.
 * usage: dronecourse arm
 */
static int dronecourse_command_arm(int argc, char *argv[]);

static int dronecourse_command_arm(int argc, char *argv[])
{
  BaseCtrl::arm();
  return 0;
}


/**
 * Disarm command.
 * usage: dronecourse disarm
 */
static int dronecourse_command_disarm(int argc, char *argv[]);

static int dronecourse_command_disarm(int argc, char *argv[])
{
  BaseCtrl::disarm();
  return 0;
}

/**
 * The deamon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to task_create().
 */
int dronecourse_main(int argc, char *argv[])
{

  // Print usage in case no command (such as help) is given
  if (argc < 2) {
     usage("");
     return 0;
  } 

  DRONECOURSE_COMMAND(start, argc, argv);
  DRONECOURSE_COMMAND(stop, argc, argv);
  DRONECOURSE_COMMAND(status, argc, argv);
  DRONECOURSE_COMMAND(pos, argc, argv);
  DRONECOURSE_COMMAND(platform, argc, argv);
  DRONECOURSE_COMMAND(waypoint_navigation, argc, argv);
  DRONECOURSE_COMMAND(sonar_landing, argc, argv);
  DRONECOURSE_COMMAND(target_following, argc, argv);
  DRONECOURSE_COMMAND(auto, argc, argv);
  DRONECOURSE_COMMAND(mission, argc, argv);
  DRONECOURSE_COMMAND(wait_for_return, argc, argv);
  DRONECOURSE_COMMAND(random_platform_position,argc, argv);
  DRONECOURSE_COMMAND(gimbal, argc, argv);
  DRONECOURSE_COMMAND(help, argc, argv);
  DRONECOURSE_COMMAND(arm, argc, argv);
  DRONECOURSE_COMMAND(disarm, argc, argv);

  usage("unrecognized command");
  return 1;
}


int dronecourse_thread_main(int argc, char *argv[])
{
  PX4_DEBUG("starting");

  if (random_platform_position) {
    if (system("platform -a -H 0")) {
      PX4_WARN("platform error");
    }
  }

  thread_running = true;

  DronecourseHandler handler;

  while (!thread_should_exit) {
    if (dc_mode == DronecourseHandler::DcMode::POS_CTRL && new_pos) {
      handler.set_position_command(pos_x, pos_y, pos_z);
    }

    if (new_gimbal_rot) {
      handler.gimbal().set_command(pitch, yaw);
      new_gimbal_rot = false;

    } else if (new_gimbal_auto) {
      handler.gimbal().setAutomatic();
      new_gimbal_auto = false;
    }

    if (mode_change) {
      handler.set_mode(dc_mode);
      mode_change = false;
      if (dc_mode == DronecourseHandler::DcMode::WAYPOINT_NAVIGATION && dc_mode_auto){
        handler.start_timer_task1();
      }
    }

    if (dc_mode_auto != handler.get_auto_mode()) {
      handler.set_auto_mode(dc_mode_auto);
    }

    dc_mode = handler.get_mode();

    handler.update();
    usleep(DT_US);
  }

  PX4_DEBUG("exiting.");

  thread_running = false;

  return 0;
}
