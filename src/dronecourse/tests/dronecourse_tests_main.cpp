/**
 * @file dronecourse_tests_main.cpp
 *
 * @author Arthur Gay <arthur.gay@epfl.ch>
 */

#include <px4_log.h>
#include <stdio.h>
#include <string.h>
#include "position_ctrl_test.h"
#include "trajectory_ctrl_test.h"
#include "target_detection_test.h"
#include "kalman_test.h"
#include "gimbal_ctrl_test.h"

extern "C" __EXPORT int dronecourse_tests_main(int argc, char *argv[]);

static bool help();
static bool all();
static bool check_tests();

/** Tests list */
static const struct {
	/** Name of the test to be used in the shell */
	const char *name;
	/** Function to be run, returns true on success and false on failure */
	bool (*fn)(void);
} dronecourse_tests[] = {
	{"help", help},
	{"all", all},
	/* Add your tests here */
	{"position_ctrl", position_ctrl_test},
	{"trajectory_ctrl", trajectory_ctrl_test},
	{"target_detection", target_detection_test},
	{"kalman", kalman_test},
	{"gimbal_ctrl", gimbal_ctrl_test},
	{NULL, NULL}
};

/** Check integrity of the tests list.
 *
 * This function checks that the first and second tests are respectively
 * named 'help' and 'all', that there is no duplicate test name, and
 * that there is no NULL function pointers.
 *
 * @return true if the tests list is valid, false otherwise.
 */
bool check_tests()
{
	if(strcmp(dronecourse_tests[0].name, "help")) {
		PX4_ERR("Test at index 0 must be named 'help'.");
		return false;
	}
	if(strcmp(dronecourse_tests[1].name, "all")) {
		PX4_ERR("Test at index 1 must be named 'all'.");
		return false;
	}
	for(int i = 0; dronecourse_tests[i].name != NULL; i++) {
		for(int j = i + 1; dronecourse_tests[j].name != NULL; j++) {
			if(!strcmp(dronecourse_tests[i].name, dronecourse_tests[j].name)) {
				PX4_ERR("Duplicate name '%s' for tests at index %d and %d.", dronecourse_tests[i].name, i, j);
				return false;
			}
		}
		if(NULL == dronecourse_tests[i].fn) {
			PX4_ERR("Test '%s' at index %d is null.", dronecourse_tests[i].name, i);
			return false;
		}
	}

	return true;
}

/** Show a list of all available tests. */
bool help()
{
	PX4_INFO("Available tests:");
	for(int i = 0; dronecourse_tests[i].name != NULL; i++) {
		PX4_INFO("  %s", dronecourse_tests[i].name);
	}

	return true;
}

/** Run all tests and display a summary. */
bool all()
{
	int passcount = 0, testcount = 0;
	for(int i = 2; dronecourse_tests[i].name != NULL; i++) {
		PX4_INFO("SUIT %s", dronecourse_tests[i].name);
		if(dronecourse_tests[i].fn()) {
			passcount++;
		}
		testcount++;
	}

	PX4_INFO("%d/%d tests passed.", passcount, testcount);
	return passcount == testcount;
}

/** Executes dronecourse tests. */
int dronecourse_tests_main(int argc, char *argv[])
{
	if(!check_tests()) {
		return 1;
	}
	if (argc < 2) {
		PX4_ERR("Missing test name - '%s help' for a list of tests", argv[0]);
		return 1;
	} else if (argc > 2) {
		PX4_ERR("Too many arguments - '%s all' to run all tests", argv[0]);
		return 1;
	}

	for(int i = 0; dronecourse_tests[i].name != NULL; i++) {
		if(!strcmp(dronecourse_tests[i].name, argv[1])) {
			if(dronecourse_tests[i].fn()) {
				PX4_INFO("%s PASSED", dronecourse_tests[i].name);
				return 0;
			} else {
				PX4_INFO("%s FAILED", dronecourse_tests[i].name);
				return -1;
			}
		}
	}
	PX4_ERR("%s: no test called '%s' - '%s help' for a list of tests",
		argv[0],
		argv[1],
		argv[0]);

	return 1;
}
