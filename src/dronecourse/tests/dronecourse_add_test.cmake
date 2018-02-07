function(dronecourse_add_test test_name)
	# generate init script
	set(generated_init_script_name ${test_name}_generated)
	set(generated_init_script ${PX4_SOURCE_DIR}/posix-configs/SITL/init/test_dronecourse/${generated_init_script_name})
	# template (replace @test_name@ with ${test_name}
	set(template_init_script ${PX4_SOURCE_DIR}/posix-configs/SITL/init/test_dronecourse/test_dronecourse_template.in)
	configure_file(${template_init_script} ${generated_init_script})

	# Check color support for colorized output
	# we assume that the shell used for testing has same color support
	# as the shell used for running cmake
	execute_process(COMMAND tput colors OUTPUT_VARIABLE color_support)
	if(color_support GREATER 7)
		string(ASCII 27 ESCAPE)
		set(colorized_output_pipe
			| sed -e 's/\\\([A-Z[:space:]]*PASSED\\\)/${ESCAPE}[32m\\1${ESCAPE}[0m/;s/\\\([A-Z[:space:]]*FAILED\\\)/${ESCAPE}[31m\\1${ESCAPE}[0m/'
		)
	else()
		set(colorized_output_pipe '')
	endif()

	# Intercept output and change error code depending on tests result:
	set(error_code_pipe
		| awk 'BEGIN{failed=0}{print $$0\\\; if(match($$0, \"${test_name}.*FAILED\")) failed=1}END{print \"test exit status: \" failed \\\;exit failed}'
	)

	# create the test target
	add_custom_target(test_${test_name}
		COMMAND ${PX4_SOURCE_DIR}/Tools/sitl_run.sh
			# sitl binary file
			$<TARGET_FILE:px4>
			# posix-configs init scripts directory
			posix-configs/SITL/init/test_dronecourse
			# debugger
			none
			# simulation program
			none
			# the name of the generated init script
			${generated_init_script_name}
			# source and binary directory
			${PX4_SOURCE_DIR}
			${PX4_BINARY_DIR} ${colorized_output_pipe} ${error_code_pipe}
		DEPENDS ${template_init_script} ${generated_init_script}
		WORKING_DIRECTORY ${SITL_WORKING_DIR}
	)
endfunction()

dronecourse_add_test(all)
dronecourse_add_test(help)
